#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>

// ---------------------------
// Configuration
// ---------------------------
constexpr const char* AP_SSID = "ESP-MONITORING";
constexpr const char* AP_PASSWORD = "12345678";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

constexpr uint8_t PZEM_RX_PIN = 26;
constexpr uint8_t PZEM_TX_PIN = 27;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t RELAY1_PIN = 4;
constexpr uint8_t RELAY2_PIN = 32;

// SD (SPI) pins
constexpr uint8_t SD_MISO_PIN = 19;
constexpr uint8_t SD_MOSI_PIN = 23;
constexpr uint8_t SD_SCK_PIN = 18;
constexpr uint8_t SD_CS_PIN = 5;
constexpr const char* LOG_FILE_PATH = "/log.csv";
constexpr const char* CHART_FILE_PATH = "/chart.js";

constexpr unsigned long SENSOR_INTERVAL_MS = 1000;
constexpr unsigned long LOG_INTERVAL_MS = 5000;
constexpr size_t HISTORY_SIZE = 120;

// ---------------------------
// Globals
// ---------------------------
AsyncWebServer server(80);
RTC_DS3231 rtc;
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
SPIClass sdSpi(VSPI);

bool rtcAvailable = false;
bool sdReady = false;
bool spiffsReady = false;
bool relay1State = false;
bool relay2State = false;

float lastVoltage = 0.0f;
float lastCurrent = 0.0f;
float lastPower = 0.0f;
float lastEnergy = 0.0f;

unsigned long lastSensorMs = 0;
unsigned long lastLogMs = 0;

struct Sample {
  char dateTime[24];
  float voltage;
  float current;
  float power;
  float energy;
};

Sample historyBuffer[HISTORY_SIZE];
size_t historyCount = 0;
size_t historyHead = 0;

// ---------------------------
// Utility / Formatting
// ---------------------------
String sanitizeJson(const String& input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    const char ch = input[i];
    if (ch == '"' || ch == '\\') {
      out += '\\';
      out += ch;
    } else if (ch == '\n' || ch == '\r') {
      out += ' ';
    } else {
      out += ch;
    }
  }
  return out;
}

String getDateTimeString() {
  if (rtcAvailable) {
    DateTime now = rtc.now();
    char buffer[24];
    snprintf(buffer,
             sizeof(buffer),
             "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(),
             now.month(),
             now.day(),
             now.hour(),
             now.minute(),
             now.second());
    return String(buffer);
  }

  // fallback if RTC unavailable
  const unsigned long ms = millis();
  const unsigned long totalSec = ms / 1000;
  const unsigned long h = (totalSec / 3600) % 24;
  const unsigned long m = (totalSec / 60) % 60;
  const unsigned long s = totalSec % 60;
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "millis:%lu (%02lu:%02lu:%02lu)", ms, h, m, s);
  return String(buffer);
}

void setAllRelays(bool enabled) {
  relay1State = enabled;
  relay2State = enabled;
  digitalWrite(RELAY1_PIN, relay1State ? HIGH : LOW);
  digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
}

// ---------------------------
// PZEM module
// ---------------------------
void readPzemData() {
  const float v = pzem.voltage();
  const float c = pzem.current();
  const float p = pzem.power();
  const float e = pzem.energy();

  if (!isnan(v)) lastVoltage = v;
  if (!isnan(c)) lastCurrent = c;
  if (!isnan(p)) lastPower = p;
  if (!isnan(e)) lastEnergy = e;
}

void pushHistorySample(const String& dateTime, float v, float c, float p, float e) {
  Sample sample{};
  dateTime.toCharArray(sample.dateTime, sizeof(sample.dateTime));
  sample.voltage = v;
  sample.current = c;
  sample.power = p;
  sample.energy = e;

  historyBuffer[historyHead] = sample;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) {
    historyCount++;
  }
}

String buildCurrentJson() {
  String payload = "{";
  payload += "\"datetime\":\"" + sanitizeJson(getDateTimeString()) + "\"";
  payload += ",\"voltage\":" + String(lastVoltage, 2);
  payload += ",\"current\":" + String(lastCurrent, 3);
  payload += ",\"power\":" + String(lastPower, 1);
  payload += ",\"energy\":" + String(lastEnergy, 3);
  payload += ",\"relay1\":" + String(relay1State ? "true" : "false");
  payload += ",\"relay2\":" + String(relay2State ? "true" : "false");
  payload += ",\"sdReady\":" + String(sdReady ? "true" : "false");
  payload += "}";
  return payload;
}

String buildHistoryJson() {
  String json = "[";
  for (size_t i = 0; i < historyCount; i++) {
    const size_t idx = (historyHead + HISTORY_SIZE - historyCount + i) % HISTORY_SIZE;
    const Sample& s = historyBuffer[idx];
    json += "{\"datetime\":\"" + sanitizeJson(String(s.dateTime)) + "\"";
    json += ",\"voltage\":" + String(s.voltage, 2);
    json += ",\"current\":" + String(s.current, 3);
    json += ",\"power\":" + String(s.power, 1);
    json += ",\"energy\":" + String(s.energy, 3);
    json += "}";
    if (i + 1 < historyCount) {
      json += ",";
    }
  }
  json += "]";
  return json;
}

// ---------------------------
// SD module
// ---------------------------
bool ensureLogFile() {
  if (!sdReady) {
    return false;
  }

  if (!SD.exists(LOG_FILE_PATH)) {
    File file = SD.open(LOG_FILE_PATH, FILE_WRITE);
    if (!file) {
      Serial.println("[SD] Erreur: impossible de creer log.csv");
      return false;
    }
    file.println("date,voltage,current,power,energy");
    file.close();
    Serial.println("[SD] log.csv cree avec en-tete");
  }
  return true;
}

void initSdCard() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdReady = SD.begin(SD_CS_PIN, sdSpi);
  if (!sdReady) {
    Serial.println("[SD] Carte SD absente ou initialisation echouee");
    return;
  }
  Serial.println("[SD] Carte SD initialisee");
  ensureLogFile();
}

void initSpiffs() {
  spiffsReady = SPIFFS.begin(true);
  if (!spiffsReady) {
    Serial.println("[SPIFFS] Initialisation echouee");
    return;
  }
  Serial.println("[SPIFFS] Initialisee");
}

bool appendLogLine(const String& dateTime, float v, float c, float p, float e) {
  if (!sdReady) {
    return false;
  }
  if (!ensureLogFile()) {
    return false;
  }

  File file = SD.open(LOG_FILE_PATH, FILE_APPEND);
  if (!file) {
    Serial.println("[SD] Erreur: fichier log.csv non accessible en ecriture");
    return false;
  }

  char line[128];
  snprintf(line, sizeof(line), "%s,%.2f,%.3f,%.1f,%.3f", dateTime.c_str(), v, c, p, e);
  file.println(line);
  file.close();
  return true;
}

String readLogsAsText(size_t maxBytes = 8192) {
  if (!sdReady) {
    return "error:sd_not_ready";
  }

  File file = SD.open(LOG_FILE_PATH, FILE_READ);
  if (!file) {
    return "error:file_not_accessible";
  }

  String out;
  out.reserve(maxBytes + 16);
  while (file.available() && out.length() < maxBytes) {
    out += static_cast<char>(file.read());
  }
  file.close();
  return out;
}

String readLogsAsJson(size_t maxRows = 200) {
  if (!sdReady) {
    return "{\"ok\":false,\"error\":\"sd_not_ready\",\"rows\":[]}";
  }

  File file = SD.open(LOG_FILE_PATH, FILE_READ);
  if (!file) {
    return "{\"ok\":false,\"error\":\"file_not_accessible\",\"rows\":[]}";
  }

  String json = "{\"ok\":true,\"rows\":[";
  String line;
  bool headerSkipped = false;
  size_t rowCount = 0;

  while (file.available() && rowCount < maxRows) {
    line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }

    if (!headerSkipped) {
      headerSkipped = true;
      continue;
    }

    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    int p4 = line.indexOf(',', p3 + 1);

    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
      continue;
    }

    String date = line.substring(0, p1);
    String voltage = line.substring(p1 + 1, p2);
    String current = line.substring(p2 + 1, p3);
    String power = line.substring(p3 + 1, p4);
    String energy = line.substring(p4 + 1);

    if (rowCount > 0) {
      json += ",";
    }
    json += "{\"date\":\"" + sanitizeJson(date) + "\"";
    json += ",\"voltage\":" + voltage;
    json += ",\"current\":" + current;
    json += ",\"power\":" + power;
    json += ",\"energy\":" + energy;
    json += "}";
    rowCount++;
  }

  file.close();
  json += "]}";
  return json;
}

// ---------------------------
// Web module
// ---------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>ESP Monitoring</title>
  <style>
    :root { --bg:#0f172a; --card:#111827; --muted:#9ca3af; --text:#f8fafc; --ok:#22c55e; --off:#ef4444; --border:rgba(255,255,255,0.08); }
    * { box-sizing:border-box; }
    body { margin:0; min-height:100vh; font-family:Inter,Segoe UI,Roboto,Arial,sans-serif; color:var(--text); background:var(--bg); padding:18px; }
    .container { width:min(1100px,100%); margin:0 auto; }
    .header { display:flex; justify-content:space-between; align-items:center; gap:12px; margin-bottom:18px; flex-wrap:wrap; }
    .title { margin:0; font-size:clamp(1.3rem,2vw,1.8rem); }
    .status { color:var(--muted); font-size:0.92rem; display:flex; align-items:center; gap:8px; }
    .dot { width:10px; height:10px; border-radius:50%; background:var(--off); transition:background .3s ease; }
    .dot.connected { background:var(--ok); }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:14px; margin-bottom:14px; }
    .card { background:#111827; border:1px solid var(--border); border-radius:16px; padding:18px; }
    .label { color:var(--muted); font-size:.9rem; margin-bottom:10px; }
    .value { font-size:clamp(1.5rem,4.3vw,2.2rem); font-weight:700; line-height:1.1; }
    .unit { font-size:1rem; color:var(--muted); margin-left:6px; font-weight:500; }
    .toolbar { display:flex; gap:10px; flex-wrap:wrap; }
    .relay-control { display:inline-flex; align-items:center; gap:10px; padding:8px 12px; border-radius:12px; border:1px solid var(--border); background:#111827; }
    .relay-label { font-size:.92rem; font-weight:600; }
    .switch { position:relative; display:inline-block; width:52px; height:28px; }
    .switch input { opacity:0; width:0; height:0; }
    .slider { position:absolute; cursor:pointer; inset:0; background-color:#374151; transition:.2s; border-radius:999px; border:1px solid rgba(255,255,255,.08); }
    .slider:before { position:absolute; content:""; height:20px; width:20px; left:3px; top:3px; background:#f8fafc; transition:.2s; border-radius:50%; }
    .switch input:checked + .slider { background-color:#14b8a6; }
    .switch input:checked + .slider:before { transform:translateX(24px); }
    .footer-note { margin-top:10px; color:var(--muted); font-size:.85rem; }
    .btn { padding:10px 12px; border:none; border-radius:10px; color:#fff; cursor:pointer; background:#0ea5e9; }
    .muted { color:var(--muted); font-size:.88rem; }
    .table-wrap { overflow:auto; max-height:340px; border:1px solid var(--border); border-radius:12px; }
    table { width:100%; border-collapse:collapse; font-size:.9rem; }
    th, td { padding:8px 10px; border-bottom:1px solid var(--border); text-align:left; white-space:nowrap; }
    th { position:sticky; top:0; background:#0b1220; z-index:1; }
    .chart-wrap { position:relative; width:100%; min-height:280px; }
    #chart { width:100%; height:300px; }
  </style>
</head>
<body>
  <div class="container">
    <header class="header">
      <div>
        <h1 class="title">📊 Dashboard Énergie ESP32</h1>
        <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">Connexion...</span></div>
      </div>
      <div class="toolbar">
        <div class="relay-control">
          <span class="relay-label">Relais GPIO 4 + 32</span>
          <label class="switch"><input id="relayAllSwitch" type="checkbox" /><span class="slider"></span></label>
        </div>
      </div>
    </header>

    <main class="grid">
      <section class="card"><div class="label">⚡ Tension</div><div id="voltage" class="value">0.00<span class="unit">V</span></div></section>
      <section class="card"><div class="label">🔌 Courant</div><div id="current" class="value">0.000<span class="unit">A</span></div></section>
      <section class="card"><div class="label">🔥 Puissance</div><div id="power" class="value">0.0<span class="unit">W</span></div></section>
      <section class="card"><div class="label">🧮 Conso totale</div><div id="energy" class="value">0.000<span class="unit">kWh</span></div></section>
    </main>

    <section class="card" style="margin-top:14px;">
      <div style="display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;">
        <div>
          <div class="label">📚 Historique (SD /log.csv)</div>
          <div id="sdStatus" class="muted">Statut SD: inconnu</div>
        </div>
        <a class="btn" href="/download" download>Télécharger CSV</a>
      </div>
      <div class="table-wrap" style="margin-top:12px;">
        <table>
          <thead>
            <tr>
              <th>Date</th>
              <th>Tension (V)</th>
              <th>Courant (A)</th>
              <th>Puissance (W)</th>
              <th>Énergie (kWh)</th>
            </tr>
          </thead>
          <tbody id="historyTableBody">
            <tr><td colspan="5" class="muted">Aucune donnée</td></tr>
          </tbody>
        </table>
      </div>
    </section>

    <section class="card" style="margin-top:14px;">
      <div style="display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;">
        <div>
          <div class="label">📈 Graphique temps réel (depuis /log.csv)</div>
          <div class="muted">Source: route <code>/logs</code> (CSV), rafraîchissement 5 s.</div>
        </div>
      </div>
      <div class="chart-wrap" style="margin-top:12px;">
        <canvas id="chart"></canvas>
      </div>
    </section>

    <div class="footer-note">Mise à jour automatique des mesures et de l'historique toutes les 5 secondes.</div>
  </div>

  <script src="/chart.js"></script>
  <script>
    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    const voltageEl = document.getElementById('voltage');
    const currentEl = document.getElementById('current');
    const powerEl = document.getElementById('power');
    const energyEl = document.getElementById('energy');
    const relayAllSwitch = document.getElementById('relayAllSwitch');
    const historyTableBody = document.getElementById('historyTableBody');
    const sdStatus = document.getElementById('sdStatus');
    const chartCanvas = document.getElementById('chart');
    let powerChart = null;

    function setConnected(ok) {
      statusDot.classList.toggle('connected', ok);
      statusText.textContent = ok ? 'Connecté' : 'Déconnecté';
    }

    function updateMetrics(data) {
      if (typeof data.voltage === 'number') voltageEl.innerHTML = `${data.voltage.toFixed(2)}<span class="unit">V</span>`;
      if (typeof data.current === 'number') currentEl.innerHTML = `${data.current.toFixed(3)}<span class="unit">A</span>`;
      if (typeof data.power === 'number') powerEl.innerHTML = `${data.power.toFixed(1)}<span class="unit">W</span>`;
      if (typeof data.energy === 'number') energyEl.innerHTML = `${data.energy.toFixed(3)}<span class="unit">kWh</span>`;
      if (typeof data.relay1 === 'boolean' && typeof data.relay2 === 'boolean') relayAllSwitch.checked = data.relay1 && data.relay2;
      if (typeof data.sdReady === 'boolean') sdStatus.textContent = data.sdReady ? 'Statut SD: prête' : 'Statut SD: absente / erreur';
    }

    function updateHistoryTable(rows) {
      if (!Array.isArray(rows) || rows.length === 0) {
        historyTableBody.innerHTML = '<tr><td colspan="5" class="muted">Aucune donnée</td></tr>';
        return;
      }
      const recentRows = rows.slice(-120).reverse();
      historyTableBody.innerHTML = recentRows.map(r => {
        const d = r.date || r.datetime || '-';
        return `<tr>
          <td>${d}</td>
          <td>${Number(r.voltage || 0).toFixed(2)}</td>
          <td>${Number(r.current || 0).toFixed(3)}</td>
          <td>${Number(r.power || 0).toFixed(1)}</td>
          <td>${Number(r.energy || 0).toFixed(3)}</td>
        </tr>`;
      }).join('');
    }

    function parseCsvLogs(csvText) {
      const lines = csvText.split(/\r?\n/).map(l => l.trim()).filter(Boolean);
      if (lines.length <= 1) {
        return { labels: [], values: [] };
      }

      const labels = [];
      const values = [];

      for (let i = 1; i < lines.length; i++) {
        const parts = lines[i].split(',');
        if (parts.length < 5) continue;
        labels.push(parts[0]);
        values.push(Number(parts[1]));
      }

      const maxPoints = 120;
      if (labels.length > maxPoints) {
        return {
          labels: labels.slice(labels.length - maxPoints),
          values: values.slice(values.length - maxPoints)
        };
      }
      return { labels, values };
    }

    function updateChart(labels, values) {
      if (typeof Chart === 'undefined') {
        sdStatus.textContent = 'Statut SD: chart.js non trouvé (placer /chart.js sur SD ou SPIFFS)';
        return;
      }

      if (!powerChart) {
        powerChart = new Chart(chartCanvas, {
          type: 'line',
          data: {
            labels,
            datasets: [{
              label: 'Tension (V)',
              data: values,
              borderColor: '#38bdf8',
              backgroundColor: 'rgba(56,189,248,0.18)',
              fill: true,
              tension: 0.22,
              pointRadius: 0
            }]
          },
          options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
              x: { ticks: { color: '#cbd5e1', maxTicksLimit: 8 } },
              y: { ticks: { color: '#cbd5e1' }, title: { display: true, text: 'Volt (V)', color: '#cbd5e1' } }
            },
            plugins: { legend: { labels: { color: '#e2e8f0' } } }
          }
        });
        return;
      }

      powerChart.data.labels = labels;
      powerChart.data.datasets[0].data = values;
      powerChart.update('none');
    }

    async function refreshChart() {
      try {
        const logsRes = await fetch('/logs');
        if (!logsRes.ok) throw new Error('logs_unavailable');
        const csv = await logsRes.text();
        const parsed = parseCsvLogs(csv);

        if (!parsed.labels.length) {
          sdStatus.textContent = 'Statut SD: fichier log.csv vide ou sans données';
        }
        updateChart(parsed.labels, parsed.values);
      } catch (_) {
        sdStatus.textContent = 'Statut SD: lecture /logs impossible (carte absente ou erreur)';
      }
    }

    async function refreshData() {
      try {
        const currentRes = await fetch('/data');
        const currentJson = await currentRes.json();
        updateMetrics(currentJson);

        const logsRes = await fetch('/logs?format=json');
        const logsJson = await logsRes.json();
        if (logsJson.ok) {
          updateHistoryTable(logsJson.rows || []);
        } else {
          historyTableBody.innerHTML = `<tr><td colspan="5" class="muted">Erreur logs: ${logsJson.error || 'inconnue'}</td></tr>`;
        }

        setConnected(true);
      } catch (_) {
        setConnected(false);
      }

      await refreshChart();
    }

    async function setAllRelays(enabled) {
      const state = enabled ? 1 : 0;
      const res = await fetch(`/relay/all/set?state=${state}`);
      updateMetrics(await res.json());
    }

    relayAllSwitch.addEventListener('change', (e) => setAllRelays(e.target.checked));

    refreshData();
    setInterval(refreshData, 5000);
  </script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildCurrentJson());
  });

  server.on("/history", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildHistoryJson());
  });

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest* request) {
    const bool asJson = request->hasParam("format") && request->getParam("format")->value() == "json";

    if (asJson) {
      request->send(200, "application/json", readLogsAsJson());
      return;
    }

    const String logText = readLogsAsText();
    if (logText.startsWith("error:")) {
      request->send(500, "text/plain", logText);
      return;
    }
    request->send(200, "text/plain", logText);
  });

  server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (sdReady && SD.exists(CHART_FILE_PATH)) {
      request->send(SD, CHART_FILE_PATH, "application/javascript");
      return;
    }

    if (spiffsReady && SPIFFS.exists(CHART_FILE_PATH)) {
      request->send(SPIFFS, CHART_FILE_PATH, "application/javascript");
      return;
    }

    request->send(404,
                  "application/javascript",
                  "console.error('chart.js absent: placez /chart.js sur SD ou SPIFFS');");
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!sdReady || !SD.exists(LOG_FILE_PATH)) {
      request->send(404, "text/plain", "log.csv non disponible");
      return;
    }
    request->send(SD, LOG_FILE_PATH, "text/csv", true);
  });

  server.on("/relay/all/set", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("state")) {
      setAllRelays(request->getParam("state")->value().toInt() != 0);
    }
    request->send(200, "application/json", buildCurrentJson());
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "application/json", "{\"error\":\"Not found\"}");
  });

  server.begin();
}

// ---------------------------
// Setup / Loop
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  rtcAvailable = rtc.begin();
  if (!rtcAvailable) {
    Serial.println("[RTC] DS3231 non detecte, fallback millis");
  } else if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  setAllRelays(false);

  initSdCard();
  initSpiffs();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.printf("[WiFi] AP actif: %s\n", AP_SSID);
  Serial.printf("[WiFi] IP locale: %s\n", WiFi.softAPIP().toString().c_str());

  setupWebServer();
}

void loop() {
  const unsigned long nowMs = millis();

  if (nowMs - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = nowMs;
    readPzemData();
    pushHistorySample(getDateTimeString(), lastVoltage, lastCurrent, lastPower, lastEnergy);
  }

  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    const String dt = getDateTimeString();
    if (!appendLogLine(dt, lastVoltage, lastCurrent, lastPower, lastEnergy)) {
      // Retry SD init if card was removed/inserted at runtime
      if (!sdReady) {
        initSdCard();
      }
    }
  }
}
