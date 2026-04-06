#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <RTClib.h>

// ------------------------------
// Configuration Wi-Fi Access Point
// ------------------------------
constexpr const char* AP_SSID = "ESP-MONITORING";
constexpr const char* AP_PASSWORD = "12345678";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// ------------------------------
// Pins
// ------------------------------
constexpr uint8_t PZEM_RX_PIN = 26; // TX PZEM -> RX ESP32
constexpr uint8_t PZEM_TX_PIN = 27; // RX PZEM -> TX ESP32
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

// ------------------------------
// Objets globaux
// ------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
RTC_DS3231 rtc;
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
bool rtcAvailable = false;

struct Sample {
  float voltage;
  float current;
  float power;
  float energy;
  char timeStr[9];
};

constexpr size_t HISTORY_SIZE = 120;
Sample historyBuffer[HISTORY_SIZE];
size_t historyCount = 0;
size_t historyHead = 0;

float lastVoltage = 0.0f;
float lastCurrent = 0.0f;
float lastPower = 0.0f;
float lastEnergy = 0.0f;
char lastTime[9] = "00:00:00";
constexpr uint8_t RELAY1_PIN = 4;
constexpr uint8_t RELAY2_PIN = 32;
bool relay1State = false;
bool relay2State = false;

unsigned long lastSampleMs = 0;
constexpr unsigned long SAMPLE_INTERVAL_MS = 1000;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>ESP Monitoring</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #111827;
      --muted: #9ca3af;
      --text: #f8fafc;
      --accent: #22d3ee;
      --accent-2: #818cf8;
      --ok: #22c55e;
      --off: #ef4444;
      --border: rgba(255,255,255,0.08);
      --shadow: 0 12px 30px rgba(0,0,0,0.35);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      font-family: Inter, Segoe UI, Roboto, Arial, sans-serif;
      color: var(--text);
      background:
        radial-gradient(1200px 500px at 10% -10%, rgba(34,211,238,0.16), transparent 60%),
        radial-gradient(800px 500px at 100% 0%, rgba(129,140,248,0.18), transparent 60%),
        var(--bg);
      padding: 18px;
    }

    .container {
      width: min(1024px, 100%);
      margin: 0 auto;
    }

    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      margin-bottom: 18px;
      flex-wrap: wrap;
    }

    .title {
      margin: 0;
      font-size: clamp(1.3rem, 2vw, 1.8rem);
      letter-spacing: 0.3px;
    }

    .status {
      color: var(--muted);
      font-size: 0.92rem;
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--off);
      transition: background 0.3s ease;
    }

    .dot.connected { background: var(--ok); }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 14px;
      margin-bottom: 14px;
    }

    .card {
      background: linear-gradient(160deg, rgba(255,255,255,0.03), rgba(255,255,255,0.01));
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 18px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(4px);
    }

    .label {
      color: var(--muted);
      font-size: 0.9rem;
      margin-bottom: 10px;
    }

    .value {
      font-size: clamp(1.5rem, 4.3vw, 2.2rem);
      font-weight: 700;
      line-height: 1.1;
      transition: color 0.18s ease;
    }

    .unit {
      font-size: 1rem;
      color: var(--muted);
      margin-left: 6px;
      font-weight: 500;
    }

    .toolbar {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }

    .relay-control {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 8px 12px;
      border-radius: 12px;
      border: 1px solid var(--border);
      background: #111827;
    }

    .relay-label {
      font-size: 0.92rem;
      color: var(--text);
      font-weight: 600;
      min-width: 120px;
    }

    .switch {
      position: relative;
      display: inline-block;
      width: 52px;
      height: 28px;
    }

    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }

    .slider {
      position: absolute;
      cursor: pointer;
      inset: 0;
      background-color: #374151;
      transition: .2s;
      border-radius: 999px;
      border: 1px solid rgba(255,255,255,0.08);
    }

    .slider:before {
      position: absolute;
      content: "";
      height: 20px;
      width: 20px;
      left: 3px;
      top: 3px;
      background: #f8fafc;
      transition: .2s;
      border-radius: 50%;
    }

    .switch input:checked + .slider {
      background-color: #14b8a6;
    }

    .switch input:checked + .slider:before {
      transform: translateX(24px);
    }

    .footer-note {
      margin-top: 10px;
      color: var(--muted);
      font-size: 0.85rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <header class="header">
      <div>
        <h1 class="title">📊 Dashboard Énergie ESP32</h1>
        <div class="status">
          <span id="statusDot" class="dot"></span>
          <span id="statusText">Connexion...</span>
        </div>
      </div>
      <div class="toolbar">
        <div class="relay-control">
          <span class="relay-label">Relais GPIO 4</span>
          <label class="switch">
            <input id="relay1Switch" type="checkbox" />
            <span class="slider"></span>
          </label>
        </div>
        <div class="relay-control">
          <span class="relay-label">Relais GPIO 32</span>
          <label class="switch">
            <input id="relay2Switch" type="checkbox" />
            <span class="slider"></span>
          </label>
        </div>
      </div>
    </header>

    <main class="grid">
      <section class="card">
        <div class="label">⚡ Tension</div>
        <div id="voltage" class="value">0.00<span class="unit">V</span></div>
      </section>

      <section class="card">
        <div class="label">🔌 Courant</div>
        <div id="current" class="value">0.000<span class="unit">A</span></div>
      </section>

      <section class="card">
        <div class="label">🔥 Puissance</div>
        <div id="power" class="value">0.0<span class="unit">W</span></div>
      </section>

      <section class="card">
        <div class="label">🧮 Conso totale</div>
        <div id="energy" class="value">0.000<span class="unit">kWh</span></div>
      </section>
    </main>

    <div class="footer-note">Mise à jour temps réel via WebSocket (fallback HTTP /data disponible).</div>
  </div>

  <script>
    let ws;
    let reconnectTimer;

    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    const voltageEl = document.getElementById('voltage');
    const currentEl = document.getElementById('current');
    const powerEl = document.getElementById('power');
    const energyEl = document.getElementById('energy');
    const relay1Switch = document.getElementById('relay1Switch');
    const relay2Switch = document.getElementById('relay2Switch');

    function updateUI(data) {
      if (typeof data.voltage === 'number') {
        voltageEl.innerHTML = `${data.voltage.toFixed(2)}<span class="unit">V</span>`;
      }
      if (typeof data.current === 'number') {
        currentEl.innerHTML = `${data.current.toFixed(3)}<span class="unit">A</span>`;
      }
      if (typeof data.power === 'number') {
        powerEl.innerHTML = `${data.power.toFixed(1)}<span class="unit">W</span>`;
      }
      if (typeof data.energy === 'number') {
        energyEl.innerHTML = `${data.energy.toFixed(3)}<span class="unit">kWh</span>`;
      }
      if (typeof data.relay1 === 'boolean') {
        relay1Switch.checked = data.relay1;
      }
      if (typeof data.relay2 === 'boolean') {
        relay2Switch.checked = data.relay2;
      }
    }

    function setConnected(ok) {
      statusDot.classList.toggle('connected', ok);
      statusText.textContent = ok ? 'Connecté' : 'Déconnecté';
    }

    function connectWS() {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws';
      ws = new WebSocket(`${proto}://${location.host}/ws`);

      ws.onopen = () => {
        setConnected(true);
      };

      ws.onclose = () => {
        setConnected(false);
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connectWS, 1200);
      };

      ws.onerror = () => {
        ws.close();
      };

      ws.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          updateUI(payload);
        } catch (_) {
          // Ignore message malformé
        }
      };
    }

    async function setRelay(relayId, enabled) {
      try {
        const state = enabled ? 1 : 0;
        const res = await fetch(`/relay/${relayId}/set?state=${state}`);
        const data = await res.json();
        updateUI(data);
      } catch (e) {
        console.error('Erreur relay:', e);
      }
    }

    relay1Switch.addEventListener('change', (event) => setRelay(1, event.target.checked));
    relay2Switch.addEventListener('change', (event) => setRelay(2, event.target.checked));
    connectWS();
  </script>
</body>
</html>
)rawliteral";

String getTimeString() {
  if (!rtcAvailable) {
    return "00:00:00";
  }
  DateTime now = rtc.now();
  char buffer[9];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  return String(buffer);
}

void readSensors() {
  float v = pzem.voltage();
  float c = pzem.current();
  float p = pzem.power();
  float e = pzem.energy();

  if (!isnan(v)) {
    lastVoltage = v;
  }
  if (!isnan(c)) {
    lastCurrent = c;
  }
  if (!isnan(p)) {
    lastPower = p;
  }
  if (!isnan(e)) {
    lastEnergy = e;
  }

  String t = getTimeString();
  t.toCharArray(lastTime, sizeof(lastTime));
}

String buildCurrentJson() {
  char payload[260];
  snprintf(
    payload,
    sizeof(payload),
    "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.1f,\"energy\":%.3f,\"time\":\"%s\",\"relay1\":%s,\"relay2\":%s}",
    lastVoltage,
    lastCurrent,
    lastPower,
    lastEnergy,
    lastTime,
    relay1State ? "true" : "false",
    relay2State ? "true" : "false"
  );
  return String(payload);
}

void pushHistory(float v, float c, float p, float e, const char* t) {
  Sample s;
  s.voltage = v;
  s.current = c;
  s.power = p;
  s.energy = e;
  strlcpy(s.timeStr, t, sizeof(s.timeStr));

  historyBuffer[historyHead] = s;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) {
    historyCount++;
  }
}

String buildHistoryJson() {
  String json = "[";
  for (size_t i = 0; i < historyCount; i++) {
    size_t idx = (historyHead + HISTORY_SIZE - historyCount + i) % HISTORY_SIZE;
    const Sample& s = historyBuffer[idx];

    char row[170];
    snprintf(
      row,
      sizeof(row),
      "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.1f,\"energy\":%.3f,\"time\":\"%s\"}",
      s.voltage,
      s.current,
      s.power,
      s.energy,
      s.timeStr
    );
    json += row;
    if (i + 1 < historyCount) {
      json += ",";
    }
  }
  json += "]";
  return json;
}

void notifyClients() {
  ws.textAll(buildCurrentJson());
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  (void)server;
  (void)arg;
  (void)data;
  (void)len;

  if (type == WS_EVT_CONNECT) {
    client->text(buildCurrentJson());
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  rtcAvailable = rtc.begin();
  if (!rtcAvailable) {
    Serial.println("[RTC] DS3231 non detecte");
  } else if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println();
  Serial.printf("[WiFi] AP actif: %s\n", AP_SSID);
  Serial.printf("[WiFi] IP locale: %s\n", WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // Endpoint demandé: /data
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildCurrentJson());
  });

  // Bonus: endpoint /history pour préparer le stockage futur
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildHistoryJson());
  });

  server.on("/relay/1/set", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("state")) {
      relay1State = request->getParam("state")->value().toInt() != 0;
    }
    digitalWrite(RELAY1_PIN, relay1State ? HIGH : LOW);
    request->send(200, "application/json", buildCurrentJson());
  });

  server.on("/relay/2/set", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("state")) {
      relay2State = request->getParam("state")->value().toInt() != 0;
    }
    digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
    request->send(200, "application/json", buildCurrentJson());
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "application/json", "{\"error\":\"Not found\"}");
  });

  server.begin();
}

void loop() {
  const unsigned long nowMs = millis();

  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = nowMs;

    readSensors();
    pushHistory(lastVoltage, lastCurrent, lastPower, lastEnergy, lastTime);
    notifyClients();

    Serial.printf(
      "[DATA] U=%.2f V | I=%.3f A | P=%.1f W | E=%.3f kWh | T=%s\n",
      lastVoltage,
      lastCurrent,
      lastPower,
      lastEnergy,
      lastTime
    );
  }

  ws.cleanupClients();
}
