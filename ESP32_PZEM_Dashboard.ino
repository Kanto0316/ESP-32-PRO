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
  char timeStr[9];
};

constexpr size_t HISTORY_SIZE = 120;
Sample historyBuffer[HISTORY_SIZE];
size_t historyCount = 0;
size_t historyHead = 0;

float lastVoltage = 0.0f;
float lastCurrent = 0.0f;
char lastTime[9] = "00:00:00";
bool relayState = false; // Préparation future pour relais

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
      transition: transform 0.22s ease, color 0.22s ease;
      will-change: transform;
    }

    .unit {
      font-size: 1rem;
      color: var(--muted);
      margin-left: 6px;
      font-weight: 500;
    }

    .pulse {
      transform: scale(1.03);
      color: var(--accent);
    }

    .toolbar {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }

    button {
      border: 1px solid var(--border);
      background: #1f2937;
      color: var(--text);
      padding: 10px 14px;
      border-radius: 10px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s ease;
    }

    button:hover {
      border-color: rgba(129,140,248,0.65);
      transform: translateY(-1px);
    }

    .relay-state {
      color: var(--muted);
      font-size: 0.92rem;
      align-self: center;
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
        <button id="relayBtn" type="button">Basculer ON/OFF</button>
        <button id="historyBtn" type="button">Tester /history</button>
        <span id="relayState" class="relay-state">Relais: OFF</span>
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
        <div class="label">🕒 Heure RTC</div>
        <div id="time" class="value">00:00:00</div>
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
    const timeEl = document.getElementById('time');
    const relayBtn = document.getElementById('relayBtn');
    const relayStateEl = document.getElementById('relayState');

    const animate = (el) => {
      el.classList.add('pulse');
      setTimeout(() => el.classList.remove('pulse'), 180);
    };

    function updateUI(data) {
      if (typeof data.voltage === 'number') {
        voltageEl.innerHTML = `${data.voltage.toFixed(2)}<span class="unit">V</span>`;
        animate(voltageEl);
      }
      if (typeof data.current === 'number') {
        currentEl.innerHTML = `${data.current.toFixed(3)}<span class="unit">A</span>`;
        animate(currentEl);
      }
      if (typeof data.time === 'string') {
        timeEl.textContent = data.time;
        animate(timeEl);
      }
      if (typeof data.relay === 'boolean') {
        relayStateEl.textContent = `Relais: ${data.relay ? 'ON' : 'OFF'}`;
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

    async function toggleRelay() {
      try {
        const res = await fetch('/relay/toggle');
        const data = await res.json();
        updateUI(data);
      } catch (e) {
        console.error('Erreur relay:', e);
      }
    }

    async function checkHistory() {
      try {
        const res = await fetch('/history');
        const data = await res.json();
        alert(`Historique prêt: ${data.length} échantillon(s)`);
      } catch (e) {
        alert('Impossible de lire /history');
      }
    }

    relayBtn.addEventListener('click', toggleRelay);
    document.getElementById('historyBtn').addEventListener('click', checkHistory);

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

  if (!isnan(v)) {
    lastVoltage = v;
  }
  if (!isnan(c)) {
    lastCurrent = c;
  }

  String t = getTimeString();
  t.toCharArray(lastTime, sizeof(lastTime));
}

String buildCurrentJson() {
  char payload[160];
  snprintf(
    payload,
    sizeof(payload),
    "{\"voltage\":%.2f,\"current\":%.3f,\"time\":\"%s\",\"relay\":%s}",
    lastVoltage,
    lastCurrent,
    lastTime,
    relayState ? "true" : "false"
  );
  return String(payload);
}

void pushHistory(float v, float c, const char* t) {
  Sample s;
  s.voltage = v;
  s.current = c;
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

    char row[120];
    snprintf(row, sizeof(row), "{\"voltage\":%.2f,\"current\":%.3f,\"time\":\"%s\"}", s.voltage, s.current, s.timeStr);
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

  // Bonus: route de préparation commande relais
  server.on("/relay/toggle", HTTP_GET, [](AsyncWebServerRequest* request) {
    relayState = !relayState;
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
    pushHistory(lastVoltage, lastCurrent, lastTime);
    notifyClients();

    Serial.printf("[DATA] U=%.2f V | I=%.3f A | T=%s\n", lastVoltage, lastCurrent, lastTime);
  }

  ws.cleanupClients();
}
