#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <RTClib.h>
#include <HardwareSerial.h>

constexpr const char* AP_SSID = "ESP-MONITORING";
constexpr const char* AP_PASSWORD = "12345678";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

constexpr uint8_t PZEM_RX_PIN = 26;
constexpr uint8_t PZEM_TX_PIN = 27;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t SIM800_RX_PIN = 17;
constexpr uint8_t SIM800_TX_PIN = 16;
constexpr uint8_t RELAY1_PIN = 4;
constexpr uint8_t RELAY2_PIN = 32;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
RTC_DS3231 rtc;
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
HardwareSerial sim800(1);

bool rtcAvailable = false;
bool relay1State = false;
bool relay2State = false;

float lastVoltage = 0.0f;
float lastCurrent = 0.0f;
float lastPower = 0.0f;
float lastEnergy = 0.0f;
char lastTime[9] = "00:00:00";

String simLastMessage = "Aucun message";
String simLastSender = "";
bool simNetworkConnected = false;
String simNetworkStatus = "Non connecte au reseau";

unsigned long lastSampleMs = 0;
unsigned long lastSimPollMs = 0;
constexpr unsigned long SAMPLE_INTERVAL_MS = 1000;
constexpr unsigned long SIM_POLL_INTERVAL_MS = 5000;

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
    .container { width:min(1024px,100%); margin:0 auto; }
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
    .input { width:100%; height:42px; border-radius:10px; background:#0b1220; color:#f8fafc; border:1px solid var(--border); padding:0 10px; }
    .btn { padding:10px 12px; border:none; border-radius:10px; color:#fff; cursor:pointer; }
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
      <div class="label">📶 SIM800</div>
      <div style="display:grid;gap:10px;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));">
        <div>
          <div class="label">État réseau</div>
          <div id="simStatus" class="value" style="font-size:1.2rem;">Non connecté au réseau</div>
        </div>
      </div>
    </section>

    <div class="footer-note">Mise à jour temps réel via WebSocket (fallback HTTP /data disponible).</div>
  </div>

  <script>
    let ws; let reconnectTimer;
    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    const voltageEl = document.getElementById('voltage');
    const currentEl = document.getElementById('current');
    const powerEl = document.getElementById('power');
    const energyEl = document.getElementById('energy');
    const relayAllSwitch = document.getElementById('relayAllSwitch');
    const simStatusEl = document.getElementById('simStatus');

    function updateUI(data) {
      if (typeof data.voltage === 'number') voltageEl.innerHTML = `${data.voltage.toFixed(2)}<span class="unit">V</span>`;
      if (typeof data.current === 'number') currentEl.innerHTML = `${data.current.toFixed(3)}<span class="unit">A</span>`;
      if (typeof data.power === 'number') powerEl.innerHTML = `${data.power.toFixed(1)}<span class="unit">W</span>`;
      if (typeof data.energy === 'number') energyEl.innerHTML = `${data.energy.toFixed(3)}<span class="unit">kWh</span>`;
      if (typeof data.relay1 === 'boolean' && typeof data.relay2 === 'boolean') relayAllSwitch.checked = data.relay1 && data.relay2;
      if (typeof data.simNetworkStatus === 'string') {
        simStatusEl.textContent = data.simNetworkStatus;
        simStatusEl.style.color = data.simNetworkConnected ? '#22c55e' : '#ef4444';
      }
    }

    function setConnected(ok) { statusDot.classList.toggle('connected', ok); statusText.textContent = ok ? 'Connecté' : 'Déconnecté'; }

    function connectWS() {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws';
      ws = new WebSocket(`${proto}://${location.host}/ws`);
      ws.onopen = () => setConnected(true);
      ws.onclose = () => { setConnected(false); clearTimeout(reconnectTimer); reconnectTimer = setTimeout(connectWS, 1200); };
      ws.onerror = () => ws.close();
      ws.onmessage = (event) => { try { updateUI(JSON.parse(event.data)); } catch (_) {} };
    }

    async function setAllRelays(enabled) {
      const state = enabled ? 1 : 0;
      const res = await fetch(`/relay/all/set?state=${state}`);
      updateUI(await res.json());
    }

    relayAllSwitch.addEventListener('change', (e) => setAllRelays(e.target.checked));
    connectWS();
  </script>
</body>
</html>
)rawliteral";

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

String sendSimCommand(const String& cmd, uint32_t timeoutMs = 1200) {
  while (sim800.available()) sim800.read();
  sim800.println(cmd);
  String response;
  const unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (sim800.available()) response += static_cast<char>(sim800.read());
  }
  return response;
}

void setAllRelays(bool enabled) {
  relay1State = enabled;
  relay2State = enabled;
  digitalWrite(RELAY1_PIN, relay1State ? HIGH : LOW);
  digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
}

void updateLatestSimMessage() {
  const String listResp = sendSimCommand("AT+CMGL=\"ALL\"", 1500);
  int pos = 0;
  int newestIdx = -1;
  String newestFrom;
  String newestText;

  while (true) {
    int header = listResp.indexOf("+CMGL:", pos);
    if (header < 0) break;

    int lineEnd = listResp.indexOf('\n', header);
    if (lineEnd < 0) break;

    const String meta = listResp.substring(header, lineEnd);
    const int idxSep = meta.indexOf(':');
    const int comma = meta.indexOf(',', idxSep + 1);
    const int msgIndex = (idxSep >= 0 && comma > idxSep) ? meta.substring(idxSep + 1, comma).toInt() : -1;

    const int q1 = meta.indexOf('"');
    const int q2 = (q1 >= 0) ? meta.indexOf('"', q1 + 1) : -1;
    const int q3 = (q2 >= 0) ? meta.indexOf('"', q2 + 1) : -1;
    const int q4 = (q3 >= 0) ? meta.indexOf('"', q3 + 1) : -1;
    const String from = (q3 >= 0 && q4 > q3) ? meta.substring(q3 + 1, q4) : "";

    int msgStart = lineEnd + 1;
    while (msgStart < listResp.length() && (listResp[msgStart] == '\r' || listResp[msgStart] == '\n')) msgStart++;

    int msgEnd = listResp.indexOf("\r\n", msgStart);
    if (msgEnd < 0) msgEnd = listResp.indexOf('\n', msgStart);
    if (msgEnd < 0) msgEnd = listResp.length();

    const String msgText = listResp.substring(msgStart, msgEnd);
    if (msgIndex > newestIdx) {
      newestIdx = msgIndex;
      newestFrom = from;
      newestText = msgText;
    }
    pos = msgEnd;
  }

  if (newestIdx >= 0) {
    simLastSender = sanitizeJson(newestFrom);
    simLastMessage = sanitizeJson(newestText);
    sendSimCommand("AT+CMGD=1,3", 700); // garde seulement le dernier SMS
  } else {
    simLastSender = "";
    simLastMessage = "Aucun message";
  }
}

void updateSimNetworkStatus() {
  const String regResp = sendSimCommand("AT+CREG?", 1200);
  const bool registeredHome = regResp.indexOf("+CREG: 0,1") >= 0 || regResp.indexOf("+CREG: 1,1") >= 0;
  const bool registeredRoaming = regResp.indexOf("+CREG: 0,5") >= 0 || regResp.indexOf("+CREG: 1,5") >= 0;
  simNetworkConnected = registeredHome || registeredRoaming;
  simNetworkStatus = simNetworkConnected ? "Connecte au reseau" : "Non connecte au reseau";
}

void initSim800() {
  sim800.begin(9600, SERIAL_8N1, SIM800_RX_PIN, SIM800_TX_PIN);
  sendSimCommand("AT");
  sendSimCommand("ATE0");
  sendSimCommand("AT+CMGF=1");
  sendSimCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"");
  sendSimCommand("AT+CNMI=1,1,0,0,0");
  updateSimNetworkStatus();
  updateLatestSimMessage();
}

void simSendSms(const String& number, const String& message) {
  sendSimCommand("AT+CMGF=1");
  sendSimCommand("AT+CMGS=\"" + number + "\"");
  sim800.print(message);
  sim800.write(26);
}

void simCallNumber(const String& number) {
  sendSimCommand("ATD" + number + ";", 800);
}

String getTimeString() {
  if (!rtcAvailable) return "00:00:00";
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

  if (!isnan(v)) lastVoltage = v;
  if (!isnan(c)) lastCurrent = c;
  if (!isnan(p)) lastPower = p;
  if (!isnan(e)) lastEnergy = e;

  String t = getTimeString();
  t.toCharArray(lastTime, sizeof(lastTime));
}

String buildCurrentJson() {
  const String safeMsg = sanitizeJson(simLastMessage);
  const String safeFrom = sanitizeJson(simLastSender);
  char payload[520];
  snprintf(payload, sizeof(payload),
           "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.1f,\"energy\":%.3f,\"time\":\"%s\",\"relay1\":%s,\"relay2\":%s,\"simLastMessage\":\"%s\",\"simLastSender\":\"%s\",\"simNetworkConnected\":%s,\"simNetworkStatus\":\"%s\"}",
           lastVoltage, lastCurrent, lastPower, lastEnergy, lastTime,
           relay1State ? "true" : "false", relay2State ? "true" : "false",
           safeMsg.c_str(), safeFrom.c_str(),
           simNetworkConnected ? "true" : "false", sanitizeJson(simNetworkStatus).c_str());
  return String(payload);
}

void pushHistory(float v, float c, float p, float e, const char* t) {
  Sample s{v, c, p, e, ""};
  strlcpy(s.timeStr, t, sizeof(s.timeStr));
  historyBuffer[historyHead] = s;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

String buildHistoryJson() {
  String json = "[";
  for (size_t i = 0; i < historyCount; i++) {
    size_t idx = (historyHead + HISTORY_SIZE - historyCount + i) % HISTORY_SIZE;
    const Sample& s = historyBuffer[idx];
    char row[170];
    snprintf(row, sizeof(row), "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.1f,\"energy\":%.3f,\"time\":\"%s\"}", s.voltage, s.current, s.power, s.energy, s.timeStr);
    json += row;
    if (i + 1 < historyCount) json += ",";
  }
  json += "]";
  return json;
}

void notifyClients() { ws.textAll(buildCurrentJson()); }

void onWsEvent(AsyncWebSocket* wsServer, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  (void)wsServer;
  (void)arg;
  (void)data;
  (void)len;
  if (type == WS_EVT_CONNECT) client->text(buildCurrentJson());
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
  setAllRelays(false);

  initSim800();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.printf("[WiFi] AP actif: %s\n", AP_SSID);
  Serial.printf("[WiFi] IP locale: %s\n", WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { request->send_P(200, "text/html", INDEX_HTML); });
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(200, "application/json", buildCurrentJson()); });
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(200, "application/json", buildHistoryJson()); });

  server.on("/relay/all/set", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("state")) setAllRelays(request->getParam("state")->value().toInt() != 0);
    request->send(200, "application/json", buildCurrentJson());
  });

  server.on("/sim/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateSimNetworkStatus();
    updateLatestSimMessage();
    request->send(200, "application/json", buildCurrentJson());
  });

  server.on("/sim/call", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("number")) simCallNumber(request->getParam("number")->value());
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/sim/sms", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("number") && request->hasParam("message")) {
      simSendSms(request->getParam("number")->value(), request->getParam("message")->value());
    }
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.onNotFound([](AsyncWebServerRequest* request) { request->send(404, "application/json", "{\"error\":\"Not found\"}"); });

  server.begin();
}

void loop() {
  const unsigned long nowMs = millis();

  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = nowMs;
    readSensors();
    pushHistory(lastVoltage, lastCurrent, lastPower, lastEnergy, lastTime);
    notifyClients();
  }

  if (nowMs - lastSimPollMs >= SIM_POLL_INTERVAL_MS) {
    lastSimPollMs = nowMs;
    updateSimNetworkStatus();
    updateLatestSimMessage();
    notifyClients();
  }

  ws.cleanupClients();
}
