#include "WebUI.hpp"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "FanControl.hpp"

static AsyncWebServer server(80);

static const char* WIFI_SSID = "Fezora";
static const char* WIFI_PASS = "autoskap";
static const char* AP_SSID   = "FanBot";
static const char* AP_PASS   = "fanbot1234";

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Fan Control</title>
<style>
body { font-family: sans-serif; padding: 20px; max-width: 520px; margin: 0 auto; }
input[type=range] { width: 100%; }
.switch { position: relative; display: inline-block; width: 60px; height: 34px; }
.switch input { opacity: 0; width: 0; height: 0; }
.slider { position: absolute; cursor: pointer; inset: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
.slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px;
                 background-color: white; transition: .4s; border-radius: 50%; }
input:checked + .slider { background-color: #2196F3; }
input:checked + .slider:before { transform: translateX(26px); }
button { background: #d32f2f; color: white; border: 0; padding: 10px 20px; border-radius: 6px; font-size: 16px; cursor: pointer; }
#stat { font-family: monospace; background: #eee; padding: 10px; border-radius: 6px; white-space: pre-line; }
</style></head><body>

<h2>Enable fans</h2>
<label class="switch">
  <input type="checkbox" id="drive" onchange="toggle(this.checked)">
  <span class="slider"></span>
</label>
<button onclick="stopAll()">STOP</button>

<div id="sliders"></div>
<h3>Status</h3>
<div id="stat">...</div>

<script>
const sliders = [
  ['forward', 'Forward (along line)', 0, 100, 100, 2],
  ['kp',      'Kp',                   0, 500, 100, 2],
  ['ki',      'Ki',                   0, 300, 100, 2],
  ['kd',      'Kd',                   0, 300, 1000, 3]
];

const box = document.getElementById('sliders');
for (const [k, l, mn, mx, sc, dec] of sliders) {
  box.insertAdjacentHTML('beforeend',
    `<h4>${l}: <span id="${k}Val">0</span></h4>
     <input type="range" id="${k}" min="${mn}" max="${mx}" value="0"
            oninput="setVal('${k}', this.value, ${sc}, ${dec})">`);
}

function setVal(k, raw, sc, dec) {
  const v = raw / sc;
  document.getElementById(k + 'Val').innerText = v.toFixed(dec);
  fetch('/set?k=' + k + '&v=' + v);
}
function toggle(on) { fetch('/toggle?state=' + (on ? 1 : 0)); }
function stopAll() { document.getElementById('drive').checked = false; toggle(false); }

async function poll() {
  try {
    const s = await (await fetch('/status')).json();
    document.getElementById('stat').innerText =
      `Line:  ${s.line.toFixed(1)} deg\n` +
      `Dev:   ${s.dev.toFixed(1)} deg  (len ${s.len.toFixed(2)})\n` +
      `Cmd:   ${s.cmd.toFixed(1)} deg  (len ${s.cmdLen.toFixed(2)})\n` +
      `Fans:  ${s.f.map(x => x.toFixed(2)).join(', ')}`;
  } catch (e) {}
  setTimeout(poll, 250);
}

async function init() {
  const s = await (await fetch('/status')).json();
  document.getElementById('drive').checked = s.en;
  for (const [k, l, mn, mx, sc, dec] of sliders) {
    document.getElementById(k).value = Math.round(s[k] * sc);
    document.getElementById(k + 'Val').innerText = s[k].toFixed(dec);
  }
  poll();
}
init();
</script>
</body></html>
)rawliteral";

static const char* wifiStatusStr(uint8_t status) {
    switch (status) {
        case WL_IDLE_STATUS:     return "IDLE";
        case WL_NO_SSID_AVAIL:   return "NO_SSID";
        case WL_SCAN_COMPLETED:  return "SCAN";
        case WL_CONNECTED:       return "CONN";
        case WL_CONNECT_FAILED:  return "FAIL";
        case WL_CONNECTION_LOST: return "LOST";
        case WL_DISCONNECTED:    return "DISC";
        default:                 return "???";
    }
}

void initWeb() {
    delay(200);
    
    Serial.println("[WiFi] Starting...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.persistent(false);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    
    // Scan first — this somehow makes subsequent connection reliable on S3
    Serial.println("[WiFi] Scanning...");
    int n = WiFi.scanNetworks();
    Serial.print(n); Serial.println(" networks found");
    bool found = false;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == WIFI_SSID) {
            found = true;
            Serial.print("  Found "); Serial.print(WIFI_SSID);
            Serial.print(" ("); Serial.print(WiFi.RSSI(i)); Serial.println(" dBm)");
        }
    }
    WiFi.scanDelete();
    
    if (!found) {
        Serial.print(WIFI_SSID); Serial.println(" not found!");
    }
    
    Serial.print("[WiFi] Connecting to "); Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    uint32_t t0 = millis();
    uint8_t lastStatus = 255;
    bool connected = false;
    
    while (true) {
        uint8_t status = WiFi.status();
        
        if (status != lastStatus) {
            Serial.print(" ["); Serial.print(wifiStatusStr(status)); Serial.print("]");
            lastStatus = status;
        } else {
            Serial.print(".");
        }
        
        if (status == WL_CONNECTED) {
            connected = true;
            break;
        }
        
        if (millis() - t0 >= 15000) {
            Serial.println(" TIMEOUT");
            break;
        }
        
        delay(300);
        yield();
    }
    
    if (connected) {
        Serial.print("\n[WiFi] Connected, IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WiFi] RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    } else {
        Serial.println("[WiFi] Starting AP mode...");
        WiFi.mode(WIFI_OFF);
        delay(100);
        WiFi.mode(WIFI_AP);
        
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                          IPAddress(192, 168, 4, 1),
                          IPAddress(255, 255, 255, 0));
        
        bool apOk = WiFi.softAP(AP_SSID, AP_PASS, 6);
        
        if (apOk) {
            Serial.print("[WiFi] AP '"); Serial.print(AP_SSID);
            Serial.print("' started, IP: ");
            Serial.println(WiFi.softAPIP());
        } else {
            Serial.println("[WiFi] AP failed!");
        }
    }
    
    // ============== WEB ROUTES ==============
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", INDEX_HTML);
    });
    
    server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest* r) {
        if (!r->hasParam("state")) {
            r->send(400, "text/plain", "missing state");
            return;
        }
        fanParams.enabled = r->getParam("state")->value() == "1";
        r->send(200, "text/plain", fanParams.enabled ? "ON" : "OFF");
    });
    
    server.on("/set", HTTP_GET, [](AsyncWebServerRequest* r) {
        if (!r->hasParam("k") || !r->hasParam("v")) {
            r->send(400, "text/plain", "missing k or v");
            return;
        }
        
        String k = r->getParam("k")->value();
        float v = r->getParam("v")->value().toFloat();
        bool ok = true;
        
        if      (k == "forward") fanParams.forward = constrain(v, 0.0f, 1.0f);
        else if (k == "kp")      fanParams.kp      = constrain(v, 0.0f, 10.0f);
        else if (k == "ki")      fanParams.ki      = constrain(v, 0.0f, 10.0f);
        else if (k == "kd")      fanParams.kd      = constrain(v, 0.0f, 10.0f);
        else                     ok = false;
        
        if (ok) {
            r->send(200, "text/plain", "OK");
        } else {
            r->send(400, "text/plain", "unknown key: " + k);
        }
    });
    
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r) {
        FanState s = getFanState();
        const float R2D = 180.0f / PI;
        
        char buf[400];
        int n = snprintf(buf, sizeof(buf),
            "{\"en\":%s,\"forward\":%.3f,\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.4f,"
            "\"line\":%.1f,\"dev\":%.1f,\"len\":%.3f,\"cmd\":%.1f,\"cmdLen\":%.3f,"
            "\"f\":[%.2f,%.2f,%.2f]}",
            fanParams.enabled ? "true" : "false",
            (float)fanParams.forward, (float)fanParams.kp,
            (float)fanParams.ki, (float)fanParams.kd,
            s.lineAngle * R2D, s.angle * R2D, s.length,
            s.cmdAngle * R2D, s.cmdLength,
            s.thrust[0], s.thrust[1], s.thrust[2]);
        
        if (n < 0 || n >= (int)sizeof(buf)) {
            r->send(500, "text/plain", "buffer overflow");
            return;
        }
        r->send(200, "application/json", buf);
    });
    
    server.begin();
    Serial.println("[Web] Server started on port 80");
}