#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---- Pin lists ----
const uint8_t INPUT_PINS[]    = {4, 12, 13, 14, 15, 16, 17, 32, 33, 34, 35, 36, 39};
const uint8_t NUM_INPUT_PINS  = sizeof(INPUT_PINS);
const uint8_t OUTPUT_PINS[]   = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
const uint8_t NUM_OUTPUT_PINS = sizeof(OUTPUT_PINS);

// ---- Config ----
struct ChannelMap { uint8_t crsfCh; uint8_t pin; };
struct Config {
  uint8_t    crsfRxPin;
  uint8_t    numChannels;
  ChannelMap ch[16];
  int16_t    crsfMin;
  int16_t    crsfMax;
  int16_t    pwmMinUs;
  int16_t    pwmMaxUs;
};

Config cfg;

const Config DEFAULT_CFG = {
  35, 5,
  {{0,18},{1,19},{2,21},{3,22},{4,23}},
  172, 1811, 988, 2012
};

Preferences prefs;

void loadConfig() {
  prefs.begin("crsf", true);
  cfg.crsfRxPin   = prefs.getUChar("rxpin", DEFAULT_CFG.crsfRxPin);
  cfg.numChannels = prefs.getUChar("numch", DEFAULT_CFG.numChannels);
  if (cfg.numChannels == 0 || cfg.numChannels > 16) cfg.numChannels = DEFAULT_CFG.numChannels;
  for (uint8_t i = 0; i < cfg.numChannels; i++) {
    char kc[6], kp[6];
    snprintf(kc, sizeof(kc), "c%d_c", i);
    snprintf(kp, sizeof(kp), "c%d_p", i);
    uint8_t defCh  = i;
    uint8_t defPin = (i < 5) ? DEFAULT_CFG.ch[i].pin : OUTPUT_PINS[0];
    cfg.ch[i].crsfCh = prefs.getUChar(kc, defCh);
    cfg.ch[i].pin    = prefs.getUChar(kp, defPin);
  }
  cfg.crsfMin  = prefs.getShort("crsfMin",  DEFAULT_CFG.crsfMin);
  cfg.crsfMax  = prefs.getShort("crsfMax",  DEFAULT_CFG.crsfMax);
  cfg.pwmMinUs = prefs.getShort("pwmMin",   DEFAULT_CFG.pwmMinUs);
  cfg.pwmMaxUs = prefs.getShort("pwmMax",   DEFAULT_CFG.pwmMaxUs);
  prefs.end();
}

void saveConfig() {
  prefs.begin("crsf", false);
  prefs.putUChar("rxpin", cfg.crsfRxPin);
  prefs.putUChar("numch", cfg.numChannels);
  for (uint8_t i = 0; i < cfg.numChannels; i++) {
    char kc[6], kp[6];
    snprintf(kc, sizeof(kc), "c%d_c", i);
    snprintf(kp, sizeof(kp), "c%d_p", i);
    prefs.putUChar(kc, cfg.ch[i].crsfCh);
    prefs.putUChar(kp, cfg.ch[i].pin);
  }
  prefs.putShort("crsfMin",  cfg.crsfMin);
  prefs.putShort("crsfMax",  cfg.crsfMax);
  prefs.putShort("pwmMin",   cfg.pwmMinUs);
  prefs.putShort("pwmMax",   cfg.pwmMaxUs);
  prefs.end();
}

// ---- HTML generation ----

String pinSelect(const char* name, const uint8_t* pins, uint8_t count, uint8_t sel) {
  String s = "<select name='";
  s += name;
  s += "'>";
  for (uint8_t i = 0; i < count; i++) {
    s += "<option value='";
    s += pins[i];
    s += "'";
    if (pins[i] == sel) s += " selected";
    s += ">GPIO ";
    s += pins[i];
    s += "</option>";
  }
  s += "</select>";
  return s;
}

String crsfSelect(const char* name, uint8_t sel) {
  String s = "<select name='";
  s += name;
  s += "'>";
  for (uint8_t c = 0; c < 16; c++) {
    s += "<option value='";
    s += c;
    s += "'";
    if (c == sel) s += " selected";
    s += ">CH";
    s += c;
    s += "</option>";
  }
  s += "</select>";
  return s;
}

String buildHtml() {
  String html;
  html.reserve(8192);

  html = "<!DOCTYPE html><html><head>"
         "<meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>CRSF-PWM Config</title>"
         "<style>"
         "body{font-family:sans-serif;max-width:620px;margin:2em auto;padding:0 1em}"
         "h1{color:#d44;margin-bottom:.2em}"
         "p{color:#666;margin-top:0}"
         "label{display:block;margin-bottom:1em}"
         "table{border-collapse:collapse;width:100%;margin-bottom:.5em}"
         "th,td{padding:.4em .6em;text-align:left}"
         "th{background:#eee}"
         "tr:nth-child(even){background:#f9f9f9}"
         "select{padding:.3em .4em}"
         ".add{background:#4a4;color:#fff;border:none;padding:.35em .8em;cursor:pointer;border-radius:3px}"
         ".del{background:#c44;color:#fff;border:none;padding:.2em .6em;cursor:pointer;border-radius:3px}"
         ".save{background:#44a;color:#fff;border:none;padding:.6em 1.8em;font-size:1em;cursor:pointer;border-radius:3px;margin-top:1em}"
         "</style></head><body>"
         "<h1>CRSF-PWM</h1>"
         "<p>Connect to <b>CRSF-PWM</b> Wi-Fi &bull; 192.168.4.1</p>"
         "<form method='POST' action='/save'>"
         "<label><b>CRSF RX Pin</b><br>";

  html += pinSelect("rxpin", INPUT_PINS, NUM_INPUT_PINS, cfg.crsfRxPin);

  html += "</label>"
          "<h3 style='margin-bottom:.4em'>Calibration</h3>"
          "<table><thead><tr>"
          "<th>CRSF Min</th><th>CRSF Max</th><th>PWM Min (&micro;s)</th><th>PWM Max (&micro;s)</th>"
          "</tr></thead><tbody><tr><td>"
          "<input type='number' name='crsfMin' min='0' max='2047' style='width:70px' value='";
  html += cfg.crsfMin;
  html += "'></td><td>"
          "<input type='number' name='crsfMax' min='0' max='2047' style='width:70px' value='";
  html += cfg.crsfMax;
  html += "'></td><td>"
          "<input type='number' name='pwmMin' min='500' max='2500' style='width:70px' value='";
  html += cfg.pwmMinUs;
  html += "'></td><td>"
          "<input type='number' name='pwmMax' min='500' max='2500' style='width:70px' value='";
  html += cfg.pwmMaxUs;
  html += "'></td></tr></tbody></table>"
          "<h3 style='margin-bottom:.4em'>Channel Mappings</h3>"
          "<table>"
          "<thead><tr><th>#</th><th>CRSF channel</th><th>Output pin</th><th></th></tr></thead>"
          "<tbody id='rows'>";

  for (uint8_t i = 0; i < cfg.numChannels; i++) {
    char nc[8], np[8];
    snprintf(nc, sizeof(nc), "crsf_%d", i);
    snprintf(np, sizeof(np), "pin_%d",  i);
    html += "<tr><td>";
    html += i;
    html += "</td><td>";
    html += crsfSelect(nc, cfg.ch[i].crsfCh);
    html += "</td><td>";
    html += pinSelect(np, OUTPUT_PINS, NUM_OUTPUT_PINS, cfg.ch[i].pin);
    html += "</td><td><button type='button' class='del' onclick='del(this)'>x</button></td></tr>";
  }

  html += "</tbody></table>"
          "<button type='button' class='add' onclick='add()'>+ Add channel</button>"
          "<input type='hidden' name='numch' id='numch' value='";
  html += cfg.numChannels;
  html += "'>"
          "<br><button type='submit' class='save'>Save &amp; Restart</button>"
          "</form>"
          "<script>"
          "const OP=[";

  for (uint8_t i = 0; i < NUM_OUTPUT_PINS; i++) {
    if (i) html += ",";
    html += OUTPUT_PINS[i];
  }

  html += "];"
          "function pOpts(s){"
          "return OP.map(p=>`<option value=\"${p}\"${p==s?' selected':''}>${p}</option>`).join('');"
          "}"
          "function cOpts(){"
          "return [...Array(16).keys()].map(i=>`<option value=\"${i}\">CH${i}</option>`).join('');"
          "}"
          "function add(){"
          "const tb=document.getElementById('rows');"
          "const n=tb.rows.length;"
          "const tr=tb.insertRow();"
          "tr.innerHTML=`<td>${n}</td>`"
          "+`<td><select name=\"crsf_${n}\">${cOpts()}</select></td>`"
          "+`<td><select name=\"pin_${n}\">${pOpts(0)}</select></td>`"
          "+`<td><button type='button' class='del' onclick='del(this)'>x</button></td>`;"
          "document.getElementById('numch').value=n+1;"
          "}"
          "function del(b){"
          "const tb=document.getElementById('rows');"
          "b.closest('tr').remove();"
          "[...tb.rows].forEach((r,i)=>{"
          "r.cells[0].textContent=i;"
          "r.cells[1].querySelector('select').name=`crsf_${i}`;"
          "r.cells[2].querySelector('select').name=`pin_${i}`;"
          "});"
          "document.getElementById('numch').value=tb.rows.length;"
          "}"
          "</script></body></html>";

  return html;
}

// ---- Web server ----
WebServer server(80);

void handleRoot() {
  server.send(200, "text/html", buildHtml());
}

void handleSave() {
  cfg.crsfRxPin   = (uint8_t) server.arg("rxpin").toInt();
  cfg.numChannels = (uint8_t) server.arg("numch").toInt();
  if (cfg.numChannels == 0 || cfg.numChannels > 16) cfg.numChannels = 1;
  for (uint8_t i = 0; i < cfg.numChannels; i++) {
    cfg.ch[i].crsfCh = (uint8_t)server.arg("crsf_" + String(i)).toInt();
    cfg.ch[i].pin    = (uint8_t)server.arg("pin_"  + String(i)).toInt();
  }
  cfg.crsfMin  = (int16_t)server.arg("crsfMin").toInt();
  cfg.crsfMax  = (int16_t)server.arg("crsfMax").toInt();
  cfg.pwmMinUs = (int16_t)server.arg("pwmMin").toInt();
  cfg.pwmMaxUs = (int16_t)server.arg("pwmMax").toInt();
  saveConfig();
  server.sendHeader("Location", "/");
  server.send(303);
  delay(200);
  ESP.restart();
}

void startAP() {
  WiFi.softAP("CRSF-PWM");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void startWebServer() {
  server.on("/",     HTTP_GET,  handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("Web server started");
}

// ---- CRSF + PWM ----
#define PIN_TX   -1
#define PIN_BOOT  0   // BOOT button — LOW when held at power-up

bool configMode = false;

const uint32_t PWM_FREQ = 50;   // Hz
const uint8_t  PWM_RES  = 16;   // bits

HardwareSerial crsfSerial(1);
AlfredoCRSF    crsf;

unsigned long lastUpdate = 0;

uint32_t usToDuty(int us) {
  return (uint32_t)((long)us * ((1 << PWM_RES) - 1) / (1000000 / PWM_FREQ));
}

uint16_t getCrsfChannel(const crsf_channels_t* ch, uint8_t idx) {
  switch (idx) {
    case  0: return ch->ch0;
    case  1: return ch->ch1;
    case  2: return ch->ch2;
    case  3: return ch->ch3;
    case  4: return ch->ch4;
    case  5: return ch->ch5;
    case  6: return ch->ch6;
    case  7: return ch->ch7;
    case  8: return ch->ch8;
    case  9: return ch->ch9;
    case 10: return ch->ch10;
    case 11: return ch->ch11;
    case 12: return ch->ch12;
    case 13: return ch->ch13;
    case 14: return ch->ch14;
    case 15: return ch->ch15;
    default: return 992;
  }
}

void initPWM() {
  for (uint8_t i = 0; i < cfg.numChannels; i++) {
    ledcAttach(cfg.ch[i].pin, PWM_FREQ, PWM_RES);
    ledcWrite(cfg.ch[i].pin, usToDuty(1500));
  }
}

// ---- Arduino entry points ----

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  loadConfig();

  // Give 3 s to press BOOT for config mode (do NOT hold at power-on — that enters bootloader)
  Serial.println("Press BOOT within 5s for config mode...");
  unsigned long deadline = millis() + 5000;
  while (millis() < deadline) {
    if (digitalRead(PIN_BOOT) == LOW) { configMode = true; break; }
    delay(10);
  }

  if (configMode) {
    startAP();
    startWebServer();
    Serial.println("Config mode — connect to CRSF-PWM Wi-Fi, open 192.168.4.1");
  } else {
    Serial.println("Normal mode");
  }
  initPWM();
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, cfg.crsfRxPin, PIN_TX);
  delay(100);
  crsf.begin(crsfSerial);
  Serial.printf("CRSF RX: GPIO%d  |  %d PWM channels\n", cfg.crsfRxPin, cfg.numChannels);
}

void loop() {
  if (configMode) server.handleClient();
  crsf.update();

  if (millis() - lastUpdate >= 20) {
    lastUpdate = millis();
    const crsf_channels_t* ch = crsf.getChannelsPacked();
    if (ch != nullptr) {
      for (uint8_t i = 0; i < cfg.numChannels; i++) {
        uint16_t raw = getCrsfChannel(ch, cfg.ch[i].crsfCh);
        int us = map(raw, cfg.crsfMin, cfg.crsfMax, cfg.pwmMinUs, cfg.pwmMaxUs);
        us = constrain(us, cfg.pwmMinUs, cfg.pwmMaxUs);
        ledcWrite(cfg.ch[i].pin, usToDuty(us));
      }
    }
  }
}
