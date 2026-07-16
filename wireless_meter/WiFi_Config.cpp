#include "WiFi_Config.h"

#if defined(ESP8266)

ESP8266WebServer server(80);

#else

WebServer server(80);

#endif

bool configPortalActive = false;

//==============================
// EEPROM读取字符串
//==============================

String readStringFromEEPROM(
  int addr,
  int maxLen) {
  // 限制最大读取长度，确保缓冲区有空间存放终止符
  if (maxLen > 63)
    maxLen = 63;

  char buf[65];

  memset(buf, 0, sizeof(buf));

  for (int i = 0; i < maxLen; i++) {
    buf[i] = EEPROM.read(addr + i);

    if (buf[i] == 0)
      break;
  }

  return String(buf);
}

//==============================
// EEPROM写字符串
//==============================

void writeStringToEEPROM(
  int addr,
  int maxLen,
  const char *value) {

  int len = strlen(value);

  for (int i = 0; i < maxLen; i++) {
    if (i < len)
      EEPROM.write(addr + i, value[i]);
    else
      EEPROM.write(addr + i, 0);
  }
}

//==============================
// 读取WiFi配置
//==============================

bool loadWiFiConfig() {

  EEPROM.begin(EEPROM_SIZE);

  byte flag =
    EEPROM.read(WIFI_CONFIG_FLAG_ADDR);

  if (flag != 0xA5) {
    EEPROM.end();
    return false;
  }

  String storedSsid =
    readStringFromEEPROM(
      WIFI_CONFIG_SSID_ADDR,
      WIFI_CONFIG_MAX_SSID);

  String storedPass =
    readStringFromEEPROM(
      WIFI_CONFIG_PASS_ADDR,
      WIFI_CONFIG_MAX_PASS);

  String storedAuth =
    readStringFromEEPROM(
      WIFI_CONFIG_AUTH_ADDR,
      WIFI_CONFIG_MAX_AUTH);

  if (storedSsid.length() == 0 || storedPass.length() == 0) {
    EEPROM.end();
    return false;
  }

  storedSsid.toCharArray(
    ssid,
    WIFI_CONFIG_MAX_SSID);

  storedPass.toCharArray(
    pswd,
    WIFI_CONFIG_MAX_PASS);

  if (storedAuth.length()) {
    storedAuth.toCharArray(
      auth,
      WIFI_CONFIG_MAX_AUTH);
  }

  EEPROM.end();

  return true;
}

//==============================
// 保存WiFi配置
//==============================

void saveWiFiConfig(
  const char *newSsid,
  const char *newPass,
  const char *newAuth) {

  EEPROM.begin(EEPROM_SIZE);

  writeStringToEEPROM(
    WIFI_CONFIG_SSID_ADDR,
    WIFI_CONFIG_MAX_SSID,
    newSsid);

  writeStringToEEPROM(
    WIFI_CONFIG_PASS_ADDR,
    WIFI_CONFIG_MAX_PASS,
    newPass);

  writeStringToEEPROM(
    WIFI_CONFIG_AUTH_ADDR,
    WIFI_CONFIG_MAX_AUTH,
    newAuth);

  EEPROM.write(
    WIFI_CONFIG_FLAG_ADDR,
    0xA5);

  EEPROM.commit();

  EEPROM.end();
}

//==============================
// 连接WiFi
//==============================

bool connectWiFi() {

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    ssid,
    pswd);

  Serial.print(
    "Connecting WiFi ");

  Serial.println(ssid);

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED && millis() - start < 20000) {

    delay(500);

    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    Serial.print(
      "IP:");

    Serial.println(
      WiFi.localIP());

    return true;
  }

  return false;
}

//==============================
// 网页首页
//==============================

void handleRoot() {
  String page = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no\"><title>EV Meter 配网</title><style>body{margin:0;font-family:Arial,sans-serif;background:#f4f7fb;color:#223;display:flex;justify-content:center;align-items:flex-start;min-height:100vh;padding:20px;box-sizing:border-box;} .card{width:min(100%,420px);background:#fff;padding:20px;border-radius:16px;box-shadow:0 8px 24px rgba(0,0,0,.12);} h2{margin-top:0;font-size:22px;text-align:center;} label{display:block;margin-bottom:6px;font-weight:600;} input[type=text]{width:100%;padding:10px 12px;border:1px solid #cfd8e3;border-radius:8px;margin-bottom:12px;box-sizing:border-box;font-size:16px;} input[type=submit]{width:100%;padding:12px;border:none;border-radius:8px;background:#1e88e5;color:#fff;font-size:16px;} p{font-size:14px;color:#667;line-height:1.5;}</style></head><body><div class=\"card\">";
  page += "<h2>EV Meter WiFi 配置</h2>";
  page += "<form action=\"/save\" method=\"get\">";
  page += "<label>SSID</label><input type=\"text\" name=\"ssid\" value=\"" + String(ssid) + "\">";
  page += "<label>Password</label><input type=\"text\" name=\"pswd\" value=\"" + String(pswd) + "\">";
  page += "<label>Blinker Auth</label><input type=\"text\" name=\"auth\" value=\"" + String(auth) + "\">";
  page += "<input type=\"submit\" value=\"保存并重启\">";
  page += "</form>";
  page += "<p>保存后设备将重启并尝试连接 WiFi。</p>";
  page += "</div></body></html>";
  server.send(200, "text/html; charset=UTF-8", page);
}

//==============================
// 保存网页提交
//==============================

void handleSave() {
  if (!server.hasArg("ssid") || !server.hasArg("pswd") || !server.hasArg("auth")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pswd");
  String newAuth = server.arg("auth");
  if (newSsid.length() == 0 || newPass.length() == 0) {
    server.send(400, "text/plain", "SSID and password cannot be empty");
    return;
  }
  if (newAuth.length() == 0) {
    newAuth = String(auth);
  }
  saveWiFiConfig(newSsid.c_str(), newPass.c_str(), newAuth.c_str());
  String page = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>EV Meter 配网</title></head><body>";
  page += "<h2>配置已保存</h2>";
  page += "<p>设备即将重启...</p>";
  page += "</body></html>";
  server.send(200, "text/html; charset=UTF-8", page);
  delay(2000);
  ESP.restart();
}


//==============================
// 启动AP配网
//==============================

void startConfigPortal() {

  WiFi.mode(WIFI_AP);

  WiFi.softAP(
    "EVMeter_Config");

  server.on(
    "/",
    handleRoot);

  server.on(
    "/save",
    handleSave);

  server.begin();

  Serial.println(
    "AP Config Started");

  while (true) {

    server.handleClient();

    delay(2);
  }
}