#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#else
#include <WiFi.h>
#include <WebServer.h>
#endif


// EEPROM配置
#define EEPROM_SIZE 4096

#define WIFI_CONFIG_FLAG_ADDR 3000
#define WIFI_CONFIG_SSID_ADDR 3001
#define WIFI_CONFIG_PASS_ADDR 3033
#define WIFI_CONFIG_AUTH_ADDR 3097

#define WIFI_CONFIG_MAX_SSID 32
#define WIFI_CONFIG_MAX_PASS 64
#define WIFI_CONFIG_MAX_AUTH 32


// 外部变量
extern char auth[];
extern char ssid[];
extern char pswd[];


// 函数声明

bool loadWiFiConfig();

void saveWiFiConfig(
    const char *newSsid,
    const char *newPass,
    const char *newAuth
);


bool connectWiFi();


void startConfigPortal();


#endif