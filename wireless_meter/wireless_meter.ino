/*
 * wireless_meter.ino
 * 48V 新能源电动车监测板
 *
 * 使用 Blinker 进行 APP 与云端数据交互。
 * 需要 ESP8266/ESP32 Arduino 支持包以及 Blinker 库。
 *
 * Blinker 库:
 * https://github.com/blinker-iot/blinker-library/archive/master.zip
 *
 * ESP8266 Arduino >= 2.5.0:
 * https://github.com/esp8266/Arduino/releases
 *
 * ESP32 Arduino >= 1.0.2:
 * https://github.com/espressif/arduino-esp32/releases
 *
 * 文档:
 * https://doc.blinker.app/
 * https://github.com/blinker-iot/blinker-doc/wiki
 */

#define BLINKER_WIFI
#define BLINKER_ALIGENIE_SENSOR
#define BLINKER_ALIGENIE_PRINT Serial
#define BLINKER_MIOT_LIGHT // 定义为语音控制灯设备
// #define BLINKER_WITH_SSL

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#else
#include <WiFi.h>
#include <WebServer.h>
#endif
#include <EEPROM.h>
#include <Blinker.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "Gif.h"
#include "DisplayGif.h"
#include "i2c_scanner.h"

#define EEPROM_SIZE 4096
// Place WiFi config well above Blinker-used ranges (Blinker uses 0-2431)
#define WIFI_CONFIG_FLAG_ADDR 3000
#define WIFI_CONFIG_SSID_ADDR 3001
#define WIFI_CONFIG_PASS_ADDR 3033
#define WIFI_CONFIG_AUTH_ADDR 3097
#define WIFI_CONFIG_MAX_SSID 32
#define WIFI_CONFIG_MAX_PASS 64
#define WIFI_CONFIG_MAX_AUTH 32

char auth[WIFI_CONFIG_MAX_AUTH] = "38759f5fd7f3";
char ssid[WIFI_CONFIG_MAX_SSID] = "Xiaomi";
char pswd[WIFI_CONFIG_MAX_PASS] = "1234567891";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp1.aliyun.com", 60 * 60 * 8, 30 * 60 * 1000);
const int hour = 0;
const int minute = 0;
#define Number_1 "BusVoltage"
#define Number_2 "ShuntCurrent"
#define Number_3 "BusPower"
#define Number_4 "CapaPower"
// #define Number_5 "BlinkerTime"
#define Number_9 "da"
#define Number_8 "BusPower1"
unsigned int Rs = 220000; // R1取值为220k  NTC测温
double Vcc = 3.3;
int val = 0;
double V_NTC;
double R_NTC;
int sampleStoreTS = 0;

static unsigned long time1 = millis();
static unsigned long time3 = millis();
static unsigned long time7 = millis();
static unsigned long time8 = millis();
static unsigned long timebuzzer = millis();
static unsigned long timewarning = millis();

static unsigned long timebuzzer1 = 0, timewarning1 = 0, timewarning2 = 0;
static unsigned long time2 = 0, time4 = 0, timedisplay = 0;
static unsigned long time5 = 0, time6 = 0, time9 = 0;

BlinkerNumber Number1(Number_1);
BlinkerNumber Number2(Number_2);
BlinkerNumber Number3(Number_3);
BlinkerNumber Number4(Number_4);
// BlinkerNumber Number5(Number_5);
BlinkerNumber Number9(Number_9);
BlinkerNumber Number8(Number_8);
BlinkerNumber DATA1("datagl");
BlinkerNumber DATA2("datadl");
BlinkerNumber TA("ta");
BlinkerNumber FULL("full");
BlinkerText TEXT1("runtime");
BlinkerText TEXT2("tex-s1");
BlinkerText TEXT3("tex-s2");
// 新建组件对象
// BlinkerButton Button1("btn-1fg");//组件对象,要和APP组件中的“数据键名”一致
BlinkerButton Button2("btn-2fg");
BlinkerButton Button9("btn-9fg");
BlinkerButton Button7("btn-3fg");
BlinkerButton Button8("btn-4fg");
BlinkerButton Button6("btn-res");
/// 读取DHTXX传感器相关定义和变量//
BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");
BlinkerNumber DATA3("datawendu");
BlinkerNumber DATA4("datatemp1");
BlinkerNumber DATA5("datahumi1");
BlinkerNumber TEM("tem-1"); // 设备温度键名
BlinkerNumber KM1("Km1");   // 设备温度键名
#define DHTPIN D7           // 定义单总线协议传输的数据引脚
// 传感器类型选择
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// #define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
float humi_read = 0, temp_read = 0;

/*****************/
#include <Arduino.h>
#include <Wire.h>
#include <INA226.h>

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
const int IA = D3;
// int buttonState=0;//存储按键值状态

float BusVoltage;
float ShuntCurrent;
float BusPower;
float BusPower2;
float CapaPower = 0;
bool ina226Ready = false;
float Da;
float Sa;
float BusPower1;
float ShuntCurrent1;
float ta;
float tem11;
int Mode2 = 0;
int Mode1 = 0;
int Displaymode = 0;
int charge1 = 0;
int charge = 0;
int Buttonmode = 0;
// int rt;
// int r,e,f,s;
float h;
float t;
int BUS_Int;
float Vscale = (54.6 / 18.02); // 分压电路分压比例
float KM = 0;                  // 剩余里程
int mileage = 20;              // 设置总里程
// unsigned int RALL = 30000;//分压电路总电阻值
//============================ 运行时间常量定义 ==================================//
String fh;
int rt;
int r, e, f, s;
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 14, /* data=*/ 12, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/2, /* data=*/0, /* reset=*/U8X8_PIN_NONE); // ESP32 Thing, pure SW emulated I2C
INA226 ina;

const unsigned char Image0[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0x00, 0x02, 0x02, 0x00, 0x02, 0x03, 0x00, 0x02, 0x03, 0x00, 0x02, 0x03, 0x00, 0x02,
    0x03, 0x00, 0x02, 0x02, 0x00, 0x02, 0x02, 0x00, 0x02, 0xFE, 0xFF, 0x03}; // 电量为零
const unsigned char Image1[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0x00, 0x03, 0x02, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00, 0x03,
    0x03, 0x00, 0x03, 0x02, 0x00, 0x03, 0x02, 0x00, 0x03, 0xFE, 0xFF, 0x03}; // 电量为1
const unsigned char Image2[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0x80, 0x03, 0x02, 0x80, 0x03, 0x03, 0x80, 0x03, 0x03, 0x80, 0x03, 0x03, 0x80, 0x03,
    0x03, 0x80, 0x03, 0x02, 0x80, 0x03, 0x02, 0x80, 0x03, 0xFE, 0xFF, 0x03}; // 电量为2
const unsigned char Image3[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xC0, 0x03, 0x02, 0xC0, 0x03, 0x03, 0xC0, 0x03, 0x03, 0xC0, 0x03, 0x03, 0xC0, 0x03,
    0x03, 0xC0, 0x03, 0x02, 0xC0, 0x03, 0x02, 0xC0, 0x03, 0xFE, 0xFF, 0x03}; // 电量为3
const unsigned char Image4[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xE0, 0x03, 0x02, 0xE0, 0x03, 0x03, 0xE0, 0x03, 0x03, 0xE0, 0x03, 0x03, 0xE0, 0x03,
    0x03, 0xE0, 0x03, 0x02, 0xE0, 0x03, 0x02, 0xE0, 0x03, 0xFE, 0xFF, 0x03}; // 电量为4
const unsigned char Image5[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xF0, 0x03, 0x02, 0xF0, 0x03, 0x03, 0xF0, 0x03, 0x03, 0xF0, 0x03, 0x03, 0xF0, 0x03,
    0x03, 0xF0, 0x03, 0x02, 0xF0, 0x03, 0x02, 0xF0, 0x03, 0xFE, 0xFF, 0x03}; // 电量为5
const unsigned char Image6[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xF8, 0x03, 0x02, 0xF8, 0x03, 0x03, 0xF8, 0x03, 0x03, 0xF8, 0x03, 0x03, 0xF8, 0x03,
    0x03, 0xF8, 0x03, 0x02, 0xF8, 0x03, 0x02, 0xF8, 0x03, 0xFE, 0xFF, 0x03}; // 电量为6
const unsigned char Image7[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xFC, 0x03, 0x02, 0xFC, 0x03, 0x03, 0xFC, 0x03, 0x03, 0xFC, 0x03, 0x03, 0xFC, 0x03,
    0x03, 0xFC, 0x03, 0x02, 0xFC, 0x03, 0x02, 0xFC, 0x03, 0xFE, 0xFF, 0x03}; // 电量为7
const unsigned char Image8[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xFE, 0x03, 0x02, 0xFE, 0x03, 0x03, 0xFE, 0x03, 0x03, 0xFE, 0x03, 0x03, 0xFE, 0x03,
    0x03, 0xFE, 0x03, 0x02, 0xFE, 0x03, 0x02, 0xFE, 0x03, 0xFE, 0xFF, 0x03}; // 电量为8
const unsigned char Image9[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x02, 0xFF, 0x03, 0x02, 0xFF, 0x03, 0x03, 0xFF, 0x03, 0x03, 0xFF, 0x03, 0x03, 0xFF, 0x03,
    0x03, 0xFF, 0x03, 0x02, 0xFF, 0x03, 0x02, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为9
const unsigned char Image10[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0x82, 0xFF, 0x03, 0x82, 0xFF, 0x03, 0x83, 0xFF, 0x03, 0x83, 0xFF, 0x03, 0x83, 0xFF, 0x03,
    0x83, 0xFF, 0x03, 0x82, 0xFF, 0x03, 0x82, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为10
const unsigned char Image11[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0xC2, 0xFF, 0x03, 0xC2, 0xFF, 0x03, 0xC3, 0xFF, 0x03, 0xC3, 0xFF, 0x03, 0xC3, 0xFF, 0x03,
    0xC3, 0xFF, 0x03, 0xC2, 0xFF, 0x03, 0xC2, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为11
const unsigned char Image12[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0xE2, 0xFF, 0x03, 0xE2, 0xFF, 0x03, 0xE3, 0xFF, 0x03, 0xE3, 0xFF, 0x03, 0xE3, 0xFF, 0x03,
    0xE3, 0xFF, 0x03, 0xE2, 0xFF, 0x03, 0xE2, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为12
const unsigned char Image13[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0xF2, 0xFF, 0x03, 0xF2, 0xFF, 0x03, 0xF3, 0xFF, 0x03, 0xF3, 0xFF, 0x03, 0xF3, 0xFF, 0x03,
    0xF3, 0xFF, 0x03, 0xF2, 0xFF, 0x03, 0xF2, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为13
const unsigned char Image14[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0xFA, 0xFF, 0x03, 0xFA, 0xFF, 0x03, 0xFB, 0xFF, 0x03, 0xFB, 0xFF, 0x03, 0xFB, 0xFF, 0x03,
    0xFB, 0xFF, 0x03, 0xFA, 0xFF, 0x03, 0xFA, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为14
const unsigned char Image15[] U8X8_PROGMEM = {
    0xFE, 0xFF, 0x03, 0xFE, 0xFF, 0x03, 0xFE, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03,
    0xFF, 0xFF, 0x03, 0xFE, 0xFF, 0x03, 0xFE, 0xFF, 0x03, 0xFE, 0xFF, 0x03}; // 电量为15

/*const unsigned char Image0[]U8X8_PROGMEM = {
0x00,0x08,0x00,0x00,0x0C,0x00,0xFE,0xFF,0x03,0x02,0x09,0x02,0x83,0x08,0x02,0x43,0x78,0x02,
0xE3,0x21,0x02,0x03,0x11,0x02,0x02,0x09,0x02,0xFE,0xFF,0x03,0x00,0x03,0x00,0x00,0x01,0x00

};//充电图标（空心）*/

const unsigned char Imagecharge[] U8X8_PROGMEM = {
    0x00, 0x08, 0x00, 0x00, 0x0C, 0x00, 0xFE, 0xFF, 0x03, 0x02, 0x0F, 0x02, 0x83, 0x0F, 0x02, 0xC3, 0x7F, 0x02,
    0xE3, 0x3F, 0x02, 0x03, 0x1F, 0x02, 0x02, 0x0F, 0x02, 0xFE, 0xFF, 0x03, 0x00, 0x03, 0x00, 0x00, 0x01, 0x00}; // 充电图标（实心）
/**
 * @brief  屏幕开关控制回调函数
 *
 *        用于响应 Blinker APP 中 Button9 按键的状态变化，
 *        根据按键状态控制 OLED/LCD 屏幕显示或关闭。
 *
 *        当按钮状态为 "on" 时：
 *          - 设置显示模式为正常显示模式
 *          - 提示屏幕已打开
 *          - 更新 APP 中按钮状态
 *
 *        当按钮状态为 "off" 时：
 *          - 设置显示模式为关闭模式
 *          - 清空屏幕缓存并刷新屏幕
 *          - 提示屏幕已关闭
 *          - 更新 APP 中按钮状态
 *
 * @param state9  Blinker Button9 控件返回的状态字符串
 *                "on"  ：打开屏幕
 *                "off" ：关闭屏幕
 *
 * @return 无
 */
void button9_callback(const String &state9) // 屏幕开关
{
  BLINKER_LOG("get button state: ", state9); // APP中的Monitor控件打印的信息
  if (state9 == "on")
  {
    Displaymode = 0;
    TEXT3.print("屏幕打开");
    Button9.print("on"); // 反馈开关状态
  }
  else if (state9 == "off")
  {

    Displaymode = 1;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    TEXT3.print("屏幕关闭");
    Button9.print("off"); // 反馈开关状态
  }
}

/*void button1_callback(const String & state)// 按下按键即会执行该函数
{
   BLINKER_LOG("get button state: ", state);//APP中的Monitor控件打印的信息
   if (state=="on")
   {
       digitalWrite(D10, HIGH);//打开风扇
       // 反馈开关状态
       Button1.print("on");
   } else {
       digitalWrite(D10, LOW);//关闭风扇
       // 反馈开关状态

       Button1.print("off");
   }
}*/
void button2_callback(const String &state2) // 手动开关
{
  BLINKER_LOG("get button state: ", state2); // APP中的Monitor控件打印的信息
  if (state2 == "on")
  {
    Mode1 = 1;
    // digitalWrite(D6, HIGH);///打开闭输出
    //  反馈开关状态
    TEXT2.color("#7CFC00");
    TEXT2.icon("fad fa-siren-on");
    TEXT2.print("手动打开");
    Button8.text("手动模式");
    Button8.print("off");
    Button2.print("on");
    analogWrite(D6, 63.75);
    delay(100);
    analogWrite(D6, 125);
    delay(100);
    analogWrite(D6, 190);
    delay(100);
    Buttonmode = 1;
  }
  else if (state2 == "off")
  {
    Mode1 = 1;
    // digitalWrite(D6, LOW);///关闭输出
    //  反馈开关状态
    Buttonmode = 0;
    TEXT2.color("#FF0000");
    TEXT2.icon("fad fa-siren-on");
    TEXT2.print("手动关闭");
    Button8.text("手动模式");
    Button8.print("off");
    Button2.print("off");
  }
}

/*void button2_callback(const String & state2)//  重启开关
{
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    BLINKER_LOG("get button state: ", state2);

    if (state2 == "tap") {
        BLINKER_LOG("Button tap!");

       Mode1=1;
       if(digitalRead(D6))
       {
        digitalWrite(D6, LOW);///打开闭输出
        TEXT2.color("#7CFC00");
        TEXT2.icon("fad fa-siren-on");
        TEXT2.print("手动关闭");
       }else{
        digitalWrite(D6, HIGH);///打开闭输出
        TEXT2.color("#7CFC00");
        TEXT2.icon("fad fa-siren-on");
        TEXT2.print("手动打开");
        }
        // 反馈开关状态

        Button8.text("手动模式");
        Button8.print("off");
    }

}*/

void button7_callback(const String &state7) //  电量、峰值功率数据清空
{
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  BLINKER_LOG("get button state: ", state7);

  if (state7 == "tap")
  {
    BLINKER_LOG("Button tap!");

    BusPower1 = 0;
    Da = 0;
    CapaPower = 0;
    ShuntCurrent1 = 0;
    Button7.text("清除成功");
    TEXT3.color("#7CFC00");
    TEXT3.icon("fad fa-siren-on");
    TEXT3.print("清除数据成功");
  }
}
void button8_callback(const String &state8)
{
  BLINKER_LOG("get button state: ", state8); // APP中的Monitor控件打印的信息
  if (state8 == "on")
  {
    Button8.text("自动模式");
    TEXT2.print("自动模式");
    Button8.print("on");
    Button2.print("off");
    if (BusVoltage > 42.9 && BusVoltage <= 55.9)
    {
      analogWrite(D6, 63.75);
      delay(100);
      analogWrite(D6, 125);
      delay(100);
      analogWrite(D6, 190);
      delay(100);
    }
    Mode1 = 0;
    Mode2 = 0;
    charge1 = 0;
    charge = 0;
  }
  else if (state8 == "off")
  {
    Button8.print("off");
  }
}

void button6_callback(const String &state6) //  重启开关
{
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  BLINKER_LOG("get button state: ", state6);

  if (state6 == "tap")
  {
    BLINKER_LOG("Button tap!");
    // Button6.text("重启发送");
    // delay(500);
    TEXT2.print("重启开始");
    delay(500);
    ESP.restart();
  }
}

// 小爱电源类回调
void miotPowerState(const String &state2)
{
  BLINKER_LOG("need set power state: ", state2);

  if (state2 == BLINKER_CMD_ON)
  {
    Mode1 = 1;
    // digitalWrite(D6, HIGH);///打开输出
    Buttonmode = 1;
    BlinkerMIOT.powerState("on");
    BlinkerMIOT.print();
  }
  else if (state2 == BLINKER_CMD_OFF)
  {
    Mode1 = 1;
    // digitalWrite(D6, LOW);///关闭输出
    Buttonmode = 0;
    BlinkerMIOT.powerState("off");
    BlinkerMIOT.print();
  }
}

/**
 * @brief  检查并打印 INA226 当前配置参数
 *
 *        读取 INA226 电流/电压检测芯片的当前工作配置，
 *        并通过串口输出以下信息：
 *
 *        1. ADC 工作模式
 *           - 关闭模式
 *           - 单次采样模式
 *           - 连续采样模式
 *
 *        2. ADC 平均采样次数
 *           - 1次 ~ 1024次
 *
 *        3. 总线电压转换时间
 *
 *        4. 分流电压转换时间
 *
 *        5. 当前量程相关参数
 *           - 最大可测电流
 *           - 最大电流
 *           - 最大分流电压
 *           - 最大功率
 *
 *        该函数主要用于：
 *          - INA226初始化调试
 *          - 检查配置是否正确
 *          - 排查电流、电压测量异常问题
 *
 * @param 无
 *
 * @return 无
 */
void checkConfig()
{
  Serial.print("Mode:                  ");
  switch (ina.getMode())
  {
  case INA226_MODE_POWER_DOWN:
    Serial.println("Power-Down");
    break;
  case INA226_MODE_SHUNT_TRIG:
    Serial.println("Shunt Voltage, Triggered");
    break;
  case INA226_MODE_BUS_TRIG:
    Serial.println("Bus Voltage, Triggered");
    break;
  case INA226_MODE_SHUNT_BUS_TRIG:
    Serial.println("Shunt and Bus, Triggered");
    break;
  case INA226_MODE_ADC_OFF:
    Serial.println("ADC Off");
    break;
  case INA226_MODE_SHUNT_CONT:
    Serial.println("Shunt Voltage, Continuous");
    break;
  case INA226_MODE_BUS_CONT:
    Serial.println("Bus Voltage, Continuous");
    break;
  case INA226_MODE_SHUNT_BUS_CONT:
    Serial.println("Shunt and Bus, Continuous");
    break;
  default:
    Serial.println("unknown");
  }

  /**********************************************
   * 输出 ADC 平均采样次数
   *
   * 平均次数越高：
   *   - 数据越稳定
   *   - 噪声越小
   *   - 转换时间越长
   **********************************************/
  Serial.print("Samples average:       ");
  switch (ina.getAverages())
  {
  case INA226_AVERAGES_1:
    Serial.println("1 sample");
    break;
  case INA226_AVERAGES_4:
    Serial.println("4 samples");
    break;
  case INA226_AVERAGES_16:
    Serial.println("16 samples");
    break;
  case INA226_AVERAGES_64:
    Serial.println("64 samples");
    break;
  case INA226_AVERAGES_128:
    Serial.println("128 samples");
    break;
  case INA226_AVERAGES_256:
    Serial.println("256 samples");
    break;
  case INA226_AVERAGES_512:
    Serial.println("512 samples");
    break;
  case INA226_AVERAGES_1024:
    Serial.println("1024 samples");
    break;
  default:
    Serial.println("unknown");
  }

  Serial.print("Bus conversion time:   ");
  switch (ina.getBusConversionTime())
  {
  case INA226_BUS_CONV_TIME_140US:
    Serial.println("140uS");
    break;
  case INA226_BUS_CONV_TIME_204US:
    Serial.println("204uS");
    break;
  case INA226_BUS_CONV_TIME_332US:
    Serial.println("332uS");
    break;
  case INA226_BUS_CONV_TIME_588US:
    Serial.println("558uS");
    break;
  case INA226_BUS_CONV_TIME_1100US:
    Serial.println("1.100ms");
    break;
  case INA226_BUS_CONV_TIME_2116US:
    Serial.println("2.116ms");
    break;
  case INA226_BUS_CONV_TIME_4156US:
    Serial.println("4.156ms");
    break;
  case INA226_BUS_CONV_TIME_8244US:
    Serial.println("8.244ms");
    break;
  default:
    Serial.println("unknown");
  }

  Serial.print("Shunt conversion time: ");
  switch (ina.getShuntConversionTime())
  {
  case INA226_SHUNT_CONV_TIME_140US:
    Serial.println("140uS");
    break;
  case INA226_SHUNT_CONV_TIME_204US:
    Serial.println("204uS");
    break;
  case INA226_SHUNT_CONV_TIME_332US:
    Serial.println("332uS");
    break;
  case INA226_SHUNT_CONV_TIME_588US:
    Serial.println("558uS");
    break;
  case INA226_SHUNT_CONV_TIME_1100US:
    Serial.println("1.100ms");
    break;
  case INA226_SHUNT_CONV_TIME_2116US:
    Serial.println("2.116ms");
    break;
  case INA226_SHUNT_CONV_TIME_4156US:
    Serial.println("4.156ms");
    break;
  case INA226_SHUNT_CONV_TIME_8244US:
    Serial.println("8.244ms");
    break;
  default:
    Serial.println("unknown");
  }

  Serial.print("Max possible current:  ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:           ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:     ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:             ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}

/*****************/

void heartbeat()
{
  TEXT1.print(fh);
  TEXT1.icon("fas fa-alarm-clock"); // 运行时间上传

  if (Mode1 == 0) // 自动模式开关反馈
  {
    Button8.text("自动模式");
    Button8.print("on");
    Button2.print("off");
  }
  else if (Mode1 == 1)
  {
    Button8.text("手动模式");
    Button8.print("off");
  }

  if (Displaymode == 1) // 屏幕开关状态显示
  {
    Button9.print("off");
  }
  else
  {
    Button9.print("on");
  }

  if (Mode1 == 0)
  {
    if (BusVoltage <= 55.9)
    {
      if (BusVoltage >= 42.9)
      {
        if (BusVoltage >= 42.9 && digitalRead(D6))
        {
          TEXT2.color("#32CD32");
          TEXT2.print("输出正常");
        }
        else if (BusVoltage >= 42.9 && !digitalRead(D6))
        {
          TEXT2.color("#32CD32");
          TEXT2.print("输出异常关断");
        }
      }
      else if (!digitalRead(D6))
      {
        TEXT2.color("#FF0000");
        TEXT2.icon("fad fa-siren-on");
        TEXT2.print("低压保护");
      }
      else
      {
        TEXT2.color("#FF0000");
        TEXT2.icon("fad fa-siren-on");
        TEXT2.print("低压保护失效");
      }
    }
  }
  else
  {
    if (Mode1 == 1 && !digitalRead(D6))
    {
      TEXT2.color("#FF0000");
      TEXT2.icon("fad fa-siren-on");
      TEXT2.print("手动关闭");
      Button2.text("手动关闭");
      Button2.print("off");
    }
    else if (Mode1 == 1 && digitalRead(D6))
    {
      TEXT2.color("#FF0000");
      TEXT2.icon("fad fa-siren-on");
      TEXT2.print("手动打开");
      Button2.text("手动打开");
      Button2.print("on");
    }
  }
}

void dataStorage()
{
  Blinker.dataStorage("datagl", BusPower);
  Blinker.dataStorage("datadl", ShuntCurrent);
  Blinker.dataStorage("datawendu", tem11);
  Blinker.dataStorage("datatemp1", temp_read);
  Blinker.dataStorage("datahumi1", humi_read);
}

/**********按键回调内容上传函数********/
void dataRead(const String &data)
{
  BLINKER_LOG("Blinker readString: ", data);
  Blinker.vibrate();
  if (BusVoltage < 45.5)
  {
    if (BusVoltage >= 44.2)
    {
      Number1.color("#FFA500");
      Number1.icon("fad fa-car-battery");
      Number1.print(BusVoltage);
    }
    if (BusVoltage < 44.2)
    {
      Number1.color("#FF0000");
      Number1.icon("fad fa-car-battery");
      Number1.print(BusVoltage);
    }
  }
  else
  {
    Number1.icon("fad fa-car-battery");
    Number1.print(BusVoltage);
  }

  if (ShuntCurrent >= 0)
  {
    Number2.color("#00FF7F");
    Number2.icon("fas fa-battery-bolt");
    Number2.print(ShuntCurrent * 1000);
    Number3.print(BusPower);
  }
  else
  {
    Number2.color("#FF0000");
    Number2.icon("fal fa-battery-half");
    Number2.print(ShuntCurrent * 1000);
    Number3.color("#FF0000");
    Number3.print(BusPower);
  }

  if (ShuntCurrent1 >= 0)
  {
    FULL.color("#00FF7F");
    FULL.icon("fas fa-battery-bolt");
    FULL.print(ShuntCurrent1 * 1000);
  }
  else
  {
    FULL.color("#FF0000");
    FULL.icon("fal fa-battery-half");
    FULL.print(ShuntCurrent1 * 1000);
  }
  // Blinker.print("W", BusPower);

  // Blinker.print("mAh", CapaPower);

  if (CapaPower < 6000)
  {
    if (CapaPower < 6000 && CapaPower >= 4000)
    {
      Number4.color("#FFA500");
      Number4.print(CapaPower);
    }
    if (CapaPower < 6000)
    {
      Number4.color("#FF0000");
      Number4.print(CapaPower);
    }
  }
  else
  {
    Number4.print(CapaPower);
  }

  if (Da < 150)
  {
    if (Da >= 100 && Da < 150)
    {
      // Blinker.print("Wh", Da);
      Number9.color("#FFA500");
      Number9.print(Da);
    }
    if (Da < 100)
    {
      // Blinker.print("Wh", Da);
      Number9.color("#FF0000");
      Number9.print(Da);
    }
  }
  else
  {
    Number9.print(Da);
  }

  // Blinker.print("W",BusPower1);

  if (BusPower1 >= 0)
  {
    Number8.print(BusPower1);
  }
  else if (BusPower1 < 0)
  {
    Number8.color("#FF0000");
    Number8.print(BusPower1);
  }

  if (ta <= 30)
  {
    if (ta <= 30 && ta >= 20)
    {
      TA.color("#FFA500");
      TA.icon("fas fa-battery-quarter");
      TA.print(ta);
    }
    if (ta < 20 && ta > 0)
    {
      TA.color("#FF0000");
      TA.icon("fas fa-battery-quarter");
      TA.print(ta);
    }
    if (ta <= 0)
    {
      TA.color("#FF0000");
      TA.icon("fas fa-battery-empty");
      TA.print(ta);
    }
  }
  else
  {
    if (ta >= 100)
    {
      TA.color("#32CD32");
      TA.icon("fas fa-battery-full");
      TA.print(ta);
    }
    if (ta < 100 && ta > 30)
    {
      TA.color("#00FF7F");
      TA.icon("fas fa-battery-three-quarters");
      TA.print(ta);
    }
  }

  KM1.print(KM);

  if (humi_read < 70)
  {
    if (humi_read <= 70 && humi_read >= 60)
    {
      HUMI.color("#9ACD32");
      HUMI.print(humi_read);
      HUMI.text("电池湿度大");
    }
    if (humi_read < 60 && humi_read >= 40)
    {
      HUMI.color("#00FF7F");
      HUMI.print(humi_read);
      HUMI.text("电池湿度较大");
    }
    if (humi_read < 40 && humi_read > 30)
    {
      HUMI.color("#FFD700");
      HUMI.print(humi_read);
      HUMI.text("电池湿度适中");
    }
    if (humi_read < 30)
    {
      HUMI.color("#FFA500");
      HUMI.print(humi_read);
      HUMI.text("电池干燥");
    }
  }
  else
  {
    // Blinker.print("%",humi_read);
    HUMI.color("#008080");
    HUMI.print(humi_read);
    HUMI.text("电池潮湿");
  }

  if (temp_read < 55)
  {
    if (temp_read < 55 && temp_read >= 40)
    {
      TEMP.icon("fas fa-temperature-up");
      TEMP.color("#FF6347");
      TEMP.print(temp_read);
      TEMP.text("电池发热");
      TEXT3.color("#FF0000");
      TEXT3.icon("fad fa-siren-on");
      TEXT3.print("电池发热");
    }
    if (temp_read < 40)
    {
      TEMP.icon("fad fa-temperature-up");
      TEMP.color("#FFA500");
      TEMP.print(temp_read);
      TEMP.text("电池温度正常");
    }
  }
  else if (temp_read >= 55 && !digitalRead(D6))
  {
    TEMP.icon("fad fa-temperature-hot");
    TEMP.color("#FF0000");
    TEMP.print(temp_read);
    TEMP.text("电池过热");
    TEXT3.color("#FF0000");
    TEXT3.icon("fad fa-siren-on");
    TEXT3.print("电池过热保护");
  }
  else if (temp_read >= 55 && digitalRead(D6))
  {
    TEMP.icon("fad fa-temperature-hot");
    TEMP.color("#FF0000");
    TEMP.print(temp_read);
    TEMP.text("电池过热");
    TEXT3.color("#FF0000");
    TEXT3.icon("fad fa-siren-on");
    TEXT3.print("电池过热保护失效");
  }

  if (tem11 >= 40)
  {
    if (tem11 >= 40 && tem11 <= 45)
    {
      TEM.icon("fad fa-temperature-down");
      TEM.color("#00FA9A");
      TEM.print(tem11); // 控制器温度
      TEM.text("场管中温");
    }
    if (tem11 > 45)
    {
      TEM.icon("fad fa-temperature-hot");
      TEM.color("#FF4500");
      TEM.print(tem11); // 控制器温度
      TEM.text("场管高温");
    }
  }
  else
  {
    TEM.icon("fad fa-oil-temp");
    TEM.color("#00FF00");
    TEM.print(tem11); // 控制器温度
    TEM.text("场管温度");
  }
}
/**********计算运行时间********/
void Runtime()
{
  rt = Blinker.runTime();
  Blinker.delay(100);
  if (rt >= 86400) // 天数
  {
    r = rt / 86400;
    e = rt / 3600 - r * 24;
    f = rt / 60 - r * 1440 - e * 60;
    s = rt - r * 86400 - e * 3600 - f * 60;
  }
  else if (rt >= 3600)
  {
    r = 0;
    e = rt / 3600;
    f = rt / 60 - e * 60;
    s = rt - e * 3600 - f * 60;
  }
  else
  {
    r = 0;
    e = 0;
    f = rt / 60;
    s = rt - f * 60;
  }
  if (f == 0 & e == 0 & r == 0)
  {
    fh = String("") + s + "秒";
  }
  else if (r == 0 & e == 0)
  {
    fh = String("") + f + "分" + s + "秒";
  }
  else if (r == 0)
  {
    fh = String("") + e + "时" + f + "分" + s + "秒";
  }
  else
  {
    fh = String("") + r + "天" + e + "时" + f + "分" + s + "秒";
  }
}
/**********显示电池图标函数********/
void batteryDespaly()
{
  if (ShuntCurrent > 0)
  {
    if (ta >= 100)
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(83, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }
    else if (ta < 10)
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(92, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }
    else
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(87, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }
    u8g2.drawXBMP(110, 2, 18, 12, Imagecharge); // 充电显示
  }
  else
  {

    if (ta >= 100)
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(83, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }
    else if (ta < 10)
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(92, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }
    else
    {
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.setCursor(100, 12);
      u8g2.print("%");
      u8g2.setCursor(87, 12);
      u8g2.print((int)ta); // 显示电量百分比
    }

    BUS_Int = (int)((BusVoltage - 42.9) / 0.78);
    if (BUS_Int < 1)
    {
      BUS_Int = 0;
    }
    if (BUS_Int > 15)
    {
      BUS_Int = 15;
    }
    if (BUS_Int == 0)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image0); // 电量显示
    }
    else if (BUS_Int == 1)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image1); // 电量显示
    }
    else if (BUS_Int == 2)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image2); // 电量显示
    }
    else if (BUS_Int == 3)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image3); // 电量显示
    }
    else if (BUS_Int == 4)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image4); // 电量显示
    }
    else if (BUS_Int == 5)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image5); // 电量显示
    }
    else if (BUS_Int == 6)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image6); // 电量显示
    }
    else if (BUS_Int == 7)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image7); // 电量显示
    }
    else if (BUS_Int == 8)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image8); // 电量显示
    }
    else if (BUS_Int == 9)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image9); // 电量显示
    }
    else if (BUS_Int == 10)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image10); // 电量显示
    }
    else if (BUS_Int == 11)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image11); // 电量显示
    }
    else if (BUS_Int == 12)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image12); // 电量显示
    }
    else if (BUS_Int == 13)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image13); // 电量显示
    }
    else if (BUS_Int == 14)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image14); // 电量显示
    }
    else if (BUS_Int == 15)
    {
      u8g2.drawXBMP(110, 3, 18, 10, Image15); // 电量显示
    }
  }
}

/**********ADC测温函数********/
void Ntctest() // 控制器温度测量
{
  if (sampleStoreTS <= 36)
  {
    val = val + analogRead(A0);
    sampleStoreTS++;
  }
  else
  {
    val = val / sampleStoreTS;
    V_NTC = (double)val / 1024;
    R_NTC = (Rs * V_NTC) / (Vcc - V_NTC);
    R_NTC = log(R_NTC);
    tem11 = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * R_NTC * R_NTC)) * R_NTC);
    tem11 = tem11 - 273.15;
    sampleStoreTS = 0;
  }
}

/**********全部显示函数********/
void displayALL()
{
  if (Mode2 == 1)
  {
    time4 = millis();
    if ((time4 - time3) >= 400)
    {
      timewarning1 = millis();
      if ((timewarning1 - timewarning) <= 2000)
      {
        if (BusVoltage < 42.9)
        {
          u8g2.clearBuffer();
          batteryDespaly();
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(15, 14);
          u8g2.print("低压报警");
          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(20, 44);
          u8g2.print(BusVoltage);
          u8g2.setCursor(95, 44);
          u8g2.print("V");
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(0, 62);
          u8g2.print("没得电了!快推车!");
          u8g2.sendBuffer();
        }
        if (BusVoltage > 55.9)
        {
          u8g2.clearBuffer();
          batteryDespaly();
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(15, 14);
          u8g2.print("超压报警");
          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(20, 44);
          u8g2.print(BusVoltage);
          u8g2.setCursor(95, 44);
          u8g2.print("V");
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(0, 62);
          u8g2.print("骑慢点!要充炸啦！");
          u8g2.sendBuffer();
          delay(1000);
        }
        if (charge1 == 1)
        {
          u8g2.clearBuffer();
          batteryDespaly();
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(15, 14);
          u8g2.print("放电过流");
          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(20, 44);
          u8g2.print(ShuntCurrent1); // 放电峰值电流
          u8g2.setCursor(95, 44);
          u8g2.print("A");
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(0, 62);
          u8g2.print("电流超载啦!");
          u8g2.sendBuffer();
        }
        if (charge == 1)
        {
          u8g2.clearBuffer();
          batteryDespaly();
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(15, 14);
          u8g2.print("充电过流");
          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(20, 44);
          u8g2.print(ShuntCurrent1); // 充电峰值电流
          u8g2.setCursor(95, 44);
          u8g2.print("A");
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(0, 62);
          u8g2.print("电流超载啦!");
          u8g2.sendBuffer();
        }

        if (temp_read > 55)
        {
          u8g2.clearBuffer();
          batteryDespaly();
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(15, 14);
          u8g2.print("电池过温");
          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(15, 44);
          u8g2.print(temp_read);
          u8g2.drawCircle(90, 20, 2, U8G2_DRAW_ALL);
          // 这里我打不出来℃所以画一个圆上去
          u8g2.setCursor(95, 44);
          u8g2.print("C");
          u8g2.setFont(u8g2_font_wqy16_t_gb2312);
          u8g2.setCursor(0, 62);
          u8g2.print("电池超载啦!");
          u8g2.sendBuffer();
        }
      }
      timewarning2 = millis();
      if ((timewarning1 - timewarning) > 2000 && (timewarning2 - timewarning) <= 3000)
      {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(24, 14);
        u8g2.print("警告！检查！");
        u8g2.setFont(u8g2_font_logisoso24_tr);
        u8g2.setCursor(15, 44);
        u8g2.print("Warning");
        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(30, 62);
        u8g2.print("快点检查!");
        u8g2.sendBuffer();
      }
      if ((timewarning2 - timewarning) > 3000)
      {
        timewarning = timewarning2;
      }
    }
  }

  if (Mode2 == 0)
  {
    time4 = millis();
    if ((time4 - time3) >= 400)
    {
      time3 = time4;
      time5 = millis();
      if ((time5 - time7) / 1000 <= 10)
      {
        u8g2.clearBuffer(); // clear the internal memory
        batteryDespaly();
        u8g2.setFont(u8g2_font_ncenB14_tf);
        u8g2.setCursor(70, 14); // transfer internal memory to the display
        u8g2.print("V");
        u8g2.setCursor(113, 30);
        u8g2.print("A");
        u8g2.setCursor(110, 46);
        u8g2.print("W");
        u8g2.setCursor(83, 64);
        u8g2.print("mAh");

        u8g2.setCursor(0, 14);
        u8g2.print(BusVoltage, 3);

        u8g2.setCursor(0, 30);
        u8g2.print(ShuntCurrent, 3);

        u8g2.setCursor(0, 46);
        u8g2.print(BusPower, 3);

        u8g2.setCursor(0, 64);
        u8g2.print(CapaPower);
        u8g2.sendBuffer();

        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }

      time6 = millis();
      if ((time6 - time7) / 1000 > 10 && (time6 - time7) / 1000 <= 15)
      {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(0, 14);
        u8g2.print("场管温度:");
        u8g2.setCursor(73, 14);
        u8g2.print(tem11);
        u8g2.setCursor(115, 14);
        u8g2.print("℃");

        u8g2.setFont(u8g2_font_logisoso24_tr);
        u8g2.setCursor(15, 44);
        u8g2.print(temp_read);
        u8g2.drawCircle(90, 20, 2, U8G2_DRAW_ALL);
        // 这里我打不出来℃所以画一个圆上去
        u8g2.setCursor(95, 44);
        u8g2.print("C");

        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(0, 62);
        u8g2.print("电池湿度:");
        u8g2.setCursor(73, 62);
        u8g2.print(humi_read);
        u8g2.setCursor(115, 63);
        u8g2.print("%");
        u8g2.sendBuffer();
      }
      timedisplay = millis();
      if ((timedisplay - time7) / 1000 > 15 && (timedisplay - time7) / 1000 <= 20) // 报警信息显示模板
      {
        u8g2.clearBuffer();
        // u8g2.setFont(u8g2_font_unifont_t_chinese2);
        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(2, 14);
        u8g2.print("当前时间(UTC+8)");
        u8g2.setFont(u8g2_font_logisoso24_tr);
        u8g2.setCursor(5, 44);
        u8g2.print(timeClient.getFormattedTime());
        // u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        // u8g2.setCursor(5, 62);
        // u8g2.print("安全骑行小周DIY");

        u8g2.setFont(u8g2_font_wqy16_t_gb2312);
        u8g2.setCursor(0, 62);
        u8g2.print("剩余里程");
        u8g2.setCursor(65, 62);
        u8g2.print(KM);
        u8g2.setCursor(109, 63);
        u8g2.print("Km");
        u8g2.sendBuffer();
      }

      if ((timedisplay - time7) / 1000 > 20)
      {
        time7 = timedisplay;
      }
    }
  }
}

/**********问候语********/
void connectHell()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setCursor(25, 14);
  u8g2.print("尊贵的车主！");
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.setCursor(20, 44);
  u8g2.print(" Hell!");
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setCursor(0, 62);
  u8g2.print("戴头盔注意安全！");
  u8g2.sendBuffer();
  delay(1500);
}

#if defined(ESP8266)
ESP8266WebServer server(80);
#else
WebServer server(80);
#endif
bool configPortalActive = false;

String readStringFromEEPROM(int addr, int maxLen)
{
  if (maxLen > 64)
  {
    maxLen = 64;
  }
  char buf[65];
  memset(buf, 0, sizeof(buf));
  for (int i = 0; i < maxLen; ++i)
  {
    buf[i] = EEPROM.read(addr + i);
    if (buf[i] == 0)
    {
      // found terminator
      break;
    }
  }
  buf[maxLen] = 0;
  return String(buf);
}

void writeStringToEEPROM(int addr, int maxLen, const char *value)
{
  int len = strlen(value);
  for (int i = 0; i < maxLen; ++i)
  {
    if (i < len)
    {
      EEPROM.write(addr + i, value[i]);
    }
    else
    {
      EEPROM.write(addr + i, 0);
    }
  }
}

bool loadWiFiConfig()
{
  EEPROM.begin(EEPROM_SIZE);
  byte flag = EEPROM.read(WIFI_CONFIG_FLAG_ADDR);
  Serial.print("EEPROM flag: 0x");
  Serial.println(flag, HEX);
  if (flag != 0xA5)
  {
    EEPROM.end();
    return false;
  }
  String storedSsid = readStringFromEEPROM(WIFI_CONFIG_SSID_ADDR, WIFI_CONFIG_MAX_SSID);
  String storedPass = readStringFromEEPROM(WIFI_CONFIG_PASS_ADDR, WIFI_CONFIG_MAX_PASS);
  String storedAuth = readStringFromEEPROM(WIFI_CONFIG_AUTH_ADDR, WIFI_CONFIG_MAX_AUTH);
  Serial.println("Loaded WiFi config from EEPROM:");
  Serial.print("  SSID: ");
  Serial.println(storedSsid);
  Serial.print("  PASS: ");
  Serial.println(storedPass);
  Serial.print("  AUTH: ");
  Serial.println(storedAuth);
  if (storedSsid.length() == 0 || storedPass.length() == 0)
  {
    EEPROM.end();
    Serial.println("Stored SSID or PASS empty");
    return false;
  }
  storedSsid.toCharArray(ssid, WIFI_CONFIG_MAX_SSID);
  storedPass.toCharArray(pswd, WIFI_CONFIG_MAX_PASS);
  if (storedAuth.length() > 0)
  {
    storedAuth.toCharArray(auth, WIFI_CONFIG_MAX_AUTH);
  }
  EEPROM.end();
  Serial.println("WiFi config loaded OK");
  return true;
}

void saveWiFiConfig(const char *newSsid, const char *newPass, const char *newAuth)
{
  EEPROM.begin(EEPROM_SIZE);
  writeStringToEEPROM(WIFI_CONFIG_SSID_ADDR, WIFI_CONFIG_MAX_SSID, newSsid);
  writeStringToEEPROM(WIFI_CONFIG_PASS_ADDR, WIFI_CONFIG_MAX_PASS, newPass);
  writeStringToEEPROM(WIFI_CONFIG_AUTH_ADDR, WIFI_CONFIG_MAX_AUTH, newAuth);
  EEPROM.write(WIFI_CONFIG_FLAG_ADDR, 0xA5);
  bool ok = EEPROM.commit();
  Serial.println();
  Serial.println("Saving WiFi config...");
  Serial.print("  SSID: ");
  Serial.println(newSsid);
  Serial.print("  PASS: ");
  Serial.println(newPass);
  Serial.print("  AUTH: ");
  Serial.println(newAuth);
  Serial.print("  commit: ");
  Serial.println(ok ? "OK" : "FAILED");
  EEPROM.end();
}

bool connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pswd);
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000)
  {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi connect failed");
  return false;
}

void handleRoot()
{
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
  server.send(200, "text/html", page);
}

void handleSave()
{
  if (!server.hasArg("ssid") || !server.hasArg("pswd") || !server.hasArg("auth"))
  {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pswd");
  String newAuth = server.arg("auth");
  if (newSsid.length() == 0 || newPass.length() == 0)
  {
    server.send(400, "text/plain", "SSID and password cannot be empty");
    return;
  }
  if (newAuth.length() == 0)
  {
    newAuth = String(auth);
  }
  saveWiFiConfig(newSsid.c_str(), newPass.c_str(), newAuth.c_str());
  String page = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>EV Meter 配网</title></head><body>";
  page += "<h2>配置已保存</h2>";
  page += "<p>设备即将重启...</p>";
  page += "</body></html>";
  server.send(200, "text/html", page);
  delay(2000);
  ESP.restart();
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

void startConfigPortal()
{
  configPortalActive = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("EVMeter_Config");
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Config portal started. Connect to WiFi SSID: EVMeter_Config and open http://192.168.4.1/");
  while (true)
  {
    server.handleClient();
    delay(2);
  }
}

/**
 * @brief 初始化 INA226 电流/电压检测芯片
 *
 * 功能：
 * 1. 初始化 I2C 总线
 * 2. 检测 INA226 是否响应
 * 3. 读取 INA226 配置寄存器，确认通信正常
 * 4. 配置 INA226 工作模式
 * 5. 设置分流电阻参数并进行校准
 * 6. 读取母线电压测试 INA226 工作状态
 *
 * @return true  初始化成功
 * @return false 初始化失败
 */
bool initINA226()
{
  Serial.println("Initializing INA226...");
  // 初始化 I2C 总线
  // ESP8266 默认 SDA/SCL 根据开发板定义
  // ESP32 可以通过 Wire.begin(SDA, SCL) 指定引脚
  Wire.begin();
  // 等待I2C外设稳定
  delay(100);

  /*
   * 检测 INA226 的 I2C 地址是否存在
   *
   * INA226 默认地址通常为：
   * 0x40
   * 如果 A0/A1 引脚配置不同，地址可能为：
   * 0x41 / 0x44 / 0x45 等
   *
   * 当前程序使用 0x44
   */
  Wire.beginTransmission(0x44);
  // 结束一次 I2C 通信
  // 返回 0 表示设备响应正常
  uint8_t error = Wire.endTransmission();
  // 如果返回错误，说明 INA226 没有响应
  if (error != 0)
  {
    Serial.print("INA226 init failed: I2C address 0x44 no response, error=");
    Serial.println(error);
    // 扫描整个 I2C 总线，帮助查找实际地址
    scanI2CDevices();
    return false;
  }

  Serial.println("INA226 answered on I2C at 0x44");

  Wire.beginTransmission(0x44);
  Wire.write(0x00);
  error = Wire.endTransmission(false);
  if (error != 0)
  {
    Serial.print("INA226 config register access failed, error=");
    Serial.println(error);
    return false;
  }

  // Wire.requestFrom(0x44, (uint8_t)2);
  Wire.requestFrom((uint8_t)0x44, (uint8_t)2);
  if (Wire.available() < 2)
  {
    Serial.println("INA226 config register read timeout");
    return false;
  }

  uint16_t configReg = ((uint16_t)Wire.read() << 8) | Wire.read();
  Serial.print("INA226 config register = 0x");
  Serial.println(configReg, HEX);

  ina.begin(0x44);
  delay(100);
  /*
   * 配置 INA226 工作参数
   *
   * 参数说明：
   *
   * INA226_AVERAGES_128
   * -------------------
   * 内部采样平均次数：
   * 128 次平均
   * 优点：
   * - 降低电流噪声
   * - 提高稳定性
   * INA226_BUS_CONV_TIME_1100US
   * ---------------------------
   * 总线电压转换时间：
   * 1100us
   * INA226_SHUNT_CONV_TIME_1100US
   * -----------------------------
   * 分流电压转换时间：
   * 1100us
   * INA226_MODE_SHUNT_BUS_CONT
   * ---------------------------
   * 连续测量模式：
   * 同时测量：
   * - 分流电压
   * - 母线电压
   */
  ina.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  /*
   * INA226 校准
   *
   * 参数：
   *
   * 0.002
   * -----
   * 分流电阻阻值
   *
   * 单位：Ω
   *
   * 即：2mΩ
   * 40
   * ---
   * 最大测量电流
   *
   * 单位：A
   *
   * 根据实际硬件：
   * Rsense = 2mΩ
   * 最大电流 = 40A
   */
  ina.calibrate(0.002, 40);
  delay(100);

  float rawVoltage = ina.readBusVoltage();
  Serial.print("INA226 raw bus voltage = ");
  Serial.print(rawVoltage, 4);
  Serial.println(" V");
  Serial.print("INA226 scaled bus voltage = ");
  Serial.print(rawVoltage * Vscale, 4);
  Serial.println(" V");

  return true;
}

void setup()
{
  Serial.begin(115200);

  BLINKER_DEBUG.stream(Serial);
  BLINKER_DEBUG.debugAll();
  if (!loadWiFiConfig())
  {
    Serial.println("没有存储的 WiFi 配置，进入配网模式...");
    startConfigPortal();
  }
  Serial.print("Using SSID: ");
  Serial.println(ssid);
  Serial.print("Using PASS: ");
  Serial.println(pswd);
  if (!connectWiFi())
  {
    Serial.println("WiFi 连接失败，进入配网模式...");
    startConfigPortal();
  }
  Blinker.begin(auth, ssid, pswd);
  Blinker.attachHeartbeat(heartbeat);
  Blinker.attachDataStorage(dataStorage);
  BlinkerMIOT.attachPowerState(miotPowerState); // 小爱电源控制
  pinMode(D6, OUTPUT);
  digitalWrite(D6, HIGH);
  pinMode(D7, OUTPUT);
  // 蜂鸣器
  pinMode(D5, OUTPUT);
  digitalWrite(D5, LOW);
  Button2.attach(button2_callback); // 绑定按键回调
  Button9.attach(button9_callback); // 绑定按键回调
  Button7.attach(button7_callback); // 绑定按键回调
  Button8.attach(button8_callback); // 绑定按键回调
  Button6.attach(button6_callback); // 绑定按键回调
  Blinker.attachData(dataRead);
  dht.begin(); // DTHXX传感器初始化

  delay(500);

  timeClient.begin();
  /*****************************************************/
  Serial.println("Initialize INA226");
  Serial.println("-----------------------------------------------");

  ina226Ready = initINA226();
  if (ina226Ready)
  {
    // Display configuration
    checkConfig();
  }
  else
  {
    Serial.println("INA226 initialization failed, please check I2C wiring, power and address.");
  }

  Serial.println("-----------------------------------------------");

  u8g2.begin();
  u8g2.enableUTF8Print();
  // u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // u8g2.setFont(u8g2_font_ncenB14_tf ); // choose a suitable font   u8g2_font_ncenB14_tf

  // 显示等待me
  /*u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese2);
  u8g2.setCursor(0, 14);
  u8g2.print("Waiting for me");
  u8g2.setCursor(0, 30);
  u8g2.print("connection...");
  u8g2.sendBuffer();
  delay(1000);*/
  displaygif();
  connectHell();

  /*****************************************************/
}

void loop()
{
  Blinker.run();
  /******************************************/
  /*NTP时间获取*/
  timeClient.update();
  Serial.println(timeClient.getFormattedTime());

  /*if(timeClient.isTimeSet()) {
    if (hour == timeClient.getHours() && minute == timeClient.getMinutes())
    {
      BusPower1 = BusPower;
    }
  }*/

  Runtime();
  time2 = millis();
  if ((time2 - time1) >= 200)
  {
    BusVoltage = (ina.readBusVoltage() * Vscale);
    ShuntCurrent = ina.readShuntCurrent();
    BusPower = (ina.readBusPower() * Vscale);
    Sa = ShuntCurrent * (time2 - time1) / 3600;
    time1 = time2;
    CapaPower = CapaPower + Sa;
    Da = Da + BusVoltage * Sa / 1000;
    ta = (BusVoltage - 42.9) / 11.7 * 100;
    if (ta >= 100)
    {
      ta = 100;
    }
    if (ta <= 0)
    {
      ta = 0;
    }

    KM = (mileage * ta) / 100;

    if (ShuntCurrent >= 0)
    {
      if (BusPower >= BusPower1)
      {
        BusPower1 = BusPower;
      }
      if (ShuntCurrent >= ShuntCurrent1)
      {
        ShuntCurrent1 = ShuntCurrent;
      }
    }
    else if (ShuntCurrent < 0)
    {
      BusPower2 = (0 - BusPower);
      if (BusPower1 >= BusPower2)
      {
        BusPower1 = BusPower2;
      }
      if (ShuntCurrent < ShuntCurrent1)
      {
        ShuntCurrent1 = ShuntCurrent;
      }
    }

    time9 = millis();
    if ((time9 - time8) >= 2000)
    {
      time8 = time9;
      if (temp_read > 55)
      {
        Blinker.notify(String("温度过高！") + String(String(temp_read) + String("℃")));
      }
      if (BusVoltage < 42.9)
      {
        Blinker.notify(String("电压过低！") + String(String(BusVoltage) + String("V")));
      }

      if (BusVoltage > 55.25)
      {
        Blinker.notify(String("电压过高！") + String(String(BusVoltage) + String("V")));
      }
    }

    /*NTC温度计算*/
    Ntctest(); // ADC端口采集温度
    /*DHT11读温湿度*/
    h = dht.readHumidity();
    t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      BLINKER_LOG("Failed to read from DHT sensor!");
    }
    else
    {
      BLINKER_LOG("Humidity: ", h, " %");
      BLINKER_LOG("Temperature: ", t, " *C");
      humi_read = h;
      temp_read = t;
    }
    // DHT11读温湿度
  }

  if (Mode1 == 0)
  {
    if (BusVoltage > 42.9 && BusVoltage <= 55.9)
    {
      digitalWrite(D6, HIGH); // 打开输出
      digitalWrite(D5, LOW);
      Mode2 = 0;
    }
    else if (BusVoltage <= 42.9) // 低压保护
    {
      digitalWrite(D6, LOW); // 关闭输出
      Mode2 = 1;
      timebuzzer1 = millis();
      if ((timebuzzer1 - timebuzzer) >= 2000)
      {
        timebuzzer = timebuzzer1;
        digitalWrite(D5, !digitalRead(D5)); // 蜂鸣器2秒响一次
      }
    }
  }
  else
  {
    if (Buttonmode == 1)
    {
      digitalWrite(D6, HIGH);
    }
    else if (Buttonmode == 0)
    {
      digitalWrite(D6, LOW);
    }
  }

  if (BusVoltage > 55.9) // 充电超压保护
  {
    Buttonmode = 0;
    digitalWrite(D6, LOW); // 关闭输出
    Mode2 = 1;
    timebuzzer1 = millis();
    if ((timebuzzer1 - timebuzzer) >= 1000)
    {
      timebuzzer = timebuzzer1;
      digitalWrite(D5, !digitalRead(D5)); // 蜂鸣器1秒响一次
    }
  }
  if (temp_read > 55) // 超温保护
  {
    digitalWrite(D6, LOW); // 关闭输出
    digitalWrite(D5, HIGH);
    Buttonmode = 0;
    Mode1 = 1; // 待温度降下来后手动解封
    Mode2 = 1;
  }

  if (ShuntCurrent < (-25)) // 过流保护
  {
    digitalWrite(D6, LOW); // 关闭输出
    Buttonmode = 0;
    Mode1 = 1; // 手动解封
    Mode2 = 1;
    charge1 = 1;
  }
  if (charge1 == 1)
  {
    timebuzzer1 = millis();
    if ((timebuzzer1 - timebuzzer) >= 3000)
    {
      timebuzzer = timebuzzer1;
      digitalWrite(D5, !digitalRead(D5)); // 蜂鸣器3秒响一次
    }
  }

  if (ShuntCurrent > 5) // 充电过流保护
  {
    digitalWrite(D6, LOW); // 关闭输出
    Buttonmode = 0;
    Mode1 = 1; // 手动解封
    Mode2 = 1;
    charge = 1;
  }
  if (charge == 1)
  {
    timebuzzer1 = millis();
    if ((timebuzzer1 - timebuzzer) >= 4000)
    {
      timebuzzer = timebuzzer1;
      digitalWrite(D5, !digitalRead(D5)); // 蜂鸣器3秒响一次
    }
  }
  if (Displaymode == 0)
  {
    displayALL();
  }
}
