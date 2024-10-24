
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <arduino_base64.hpp>
#include <CRC16.h>
#include <CRC8.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <AS5600.h>
#include <assert.h>

#include "setups.h"
#include "amslite.h"

// 引脚的分配
#define RS485 Serial1
#define RS485_RX_PIN  16
#define RS485_TX_PIN  17
#define RS485_RTS_PIN 4
#define AS5600_SDA_PIN  21
#define AS5600_SCL_PIN  22
#define MOTOR0_PIN1   27
#define MOTOR0_PIN2   26
#define MOTOR1_PIN1   12
#define MOTOR1_PIN2   13
#define SERVO0_PIN   14
#define SERVO1_PIN   25
#define CD74HC4067_S0_PIN 19
#define CD74HC4067_S1_PIN 18

const float EXTRUDER_GEAR_DIAMETER = 7.41;

// 开启调试模式，esp32 将不会连接拓竹
#define __DEBUG__

// 拓竹指令
// 执行 Unload 指令，打印机将开始自动加热热端，并切断线材。
const char* bambu_unload = "{\
  \"print\": {\
    \"command\": \"ams_change_filament\",\
    \"curr_temp\": 210,\
    \"sequence_id\": \"zp-ams-1\",\
    \"tar_temp\": 210,\
    \"target\": 255\
  }\
}";

const char* bambu_load = "{\
  \"print\": {\
    \"command\": \"ams_change_filament\",\
    \"curr_temp\": 210,\
    \"sequence_id\": \"zp-ams-1\",\
    \"tar_temp\": 210,\
    \"target\": 254\
  }\
}";

const char* bambu_done = "{\
  \"print\": {\
    \"command\": \"ams_control\",\
    \"param\": \"done\",\
    \"sequence_id\":\"zp-ams-1\"\
  },\
  \"user_id\": \"mqttx_c59bbf21\"\
}";

// 重试|继续打印
const char* bambu_resume = "{\
  \"print\": {\
    \"command\": \"resume\",\
    \"sequence_id\": \"zp-ams-1\"\
  },\
  \"user_id\": \"mqttx_c59bbf21\"\
}";

const char* bambu_gcode_m109 = "{\
  \"print\": {\
    \"command\": \"gcode_line\",\
    \"sequence_id\": \"zp-ams-1\",\
    \"param\": \"M109 S\"220\"\
  },\
  \"user_id\": \"mqttx_c59bbf21\"\
}";

const char* bambu_pushall = "{\
  \"pushing\": {\
    \"sequence_id\": \"zp-ams-1\",\
    \"command\": \"pushall\"\
  }\
}";

WiFiClientSecure wifi_client;
PubSubClient bambu_client(wifi_client);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

class Config {
public:
  JsonDocument m_data;
  void setup() {
    if (LittleFS.exists("/config.json")) {
      File file = LittleFS.open("/config.json", "r");
      deserializeJson(m_data, file);
      file.close();
    }
  }
  template <typename T>
  T get(const char* key, T default_value) {
    if (m_data[key].is<T>()) {
      return m_data[key].as<T>();
    } else {
      return default_value;
    }
  }
  void save() {
    File file = LittleFS.open("/config.json", "w");
    serializeJson(m_data, file);
    file.close();
  }
};

Config s_config;

AS5600 as5600;   //  use default Wire

// 减速马达，通过 DRV8833 控制
class Motor {
public:
  int m_pin1;
  int m_pin2;

  void setup(int pin1, int pin2) {
    m_pin1 = pin1;
    m_pin2 = pin2;
    pinMode(m_pin1, OUTPUT);
    pinMode(m_pin2, OUTPUT);
    stop();
  }
  // 正转
  void forward() {
    digitalWrite(m_pin1, HIGH);
    digitalWrite(m_pin2, LOW);
  }
  // 反转
  void reverse() {
    digitalWrite(m_pin1, LOW);
    digitalWrite(m_pin2, HIGH);
  }
  void stop() {
    digitalWrite(m_pin1, HIGH);
    digitalWrite(m_pin2, HIGH);
  }
};

class AMSLite {
public:
  Motor m_motor0;
  Motor m_motor1;
  Servo m_servo0;
  Servo m_servo1;
  int m_servo0_init = 90;
  int m_servo1_init = 90;
  int m_servo_power = 30;
  float m_p = 0;
  float m_x = -1;

  void setup(int m0pin1, int m0pin2, int m1pin1, int m1pin2, int s0pin, int s1pin) {
    m_motor0.setup(m0pin1, m0pin2);
    m_motor1.setup(m1pin1, m1pin2);
    m_servo0.attach(s0pin);
    // m_servo1.attach(s1pin);
    m_servo0_init = s_config.get("servo0_init", 90);
    m_servo1_init = s_config.get("servo1_init", 90);
    m_servo_power = s_config.get("servo_power", 30);
  }

  void forward(int id) {
    // A1 与 A2 一组
    if (id == 0) {
      m_motor0.forward();
      m_servo0.write(m_servo0_init - m_servo_power);
    } else if (id == 1) {
      m_motor0.reverse();
      m_servo0.write(m_servo0_init + m_servo_power);
    }
  }

  void backward(int id) {
    if (id == 0) {
      m_motor0.reverse();
      m_servo0.write(m_servo0_init - m_servo_power);
    } else if (id == 1) {
      m_motor0.forward();
      m_servo0.write(m_servo0_init + m_servo_power);
    }
  }
  void _stop() {
    m_motor0.stop();
    m_motor1.stop();
    m_servo0.write(m_servo0_init);
  }
  void stop() {
    m_motor0.stop();
    m_motor1.stop();
    // m_servo0.write(m_servo0_init);
    int power = m_servo0.read();
    if (power < m_servo0_init - 10) {
       m_servo0.write(m_servo0_init + 5);
    } else if (power > m_servo0_init + 10) {
       m_servo0.write(m_servo0_init - 5);
    }
    /*
    power = m_servo1.read();
    if (power < m_servo1_init - 10) {
       m_servo1.write(m_servo1_init + 5);
    } else if (power > m_servo1_init + 10) {
       m_servo1.write(m_servo1_init - 5);
    }
    */
  }
  int m_status = 0;
  void stop_ex() {
    m_status = 1;
    m_motor0.stop();
  }
  void stop(float p, float x) {
    // 在继续推/拉 x 毫米后停止
    m_p = p;
    m_x = x;
  }
  void loop() {
    if (m_status == 1) {
      int power = m_servo0.read();
      if (power < m_servo0_init - 10) {        // A1
        m_servo0.write(m_servo0_init + 5);     // 舵机和马达合力回正
        m_motor0.reverse();
      } else if (power > m_servo0_init + 10) {
        m_servo0.write(m_servo0_init - 5);
        m_motor0.forward();
      } else {
        m_motor0.stop();
        m_status = 0;
        // m_motor1.stop();
      }
    }
    if (m_x <= 0) {
      return;
    }
    float p = as5600.getCumulativePosition() * PI * EXTRUDER_GEAR_DIAMETER / (1 << 12);
    if (abs(p - m_p) > m_x) {
      stop();
      m_x = -1;
    }
  }
};

AMSLite ams_lite;
AMSLite &amslite = ams_lite;

double get_arg(AsyncWebServerRequest *request, const char* name, double default_value = 0.0) {
  if (request->hasParam(name)) {
    return request->getParam(name)->value().toDouble();
  }
  return default_value;
}

// 打印机通过 mqtt 发送来的信息
// -1 表示未知
int print_error = -1;
int ams_status = -1;
// 进料开关状态，0 无料，1 有料
int hw_switch_state = -1;
// 黑客入侵[打印进度]，用[mc_percent - 110]表示接下来期望换用的挤出机id
int mc_percent = -1;
String gcode_state;

// ZP AMS 状态:
// 自动换料的状态，0 空闲，1 忙碌
int zp_state = 0;
// 有待退料管道
int previous_extruder = 0;
// 有待进料管道
int next_extruder = 0;

void get_config(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  serializeJson(s_config.m_data, *response);
  if (bambu_client.connected()) {
    bambu_client.publish(s_config.m_data["bambu_topic_publish"], bambu_pushall);
  }
  request->send(response);
}

void put_config(AsyncWebServerRequest *request) {
  const AsyncWebParameter* param = nullptr;
  param = request->getParam("WiFi_ssid");
  if (param) {
    s_config.m_data["WiFi_ssid"] = param->value();
  }
  param = request->getParam("WiFi_passphrase");
  if (param) {
    s_config.m_data["WiFi_passphrase"] = param->value();
  }
  // 如果没有联网，则进行连接；如果已经联网，则忽略
  const String& ssid = s_config.m_data["WiFi_ssid"].as<String>();
  const String& passphrase = s_config.m_data["WiFi_passphrase"].as<String>();
  if (WiFi.status() != WL_CONNECTED && !ssid.isEmpty() && !passphrase.isEmpty()) {
    WiFi.begin(ssid, passphrase);
  }
  param = request->getParam("mode");
  if (param) {
    s_config.m_data["mode"] = param->value();
  }
  param = request->getParam("phone_number");
  if (param) {
    s_config.m_data["phone_number"] = param->value();
  }
  param = request->getParam("password");
  if (param) {
    s_config.m_data["password"] = param->value();
  }
  param = request->getParam("bambu_mqtt_broker");
  if (param) {
    s_config.m_data["bambu_mqtt_broker"] = param->value();
  }
  param = request->getParam("bambu_mqtt_password");
  if (param) {
    s_config.m_data["bambu_mqtt_password"] = param->value();
  }
  param = request->getParam("bambu_device_serial");
  if (param) {
    const String& bambu_device_serial = param->value();
    s_config.m_data["bambu_device_serial"] = bambu_device_serial;
    s_config.m_data["bambu_topic_subscribe"] = "device/" + bambu_device_serial + "/report";
    s_config.m_data["bambu_topic_publish"] = "device/" + bambu_device_serial + "/request";
  }
  param = request->getParam("servo0_init");
  if (param) {
    s_config.m_data["servo0_init"] = param->value().toInt();
    ams_lite.m_servo0_init = param->value().toInt();
  }
  param = request->getParam("servo1_init");
  if (param) {
    s_config.m_data["servo1_init"] = param->value().toInt();
    ams_lite.m_servo1_init = param->value().toInt();
  }
  param = request->getParam("servo_power");
  if (param) {
    s_config.m_data["servo_power"] = param->value().toInt();
    ams_lite.m_servo_power = param->value().toInt();
  }
  s_config.save();
  request->send(200);
}

void get_local_ip(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  JsonDocument data;
  if (WiFi.status() == WL_CONNECTED) {
    data["local_ip"] = WiFi.localIP().toString();
  }
  serializeJson(data, *response);
  request->send(response);
}

void unload(AsyncWebServerRequest* request) {
  if (gcode_state != "FINISH" && gcode_state != "FAILURE") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  previous_extruder = get_arg(request, "previous_extruder", 0);
  bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_unload);
  request->send(200);
}

void load(AsyncWebServerRequest* request) {
  if (gcode_state != "FINISH") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  next_extruder = get_arg(request, "next_extruder", 0);
  bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_load);
  request->send(200);
}

void stop(AsyncWebServerRequest* request) {
  ams_lite._stop();
  previous_extruder = get_arg(request, "previous_extruder");
  next_extruder = get_arg(request, "next_extruder");
  request->send(200);
}

void resume(AsyncWebServerRequest* request) {
  bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_resume);
  request->send(200);
}

void gcode_m109(AsyncWebServerRequest* request) {
  bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_gcode_m109);
  request->send(200);
}

void test_forward(AsyncWebServerRequest* request) {
  #ifndef __DEBUG__
  // FINISH
  if (gcode_state != "FINISH") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  #endif
  next_extruder = get_arg(request, "next_extruder", 0);
  ams_lite.forward(next_extruder);
  previous_extruder = next_extruder;
  request->send(200);
}

void test_backward(AsyncWebServerRequest* request) {
  #ifndef __DEBUG__
  if (gcode_state != "FINISH") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  #endif
  previous_extruder = get_arg(request, "previous_extruder", 0);
  ams_lite.backward(previous_extruder);
  next_extruder = previous_extruder;
  request->send(200);
}

void restart(AsyncWebServerRequest* request) {
  request->send(200);
  ESP.restart();
}

void wifi_setup() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("zhaipro-amslite", "zhaipro-amslite");
  if (!s_config.m_data["WiFi_ssid"].is<const char*>() || !s_config.m_data["WiFi_passphrase"].is<const char*>()) {
    return;
  }
  const String& ssid = s_config.m_data["WiFi_ssid"];
  const String& passphrase = s_config.m_data["WiFi_passphrase"];
  if (ssid.isEmpty() || passphrase.isEmpty()) {
    return;
  }
  // 最终常亮表示成功，常灭表示失败
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.printf("Connecting to %s ", ssid.c_str());
  WiFi.begin(ssid, passphrase);
  for (int i = 0; i < 7; i++) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED off by making the voltage LOW
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                      // wait for a second
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.disconnect();
    Serial.println(" failed");
  }
}

void bambu_callback(char* topic, byte* payload, unsigned int length) {
  // https://arduinojson.org/v7/api/jsondocument/
  JsonDocument data;
  deserializeJson(data, payload, length);
  if (!data["print"].is<JsonObject>()) {
    // 收到未知信息，直接不理睬
    return;
  }

  JsonDocument _data;
  const char* sequence_id = data["print"]["sequence_id"];
  if (data["print"]["hw_switch_state"].is<int>()) {
    hw_switch_state = data["print"]["hw_switch_state"];
    _data["hw_switch_state"] = hw_switch_state;
  }
  if (data["print"]["gcode_state"].is<const char*>()) {
    gcode_state = data["print"]["gcode_state"].as<String>();
    _data["gcode_state"] = gcode_state;
  }
  if (data["print"]["mc_percent"].is<int>()) {
    mc_percent = data["print"]["mc_percent"];
    _data["mc_percent"] = mc_percent;
  }
  if (gcode_state != "PAUSE") {
    // 如果打印机不空闲，那么我必空闲
    zp_state = 0;
  } else if (mc_percent > 100 && zp_state == 0) {
    zp_state = 1;
    // 打印机处于暂停状态，且收到黑客请求，且处于空闲状态
    next_extruder = mc_percent - 110;
    if (hw_switch_state == -1) {
      // 当前状态未知？？？error error error
      return;
    }
    if (hw_switch_state == 0) {
      // 无料，直接进料
      bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_load);
    } else if (next_extruder != previous_extruder) {
      // 换料，先退料
      bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_unload);
    } else {
      // 直接点完成吧
      bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_resume);
    }
  }
  if (data["print"]["ams_status"].is<int>()) {
    ams_status = data["print"]["ams_status"];
    _data["ams_status"] = ams_status;
    Serial.printf("bambu sequence_id: \"%s\" ams_status: %d\n", sequence_id, ams_status);

    if (ams_status == 260) {
      // 请回抽
      ams_lite.backward(previous_extruder);
    } if (ams_status == 261) {
      // 请推入
      ams_lite.forward(next_extruder);
    } else if (ams_status == 262) {
      // 推入完成
      ams_lite.stop();
    } else if (ams_status == 768) {
      // 完成换料
      previous_extruder = next_extruder;
      if (zp_state == 1) {
        bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_resume);
      }
    } else if (ams_status == 0) {
      // 完成退料，但还要继续拔出一段
      delay(1000);
      ams_lite.stop();
      if (zp_state == 1) {
        bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_load);
      }
    }
    /*
    0    空闲 or 完成退料？
    258  加热中
    259  裁剪耗材中
    260  请回抽
    261  请推入
    262  检测到进料，hw_switch_state = 1
    263  清理
    768  完成换料 or 完成进料
    1280 完成
    */
  }
  
  if (data["print"]["print_error"].is<int>()) {
    print_error = data["print"]["print_error"];
    _data["print_error"] = print_error;
    Serial.printf("bambu sequence_id: \"%s\" print_error: %d\n", sequence_id, print_error);
    // 318750726 0b1001011111111 11000000 00000110 请推入耗材？
    // 318734342 0b1001011111111 11001110 00100110 没检测到进料？
    // 318750723 0b1001011111111 11000000 00000011 请拔出耗材？
    // 318734339 0b1001011111111 10000000 00000011 拔出耗材
    // 318734343 0b1001011111111 10000000 00000111 是否完成换料？
    if (print_error == 318734343) {
      // 弹窗询问：“是否完成换料？”，我们点击完成
      bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_done);
    }
  }
  if (_data.size()) {
    char buffer[256];
    _data["sequence_id"] = sequence_id;
    serializeJson(_data, buffer, sizeof(buffer));
    ws.textAll(buffer);
  }
}

void bambu_setup() {
  // https://pubsubclient.knolleary.net/
  wifi_client.setInsecure();
  bambu_client.setCallback(bambu_callback);
  bambu_client.setBufferSize(4096);   // 其默认值 256 太小啦
}

void on_post_amslite_status(AsyncWebServerRequest* request);

void wifi_server_setup() {
  server.rewrite("/", "/index.html");
  server.on("/unload", unload);
  server.on("/load", load);
  server.on("/stop", stop);
  server.on("/resume", resume);
  server.on("/gcode_m109", gcode_m109);
  server.on("/test_forward", test_forward);
  server.on("/test_backward", test_backward);
  server.on("/put_config", put_config);
  server.on("/get_config", get_config);
  server.on("/get_local_ip", get_local_ip);
  server.on("/restart", restart);
  server.on("/amslite_status", HTTP_POST, on_post_amslite_status);
  server.addHandler(&ws);
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("HTTP server started");
}

void as5600_setup() {
  pinMode(CD74HC4067_S0_PIN, OUTPUT);
  pinMode(CD74HC4067_S1_PIN, OUTPUT);
  digitalWrite(CD74HC4067_S0_PIN, LOW);
  digitalWrite(CD74HC4067_S1_PIN, LOW);
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  bool ret = as5600.begin();
  Serial.printf("as5600.begin() => %d\n", ret);
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600.resetCumulativePosition();
}

typedef struct {
  uint8_t temp5;
  uint8_t temp6;
  uint8_t id;
  uint8_t status;
} extruder_t;

typedef struct {
  uint8_t temp5;
  uint8_t temp6;
  uint8_t status;
  uint8_t temp8;
  uint8_t id;
  
} extruder_ex_t;

typedef struct {
  uint8_t r, g, b, a;
} color_t;

typedef struct {
  uint8_t index;
  uint8_t temp;
  uint8_t id[8];
  color_t color;
  uint16_t temperature_min;
  uint16_t temperature_max;
  uint8_t name[20];   // 耗材的名称如：PLA
} filament_t;
static_assert(sizeof(filament_t) == 38);

#pragma pack (1)
// 这里多了NFC相关的内容
typedef struct {
  uint8_t ams_num;
  uint8_t index;
  uint8_t temp1[17];
  uint8_t id[8];
  uint8_t name[20];
  uint8_t NFC0[12];
  color_t color;
  uint8_t NFC1[16];
  uint16_t temperature_min;
  uint16_t temperature_max;
  uint8_t NFC2[48];
} filament_ex_t;

typedef struct {
  uint8_t head;         // 帧头 0x3D
  uint8_t type;
  union {
    struct {            // type: 0x80
      uint8_t size;
      uint8_t rv;       // crc8
      uint8_t cmd;
      union {
        filament_t filament;
        extruder_t extruder;
        extruder_ex_t extruder_ex;
        uint16_t divice_id;
      } data;
    } body_80;
    struct {            // type: 0x00
      uint16_t package_number;
      uint16_t size;    // 4
      uint8_t rv;       // crc8
      uint16_t target_address;    // amslite(0x1200) or ams(0x0700)
      uint16_t source_address;    // a1mini(0x0600)?
      uint16_t type;
      union {
        filament_ex_t filament_ex;
      } data;
    } body_00;
  };
  uint16_t __rv;        // 占地用的
} bambu_data_t;
#pragma pack ()

static_assert(sizeof(bambu_data_t) == 146, "");

// 这里后面都预留很多不用的空间
uint8_t firmware_version_res[25 + 11] = {0x3D, 0x00, 0x6A, 0x00, 36, 0x00, 0xC0, 0x00,
                                  0x09, 0x00, 0x12,
                                  0x03, 0x01,                                                 // 命令号
                                  94, 07, 00, 00,                                             // 我们伪装(AMS Lite)的版本: 00.00.07.94
                                        0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32};      // AMS_F102
uint8_t hardware_serial_number_res[80] = {0x3D, 0x00, 0xB3, 0x00, 80, 0x00, 0x28, 0x00,
                                  0x09, 0x00, 0x12, 0x02, 0x04, 0x0F,                         // 序列号长度：15位
  0x30, 0x33, 0x43, 0x31, 0x32, 0x41, 0x33, 0x43, 0x30, 0x34, 0x30, 0x30, 0x35, 0x32, 0x39};  // 我们伪装(AMS Lite)的序列号: 03C12A3C0400529

#define C_test 0x00, 0x00, 0x00, 0xFF, \
               0x00, 0x00, 0x80, 0xBF, \
               0x00, 0x00, 0x00, 0xC0, \
               0x00, 0xC0, 0x5D, 0xFF, \
               0xFC, 0xFF, 0xFC, 0xFF, \
               0x00, 0x00, 0x44, 0x00, \
               0x55,                   \
               0xC1, 0xC3, 0xEC, 0xBC, \
               0x01, 0x01, 0x01, 0x01,
unsigned char Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, 0x00, 0x00, 0x00,
                           0x04, 0x04, 0x04, 0xFF, // flags
                           0x00, 0x00, 0x00, 0x00,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0x64, 0x64, 0x64, 0x64,
                           0x90, 0xE4};
  unsigned char Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                            C_test 0x00, 0x00, 0x00, 0x00,
                            0x90, 0xE4};

class BL3DPrinter {
public:
  CRC16 m_crc16;
  CRC8 m_crc8;
  filament_ex_t m_filaments[4];

  BL3DPrinter():m_crc16(0x1021, 0x913D, 0, false, false), m_crc8(0x39, 0x66, 0, false, false) {
  }
  void setup() {
    RS485.begin(1228800, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);
    if (!RS485.setPins(-1, -1, -1, RS485_RTS_PIN)) {
      Serial.print("Failed to set RS485 pins");
    }
    // Certain versions of Arduino core don't define MODE_RS485_HALF_DUPLEX and so fail to compile.
    // By using UART_MODE_RS485_HALF_DUPLEX defined in hal/uart_types.h we work around this problem.
    // If using a newer IDF and Arduino core you can omit including hal/uart_types.h and use MODE_RS485_HALF_DUPLEX
    // defined in esp32-hal-uart.h (included during other build steps) instead.
    if (!RS485.setMode(UART_MODE_RS485_HALF_DUPLEX)) {
      Serial.print("Failed to set RS485 mode");
    }

    if (LittleFS.exists("/filaments.bin")) {
      File file = LittleFS.open("/filaments.bin", "rb");
      size_t size = file.read((uint8_t*)m_filaments, sizeof(m_filaments));
      if (size != sizeof(m_filaments)) {
        Serial.printf("filaments.bin file size %d does not equal to %d\n", size, sizeof(m_filaments));
        memset(m_filaments, 0, sizeof(m_filaments));
      }
      file.close();
    } else {
      memset(m_filaments, 0, sizeof(m_filaments));
    }
  }
  void send(bambu_data_t *data) {
    size_t size;
    m_crc8.restart();
    if (data->type & 0x80) {
      m_crc8.add((uint8_t*)data, 3);
      data->body_80.rv = m_crc8.calc();
      size = data->body_80.size;
    } else {
      m_crc8.add((uint8_t*)data, 6);
      data->body_00.rv = m_crc8.calc();
      size = data->body_00.size;
    }
    m_crc16.restart();
    m_crc16.add((uint8_t*)data, size - 2);
    int rv = m_crc16.calc();
    ((uint8_t*)data)[size - 2] = rv & 0xFF;
    ((uint8_t*)data)[size - 1] = rv >> 8;
    RS485.write((uint8_t*)data, size);
  }

  uint8_t m_buffer[255];
  size_t m_end = 0;
  enum {
    BL3DPRINTER_START
  } m_state = BL3DPRINTER_START;

  void print_data(const char *fmt) {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    String hex;
    size_t size;
    if (data->type & 0x80) {
      size = data->body_80.size;
    } else {
      size = data->body_00.size;
    }
    for(int i = 0; i < size; i++) {
      uint8_t c2 = ((uint8_t*)data)[i];
      uint8_t c1 = c2 >> 4;
      c2 = c2 & 0x0f;
      hex += String(c1, HEX);
      hex += String(c2, HEX);
    }
    Serial.printf(fmt, hex.c_str());
  }

  uint8_t m_cur_extruder_id = -1;
  uint8_t m_cur_extruder_status = -1;
  uint8_t m_filament_online_status = 0;
  float m_meters = 0.0;
  uint8_t m_packge_num = 0;

  void print_now() {
    struct tm now;
    getLocalTime(&now);
    Serial.print(&now);
  }
  int m_last_meters = 0;

  void on_get_extruder_meters() {
    // 进退料时被频繁调用
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    Cxx_res[1] = 0xC0 | (m_packge_num << 3);

    uint8_t extruder_id = data->body_80.data.extruder.id;
    uint8_t extruder_cmd = data->body_80.data.extruder.status;
    float meters = -1;
    if (extruder_id < 4 && amslite.m_x <= 0) {
      if (extruder_id != m_cur_extruder_id) {
        if (extruder_id == 0) {
          digitalWrite(CD74HC4067_S0_PIN, LOW);
          digitalWrite(CD74HC4067_S1_PIN, LOW);
        } else if (extruder_id == 1) {
          digitalWrite(CD74HC4067_S0_PIN, HIGH);
          digitalWrite(CD74HC4067_S1_PIN, LOW);
        } else if (extruder_id == 2) {
          digitalWrite(CD74HC4067_S0_PIN, LOW);
          digitalWrite(CD74HC4067_S1_PIN, HIGH);
        } else if (extruder_id == 3) {
          digitalWrite(CD74HC4067_S0_PIN, HIGH);
          digitalWrite(CD74HC4067_S1_PIN, HIGH);
        }
        as5600.resetCumulativePosition();
        m_meters = 0;
      }
      if (extruder_cmd == 0x3f) {        // 请求退料
        m_cur_extruder_status = extruder_cmd;
        m_meters = as5600.getCumulativePosition() * PI * EXTRUDER_GEAR_DIAMETER / (1 << 12);
        ams_lite.backward(extruder_id);
      } else if (extruder_cmd == 0xbf) { // 请求进料
        m_cur_extruder_status = extruder_cmd;
        ams_lite.forward(extruder_id);
      } else {
        m_meters = as5600.getCumulativePosition() * PI * EXTRUDER_GEAR_DIAMETER / (1 << 12);
        if (m_cur_extruder_status == 0x3f) {
          amslite.stop(m_meters, 60);
        } else {
          amslite.stop_ex();
        }
      }
      if (extruder_id != m_cur_extruder_id || extruder_cmd != m_cur_extruder_status || abs(m_last_meters - m_meters) > 2.0) {
        print_now(); Serial.printf(" on_get_extruder_meters extruder_id: %d, extruder_cmd: %x meters: %f\n", extruder_id, extruder_cmd, m_meters);
        m_last_meters = m_meters;
      }
      m_cur_extruder_id = extruder_id;
      m_cur_extruder_status = extruder_cmd;
      meters = m_meters;
    }
    Cxx_res[7] = 0x02;
    Cxx_res[8] = extruder_id;
    memcpy(Cxx_res + 9, &meters, sizeof(meters));

    send((bambu_data_t*)Cxx_res);
    // packge_num = (packge_num + 1) % 8;
  }

  int m_last_meters2 = 0;

  void on_get_extruder_status() {
    // 平时频繁调用
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    uint8_t extruder_id = data->body_80.data.extruder_ex.id;
    uint8_t extruder_status = data->body_80.data.extruder_ex.status;
    float meters = -1;
    if (extruder_id < 4) {
      if (extruder_status != 0xbf) {   // 进料中不计数，这是为什么？？？
        m_meters = as5600.getCumulativePosition() * PI * EXTRUDER_GEAR_DIAMETER / (1 << 12);
      }
      if (extruder_id != m_cur_extruder_id || extruder_status != m_cur_extruder_status || abs(m_last_meters2 - m_meters) > 2.0) {
        print_now(); Serial.printf(" on_get_extruder_status extruder_id: %d, extruder_cmd: %x meters: %f\n", extruder_id, extruder_status, m_meters);
        m_last_meters2 = m_meters;
      }
      meters = m_meters;
    }
    Dxx_res[1] = 0xC0 | (m_packge_num << 3);
    Dxx_res[9] = m_filament_online_status;
    Dxx_res[10] = m_filament_online_status;
    Dxx_res[11] = m_filament_online_status;
    Dxx_res[12] = extruder_id;
    Dxx_res[13] = 0;
    Dxx_res[19] = 0x02;
    Dxx_res[20] = extruder_id;
    memcpy(Dxx_res + 21, &meters, sizeof(meters));
    send((bambu_data_t*)Dxx_res);
    // m_packge_num = (m_packge_num + 1) % 8;
  }

  void on_online_detection() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (data->body_80.data.divice_id = 0x0001) {
      uint8_t restuls[0x1d]{0x3d, 0xc0, 0x1d, 0xb4, 0x05, 0x01, 0x00};
      send((bambu_data_t*)restuls);
    }
  }

  void on_set_filament() {
    Serial.printf("on_set_filament\n");
    // data->body_80.data.filament.index 高四位为AMS设备编号
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    uint8_t n = data->body_80.data.filament.index;
    m_filaments[n].index = n;
    memcpy(m_filaments[n].id, data->body_80.data.filament.id, sizeof(m_filaments[n].id));
    memcpy(m_filaments[n].name, data->body_80.data.filament.name, sizeof(m_filaments[n].name));
    m_filaments[n].color = data->body_80.data.filament.color;
    m_filaments[n].temperature_min = data->body_80.data.filament.temperature_min;
    m_filaments[n].temperature_max = data->body_80.data.filament.temperature_max;
    File file = LittleFS.open("/filaments.bin", "wb");
    file.write((uint8_t*)m_filaments, sizeof(m_filaments));
    file.close();
    uint8_t restuls[0x08]{0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60};
    send((bambu_data_t*)restuls);
  }

  void process_body_80() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (m_end < data->body_80.size) {
      return;
    }
    size_t size = data->body_80.size;
    m_crc16.restart();
    m_crc16.add(m_buffer, size - 2);
    int rv = m_crc16.calc();
    if (m_buffer[size - 2] != (rv & 0xFF) || m_buffer[size - 1] != (rv >> 8)) {
      Serial.print("process_body_80: m_buffer[size - 2] != (rv & 0xFF) || m_buffer[size - 1] != (rv >> 8)\n");
      m_buffer[0] = 0;
      return;
    }

    // 正式开始处理数据
    if (data->body_80.cmd == 0x03) {
      on_get_extruder_meters();
    } else if (data->body_80.cmd == 0x04) {
      // 打印机询问我们状态
      on_get_extruder_status();
    } else if (data->body_80.cmd == 0x05) {
      on_online_detection();
    } else if (data->body_80.cmd == 0x07) {
      // 0x07 是 NFC 信号，忽略即可
    } else if (data->body_80.cmd == 0x08) {
      on_set_filament();
    } else if (data->body_80.cmd == 0x20) {
      // 0x20 是心跳信号，忽略即可
    } else {
      print_data("0x80的未知消息：%s\n");
    }
    m_end -= size;
    memcpy(m_buffer, m_buffer + size, m_end);
  }

  void process_head_80() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (m_end < 4) {
      return;
    }
    m_crc8.restart();
    m_crc8.add(m_buffer, 3);
    if (data->body_80.rv != m_crc8.calc()) {
      Serial.print("process_head_80: data->body_80.rv != m_crc8.calc()\n");
      m_buffer[0] = 0;
      return;
    }
    process_body_80();
  }

  // 以下回复信息
  void on_get_hardware_serial_number() {
    // 硬件序列号
    Serial.printf("on_get_hardware_serial_number\n");
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    bambu_data_t *result = (bambu_data_t*)hardware_serial_number_res;
    result->body_00.package_number = data->body_00.package_number;
    send(result);
  }

  void on_get_firmware_version() {
    // 固件版本
    Serial.printf("on_get_firmware_version\n");
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    bambu_data_t *result = (bambu_data_t*)firmware_version_res;
    result->body_00.package_number = data->body_00.package_number;
    send(result);
  }

  void on_get_filament() {
    // 耗材信息
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    bambu_data_t results;
    results.head = 0x3d;
    results.type = 0x00;
    results.body_00.package_number = data->body_00.package_number;
    results.body_00.size = 13 + sizeof(filament_ex_t) + 2;
    results.body_00.target_address = data->body_00.source_address;
    results.body_00.source_address = data->body_00.target_address;
    results.body_00.type = data->body_00.type;
    uint8_t n = data->body_00.data.filament_ex.index;
    Serial.printf("打印机询问我们耗材类型: %d\n", n);
    m_filaments[n].index = n;
    results.body_00.data.filament_ex = m_filaments[n];
    send(&results);
  }

  void process_body_00() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (m_end < data->body_00.size) {
      return;
    }
    size_t size = data->body_00.size;
    m_crc16.restart();
    m_crc16.add(m_buffer, size - 2);
    int rv = m_crc16.calc();
    if (m_buffer[size - 2] != (rv & 0xFF) || m_buffer[size - 1] != (rv >> 8)) {
      Serial.print("process_body_00: m_buffer[size - 2] != (rv & 0xFF) || m_buffer[size - 1] != (rv >> 8)\n");
      m_buffer[0] = 0;
      return;
    }

    // Serial.printf("body_00 package_number: %X, size: %X, target_address: %X, source_address: %X, type: %X \n",  data->body_00.package_number, data->body_00.size, data->body_00.target_address, data->body_00.source_address, data->body_00.type);

    // 我们只处理发给 amslite 的指令
    if (data->body_00.target_address == 0x1200) {
      if (data->body_00.type == 0x0211) {
        on_get_filament();
      } else if (data->body_00.type == 0x0402) {
        on_get_hardware_serial_number();
      } else if (data->body_00.type == 0x0103) {
        on_get_firmware_version();
      }
    } else if(data->body_00.target_address != 0x0e00 && data->body_00.target_address != 0x0f00) {
      // 0x0e00 和 0x0f00 属于未知的指令，其它的属于没见过的指令
      Serial.printf("target_address: %x?\n", data->body_00.target_address);
    }
    m_end -= size;
    memcpy(m_buffer, m_buffer + size, m_end);
  }

  void process_head_00() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (m_end < 7) {
      return;
    }
    m_crc8.restart();
    m_crc8.add(m_buffer, 6);
    if (data->body_00.rv != m_crc8.calc()) {
      Serial.print("process_head_00: data->body_00.rv != m_crc8.calc()\n");
      m_buffer[0] = 0;
      return;
    }
    if (data->body_00.size > 255) {
      Serial.printf("bambu_data->body_00.size: %d\n", data->body_00.size);
      m_buffer[0] = 0;
      return;
    }
    process_body_00();
  }

  void process_header() {
    bambu_data_t *data = (bambu_data_t*)m_buffer;
    if (m_end >= 2) {
      if (data->type == 0xc5) {
        process_head_80();
      } else if (data->type == 0x05){
        process_head_00();
      } else {
        print_data("未知消息：%s\n");
      }
    }
  }

  void head_seek() {
    for (int i = 0; i < m_end; i++) {
      if (m_buffer[i] == 0x3d) {
        if (i > 0) {
          // 过滤掉初次启动时接收的碎片
          Serial.printf("过滤碎片 %d 字节\n", i);
          memcpy(m_buffer, m_buffer + i, m_end - i);
          m_end = m_end - i;
        }
        return process_header();
      }
    }
    m_end = 0;
  }

  void loop() {
    m_end = RS485.read(m_buffer + m_end, sizeof(m_buffer) - m_end) + m_end;
    head_seek();
  }
};

BL3DPrinter s_printer;

void on_post_amslite_status(AsyncWebServerRequest* request) {
  const AsyncWebParameter* param = nullptr;
  s_printer.m_filament_online_status = 0;
  param = request->getParam("A1", true);
  if (param && param->value() == "online") {
    s_printer.m_filament_online_status |= 1;
  }
  param = request->getParam("A2", true);
  if (param && param->value() == "online") {
    s_printer.m_filament_online_status |= 2;
  }
  param = request->getParam("A3", true);
  if (param && param->value() == "online") {
    s_printer.m_filament_online_status |= 4;
  }
  param = request->getParam("A4", true);
  if (param && param->value() == "online") {
    s_printer.m_filament_online_status |= 8;
  }
  request->send(200);
}

void setup() {
  Serial.begin(115200);
  little_fs_setup();
  s_printer.setup();
  // Serial.println(String(ESP.getEfuseMac(), HEX).c_str());
  s_config.setup();
  wifi_setup();
  time_setup();
  // Make it possible to access webserver at http://zhaipro-amslite.local
  const char *hostname = "zhaipro-amslite";
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("Access at http://%s.local\n", hostname);
  }

  as5600_setup();
#ifndef __DEBUG__
  bambu_setup();
#endif
  wifi_server_setup();
  ams_lite.setup(MOTOR0_PIN1, MOTOR0_PIN2, MOTOR1_PIN1, MOTOR1_PIN2, SERVO0_PIN, 14);
  ams_lite.m_servo0_init = s_config.m_data["servo0_init"];
  ams_lite.m_servo1_init = s_config.m_data["servo1_init"];
  ams_lite.m_servo_power = s_config.m_data["servo_power"];
  ams_lite.stop();
}

void loop() {
  amslite.loop();
  ElegantOTA.loop();
  s_printer.loop();
#ifndef __DEBUG__
  if (WiFi.status() == WL_CONNECTED && !bambu_client.connected()) {
    if (s_config.m_data["mode"] == "WAN") {
      const char* bambu_mqtt_id = "mqttx_c59bbf21";
      const char* username = s_config.m_data["username"];
      const char* access_token = s_config.m_data["access_token"];
      if (username && access_token) {
        Serial.printf("bambu_client.connect(<id>, \"%s\", <token>)\n", username);
        bambu_client.setServer("cn.mqtt.bambulab.com", 8883);
        if (bambu_client.connect(bambu_mqtt_id, username, access_token)) {
          Serial.println("Connecting to bambu .. connected!");
          bambu_client.subscribe(s_config.m_data["bambu_topic_subscribe"]);
          bambu_client.publish(s_config.m_data["bambu_topic_publish"], bambu_pushall);
        } else {
          Serial.printf("The bambu connection(WAN) failed!\nbambu_client.state() => %d\n", bambu_client.state());
          s_config.m_data.remove("username");
          s_config.m_data.remove("access_token");
        }
      } else {
        const String& phone_number = s_config.m_data["phone_number"].as<String>();
        const String& password = s_config.m_data["password"].as<String>();
        if (!phone_number.isEmpty() && !password.isEmpty()) {
          HTTPClient http;
          JsonDocument data;
          char buffer[256];
          http.begin("https://api.bambulab.cn/v1/user-service/user/login");
          http.addHeader("Content-Type", "application/json");
          data["account"] = phone_number;
          data["password"] = password;
          serializeJson(data, buffer, 256);
          Serial.printf("[HTTP] POST: %s\n", buffer);
          int code = http.POST(buffer);
          if (code == HTTP_CODE_OK) {
            deserializeJson(data, http.getString());
            s_config.m_data["access_token"] = data["accessToken"].as<String>();
            const char* jwt = data["accessToken"].as<const char*>();
            const char* sep = ".";
            strtok((char*)jwt, sep);
            char* encoded_payload = strtok(NULL, sep);
            uint8_t payload[base64::decodeLength(encoded_payload)];
            base64::decode(encoded_payload, payload);
            deserializeJson(data, payload);
            s_config.m_data["username"] = data["username"].as<String>();
            Serial.printf("username: %s\n", data["username"].as<const char*>());
            s_config.save();
          } else {
            Serial.printf("[HTTP] GET... code: %d\n%s", code, http.getString().c_str());
            s_config.m_data["mode"] = "";
          }
        }
      }
    } else if (s_config.m_data["mode"] == "LAN") {
      const char* bambu_mqtt_id = "mqttx_c59bbf21";
      const char* bambu_mqtt_user = "bblp";
      bambu_client.setServer(s_config.m_data["bambu_mqtt_broker"].as<const char*>(), 8883);
      if (bambu_client.connect(bambu_mqtt_id, bambu_mqtt_user, s_config.m_data["bambu_mqtt_password"].as<const char*>())) {
        Serial.println("Connecting to bambu .. connected!");
        bambu_client.subscribe(s_config.m_data["bambu_topic_subscribe"].as<const char*>());
        bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_pushall);
      } else {
        ws.printfAll("{\"message\": \"The bambu connection(LAN) failed! state: %d\"}", bambu_client.state());
        Serial.printf("The bambu connection(LAN) failed! state: %d\n", bambu_client.state());
        s_config.m_data["mode"] = "";
      }
    }
  }
  bambu_client.loop();
#endif
}
