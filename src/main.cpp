
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <arduino_base64.hpp>
#include <CRC16.h>
#include <CRC8.h>

#include "setups.h"

// 开启调试模式，esp32 将不会连接拓竹
// #define __DEBUG__

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
      return m_data["bambu_topic_publish"].as<T>();
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
  void forward() {
    digitalWrite(m_pin1, HIGH);
    digitalWrite(m_pin2, LOW);
  }
  void backward() {
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
  Servo m_servo;
  int m_servo_init = 90;
  int m_servo_power = 30;

  void setup(int m0pin1, int m0pin2, int m1pin1, int m1pin2, int s1pin1) {
    m_motor0.setup(m0pin1, m0pin2);
    m_motor1.setup(m1pin1, m1pin2);
    m_servo.attach(s1pin1);
    m_servo_init = s_config.get("servo1_init", 90);
    m_servo_power = s_config.get("servo_power", 30);
  }

  void forward(int id) {
    if (id == 0) {
      m_servo.write(m_servo_init - m_servo_power);
      m_motor0.forward();
    } else if (id == 1) {
      m_servo.write(m_servo_init + m_servo_power);
      m_motor1.forward();
    }
  }

  void backward(int id) {
    if (id == 0) {
      m_motor0.backward();
      m_servo.write(m_servo_init - m_servo_power);
    } else if (id == 1) {
      m_motor1.backward();
      m_servo.write(m_servo_init + m_servo_power);
    }
  }
  void stop() {
    m_motor0.stop();
    m_motor1.stop();
    m_servo.write(m_servo_init);
  }
};

AMSLite ams_lite1;

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
  AsyncWebParameter* param = nullptr;
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
  param = request->getParam("servo1_init");
  if (param) {
    s_config.m_data["servo1_init"] = param->value().toInt();
    ams_lite1.m_servo_init = param->value().toInt();
  }
  param = request->getParam("servo_power");
  if (param) {
    s_config.m_data["servo_power"] = param->value().toInt();
    ams_lite1.m_servo_power = param->value().toInt();
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
  ams_lite1.stop();
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
  // FINISH
  if (gcode_state != "FINISH") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  next_extruder = get_arg(request, "next_extruder", 0);
  ams_lite1.forward(next_extruder);
  previous_extruder = next_extruder;
  request->send(200);
}

void test_backward(AsyncWebServerRequest* request) {
  if (gcode_state != "FINISH") {
    request->send(400, "text", "当前非暂停状态，不可操控！");
    return;
  }
  previous_extruder = get_arg(request, "previous_extruder", 0);
  ams_lite1.backward(previous_extruder);
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
      ams_lite1.backward(previous_extruder);
    } if (ams_status == 261) {
      // 请推入
      ams_lite1.forward(next_extruder);
    } else if (ams_status == 262) {
      // 推入完成
      ams_lite1.stop();
    } else if (ams_status == 768) {
      // 完成换料
      previous_extruder = next_extruder;
      if (zp_state == 1) {
        bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_resume);
      }
    } else if (ams_status == 0) {
      // 完成退料，但还要继续拔出一段
      delay(1000);
      ams_lite1.stop();
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

void wifi_server_setup() {
  server.rewrite("/", "/index.html");
  server.serveStatic("/", LittleFS, "/");
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
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");
}

CRC16 crc16(0x1021, 0x913D, 0, false, false);
CRC8 crc8(0x39, 0x66, 0, false, false);

#define RS485 Serial1
#define RS485_RTS_PIN 4

void setup() {
  Serial.begin(115200);
  RS485.begin(1228800, SERIAL_8E1, 16, 17);
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
  // Serial.println(String(ESP.getEfuseMac(), HEX).c_str());
  // https://randomnerdtutorials.com/esp32-write-data-littlefs-arduino/
  //  You only need to format LittleFS the first time you run a
  //  test or else use the LITTLEFS plugin to create a partition 
  //  https://github.com/lorol/arduino-esp32littlefs-plugin
  #define FORMAT_LITTLEFS_IF_FAILED true
  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
    Serial.println("LittleFS Mount Failed");
  }
  s_config.setup();
  wifi_setup();
#ifndef __DEBUG__
  bambu_setup();
#endif
  wifi_server_setup();
  ams_lite1.setup(12, 13, 27, 26, 14);
  ams_lite1.m_servo_init = s_config.m_data["servo1_init"];
  ams_lite1.m_servo_power = s_config.m_data["servo_power"];
}

typedef struct {
  uint8_t head;         // 帧头 0x3D
  uint8_t type;
  union {
    struct {            // type: 0x80
      uint8_t size;
      uint8_t rv;       // crc8
      uint8_t cmd;
      uint8_t data[1];
    } body_80;
    struct {            // type: 0x00
      uint8_t temp2;
      uint8_t temp3;
      uint8_t size;
      uint8_t temp5;
      uint8_t rv;       // crc8
      uint8_t cmd;
      uint8_t data[1];
    } body_00;
  };
} bambu_data_t;

#include <assert.h>

static_assert(sizeof(bambu_data_t) == 9, "");

unsigned char F10_res[] = {0x3D, 0xC0, 0x1D, 0xB4, 0x05,
                           0x01, 0x00, 0x20, 0x9B,
                           0x31, 0x33, 0x34, 0x36,
                           0x35, 0x02, 0x00, 0x37,
                           0x39, 0x33, 0x38,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x00, 0x00, 0x00,
                           0x00, 0x00};

void loop() {
  if (Serial.available()) {
    String s = Serial.readString();
    s.replace("\n", "");
    ws.printfAll("{\"message\": \"%s\"}", s.c_str());
  }
  static uint8_t buffer[256];
  static uint8_t buffer2[256];
  static size_t end = 0;
  if (RS485.available()) {
    end = RS485.readBytes(buffer + end, 256 - end) + end;
    int i = 0;
    for (; i < end; i++) {
      if (buffer[i] == 0x3d) {
        break;
      }
    }
    if (i == end) {
      end = 0;
    } else if (i > 0) {
      memcpy(buffer, buffer + i, end - i);
      end = end - i;
    }
    bambu_data_t *bambu_data = (bambu_data_t*)buffer;
    if (end >= 5) {
      if (bambu_data->type & 0x80) {
        if (end >= bambu_data->body_80.size) {
          // 0x20 是心跳信号，可以忽略啦
          if (bambu_data->body_80.cmd != 0x20) {
            String string_to_hex;
            for(int i = 0; i < bambu_data->body_80.size; i++) {
              uint8_t c2 = ((uint8_t*)bambu_data)[i];
              uint8_t c1 = c2 >> 4;
              c2 = c2 & 0x0f;
              string_to_hex += String(c1, HEX);
              string_to_hex += String(c2, HEX);
            }
            ws.printfAll("{\"ams\": \"%s\"}", string_to_hex.c_str());
          }
          memcpy(buffer2, buffer, bambu_data->body_80.size);
          bambu_data = (bambu_data_t*)buffer2;
          end = end - bambu_data->body_80.size;
          memcpy(buffer, buffer + bambu_data->body_80.size, end);
          if (bambu_data->type == 0xc5 && bambu_data->body_80.cmd == 0x05 && bambu_data->body_80.data[0] == 0x01 && bambu_data->body_80.data[1] == 0x00) {
            bambu_data->type = 0xc0;
            bambu_data->body_80.size = 0x1d;
            crc8.restart();
            crc8.add((uint8_t*)bambu_data, 3);
            bambu_data->body_80.rv = crc8.calc();
            crc16.restart();
            crc16.add((uint8_t*)bambu_data, bambu_data->body_80.size - 2);
            int num = crc16.calc();
            int size = bambu_data->body_80.size;
            ((uint8_t*)bambu_data)[size - 2] = num & 0xFF;
            ((uint8_t*)bambu_data)[size - 1] = num >> 8;
            
            // delayMicroseconds(100);
            RS485.write((uint8_t*)bambu_data, size);
            /*
            crc16.restart();
            int size = 0x1D;
            crc16.add((uint8_t*)F10_res, size - 2);
            int num = crc16.calc();
            ((uint8_t*)F10_res)[size - 2] = num & 0xFF;
            ((uint8_t*)F10_res)[size - 1] = num >> 8;
            Serial1.write((uint8_t*)F10_res, size);
            // Serial1.write((uint8_t*)bambu_data, size);
            // Serial1.flush(true);
            */

            String string_to_hex;
            for(int i = 0; i < bambu_data->body_80.size; i++) {
            // for(int i = 0; i < ((bambu_data_t*)F10_res)->body_80.size; i++) {
              uint8_t c2 = ((uint8_t*)bambu_data)[i];
              // uint8_t c2 = ((uint8_t*)F10_res)[i];
              uint8_t c1 = c2 >> 4;
              c2 = c2 & 0x0f;
              string_to_hex += String(c1, HEX);
              string_to_hex += String(c2, HEX);
            }
            ws.printfAll("{\"ams\": \"=> %s\"}", string_to_hex.c_str());
          }
        }
      } else {
        if (end >= bambu_data->body_00.size) {
          String string_to_hex;
          for(int i = 0; i < bambu_data->body_00.size; i++) {
            uint8_t c2 = ((uint8_t*)bambu_data)[i];
            uint8_t c1 = c2 >> 4;
            c2 = c2 & 0x0f;
            string_to_hex += String(c1, HEX);
            string_to_hex += String(c2, HEX);
          }
          if (bambu_data->body_00.data[0] == 0x12) {
            ws.printfAll("{\"ams\": \"!!! %s\"}", string_to_hex.c_str());
          } else {
            ws.printfAll("{\"ams\": \"%s\"}", string_to_hex.c_str());
          }
          end = end - bambu_data->body_00.size;
          memcpy(buffer, buffer + bambu_data->body_00.size, end);
        }
      }
    }
  }
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
