
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <arduino_base64.hpp>

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

void homepage(AsyncWebServerRequest* request) {
  const char *html = "<!DOCTYPE html>\n\
<html>\n\
<meta name='viewport' content='width=device-width;text/html;charset=utf-8' http-equiv='Content-Type' />\n\
<style>\n\
  div.hidden {display:none;}\n\
</style>\n\
<style>\n\
  .tooltip {\n\
    position: relative;\n\
    display: inline-block;\n\
    border-bottom: 1px dotted black;\n\
  }\n\
  \n\
  .tooltip .tooltiptext {\n\
    visibility: hidden;\n\
    width: 256px;\n\
    background-color: black;\n\
    color: #fff;\n\
    /* text-align: center; */\n\
    border-radius: 6px;\n\
    padding: 5px 10px;\n\
\n\
    /* 定位 */\n\
    position: absolute;\n\
    z-index: 1;\n\
  }\n\
  \n\
  .tooltip:hover .tooltiptext {\n\
    visibility: visible;\n\
  }\n\
</style>\n\
<script>\n\
  function do_fetch(pathname, params=null, kws=null) {\n\
    var origin = window.location.origin;\n\
    if (origin.startsWith('file')) {\n\
      return;\n\
    }\n\
    var url = new URL(pathname, origin);\n\
    if (params == null) {\n\
      params = [];\n\
    }\n\
    for (let i in params) {\n\
      var k = params[i];\n\
      var e = document.getElementById(k);\n\
      if (e == null) {\n\
        e = document.getElementsByName(k)[0];\n\
      }\n\
      var v = e.value;\n\
      url.searchParams.append(k, v);\n\
    }\n\
    if (kws == null) {\n\
      kws = {}\n\
    }\n\
    for (let k in kws) {\n\
      url.searchParams.append(k, kws[k]);\n\
    }\n\
    return fetch(url);\n\
  }\n\
\n\
  function unload() {\n\
    do_fetch('/unload', ['previous_extruder'])\n\
  }\n\
  function load() {\n\
    do_fetch('/load', ['next_extruder'])\n\
  }\n\
  function stop() {\n\
    do_fetch('/stop', ['servo1_init', 'servo_power', 'previous_extruder', 'next_extruder'])\n\
  }\n\
  function test_forward() {\n\
    do_fetch('/test_forward', ['next_extruder'])\n\
  }\n\
  function test_backward() {\n\
    do_fetch('/test_backward', ['previous_extruder'])\n\
  }\n\
  function get_config() {\n\
    do_fetch('/get_config')\n\
      .then((response) => response.json())\n\
      .then((data) => {\n\
        for (let k in data) {\n\
          var e = document.getElementsByName(k)[0];\n\
          if (e) {\n\
            e.value = data[k];\n\
          }\n\
        }\n\
      });\n\
  }\n\
  function wifi_begin() {\n\
    do_fetch('/wifi_begin', ['WiFi_ssid', 'WiFi_passphrase'])\n\
  }\n\
  function get_local_ip() {\n\
    do_fetch('/get_local_ip')\n\
      .then((response) => response.json())\n\
      .then((data) => {\n\
        var e = document.getElementById('local_ip');\n\
        e.href = 'http://' + data['local_ip'];\n\
        e.text = data['local_ip'];\n\
      })\n\
  }\n\
  function on_load() {\n\
    get_config();\n\
  }\n\
  window.addEventListener('load', on_load);\n\
</script>\n\
<script>\n\
  function mode_change() {\n\
    var mode = document.getElementsByName('mode')[0].value;\n\
    if (mode == 'WAN') {\n\
      document.getElementById('LAN').setAttribute('class', 'hidden');\n\
      document.getElementById('WAN').setAttribute('class', '');\n\
    } else if (mode == 'LAN'){\n\
      document.getElementById('LAN').setAttribute('class', '');\n\
      document.getElementById('WAN').setAttribute('class', 'hidden');\n\
    }\n\
  }\n\
</script>\n\
\n\
<div style='width:400px;float:left;'>\n\
\n\
<button onmouseup=get_config()>0. 获取配置信息</button> <br>\n\
WiFi名称：<input name='WiFi_ssid'> <br>\n\
WiFi密码：<input name='WiFi_passphrase'> <br>\n\
<button onmouseup=wifi_begin()>1. 连接 WiFi</button> <br>\n\
<button onmouseup=get_local_ip()>2. 获取 ip</button> <br>\n\
3. 跳转到：<a id='local_ip'></a> <br>\n\
--------------------------------------------------\n\
<form action='/put_config' target='stop'>\n\
4. 请选择联机模式: \n\
<select name='mode' onchange=mode_change()>\n\
  <option value='WAN'>广域网模式</option>\n\
  <option value='LAN'>局域网模式</option>\n\
</select>\n\
<br>\n\
<div id='LAN' class='hidden'>\n\
  打印机ip地址：<input name='bambu_mqtt_broker'> <br>\n\
  打印机访问码：<input name='bambu_mqtt_password'> <br>\n\
</div>\n\
<div id='WAN'>\n\
  手机号码：<input name='phone_number'> <br>\n\
  密码：<input name='password'> <br>\n\
</div>\n\
打印机序列号：<input name='bambu_device_serial'> <br>\n\
servo1_init: <input type='number' name='servo1_init' value=90> <br>\n\
servo_power: <input type='number' name='servo_power' value=30> <br>\n\
有待退料管道：<input type='number' name='previous_extruder' value=0> <br>\n\
有待进料管道：<input type='number' name='next_extruder' value=0> <br>\n\
<input type='submit' value='5. 上传配置信息'>\n\
</form>\n\
<iframe  name='stop' style='display:none;'></iframe>\n\
<button onmouseup=unload()>unload</button>\n\
<button onmouseup=load()>load</button>\n\
<button onmouseup=stop()>stop</button> <br>\n\
<button onmouseup=fetch('/resume')>resume</button>\n\
<button onmouseup=fetch('/gcode_m109')>gcode_m109</button> <br>\n\
<button onmouseup=test_forward()>test_forward</button>\n\
<button onmouseup=test_backward()>test_backward</button>\n\
</div>\n\
\n\
<div style='float:left;'>\n\
  <table>\n\
    <tr>\n\
      <td>sequence_id:</td>\n\
      <td id='sequence_id'>unknown</td>\n\
    </tr>\n\
    <tr>\n\
      <td>hw_switch_state\n\
        <div class='tooltip'>❔\n\
          <span class='tooltiptext'>\n\
            料线检测<br>\n\
            1：正常<br>\n\
            0：无料\n\
          </span>\n\
        </div>\n\
        :\n\
      </td>\n\
      <td id='hw_switch_state'>unknown</td>\n\
    </tr>\n\
    <tr>\n\
      <td>ams_status\n\
        <div class='tooltip'>❔\n\
          <span class='tooltiptext'>\n\
            258：加热喷嘴<br>\n\
            259：剪断耗材丝<br>\n\
            260:退料最后提示回抽<br>\n\
            0：回抽完成<br>\n\
            261：提示插入线材<br>\n\
            262：插入成功，提示观察喷嘴<br>\n\
            263：确认之后挤出材料并冲刷<br>\n\
            768：进料完成<br>\n\
            1280：正在打印（不确定）\n\
          </span>\n\
        </div>:\n\
      </td>\n\
      <td id='ams_status'>unknown</td>\n\
    </tr>\n\
    <tr>\n\
      <td>print_error: </td>\n\
      <td id='print_error'>unknown</td>\n\
    </tr>\n\
    <tr>\n\
      <td>gcode_state: </td>\n\
      <td id='gcode_state'>unknown</td>\n\
    </tr>\n\
    <tr>\n\
      <td>mc_percent: </td>\n\
      <td id='mc_percent'>unknown</td>\n\
    </tr>\n\
  </table>\n\
</div>\n\
<script>\n\
  var gateway = `ws://${window.location.hostname}/ws`;\n\
  var websocket;\n\
  function initWebSocket() {\n\
    console.log('Trying to open a WebSocket connection...');\n\
    websocket = new WebSocket(gateway);\n\
    websocket.onopen    = onOpen;\n\
    websocket.onclose   = onClose;\n\
    websocket.onmessage = onMessage;\n\
  }\n\
  function onOpen(event) {\n\
    console.log('Connection opened');\n\
  }\n\
  function onClose(event) {\n\
    console.log('Connection closed');\n\
  }\n\
  function onMessage(event) {\n\
    console.log('On message:');\n\
    console.log(event.data);\n\
    const data = JSON.parse(event.data);\n\
    for (let k in data) {\n\
      document.getElementById(k).innerText = data[k];\n\
    }\n\
  }\n\
\n\
  window.addEventListener('load', onLoad);\n\
\n\
  function onLoad(event) {\n\
    initWebSocket();\n\
  }\n\
</script>\n\
</html>";

  request->send(200, "text/html", html);
}

double get_arg(AsyncWebServerRequest *request, const char* name, double default_value = 0.0) {
  if (request->hasParam(name)) {
    return request->getParam(name)->value().toDouble();
  }
  return default_value;
}

class Config {
public:
  JsonDocument m_data;
  void setup() {
    if (LittleFS.exists("/config.json")) {
      File file = LittleFS.open("/config.json", "r");
      deserializeJson(m_data, file);
      file.close();
    }
    if (!m_data.containsKey("mode")) {
      m_data["mode"] = "";
    }
    if (!m_data.containsKey("bambu_mqtt_broker")) {
      m_data["bambu_mqtt_broker"] = "";
    }
    if (!m_data.containsKey("bambu_mqtt_password")) {
      m_data["bambu_mqtt_password"] = "";
    }
    if (!m_data.containsKey("bambu_device_serial")) {
      m_data["bambu_device_serial"] = "";
    }
    if (!m_data.containsKey("servo_power")) {
      m_data["servo_power"] = 30;
    }
    if (!m_data.containsKey("servo1_init")) {
      m_data["servo1_init"] = 90;
    }
    if (!m_data.containsKey("WiFi_ssid")) {
      m_data["WiFi_ssid"] = "";
    }
    if (!m_data.containsKey("WiFi_passphrase")) {
      m_data["WiFi_passphrase"] = "";
    }
    if (!m_data.containsKey("previous_extruder")) {
      m_data["previous_extruder"] = 0;
    }
    if (!m_data.containsKey("next_extruder")) {
      m_data["next_extruder"] = 0;
    }
  }
  void save() {
    File file = LittleFS.open("/config.json", "w");
    serializeJson(m_data, file);
    file.close();
  }
};

Config s_config;

void get_config(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  serializeJson(s_config.m_data, *response);
  if (bambu_client.connected()) {
    bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_pushall);
  }
  request->send(response);
}

void put_config(AsyncWebServerRequest *request) {
  AsyncWebParameter* param = nullptr;
  param = request->getParam("mode");
  if (param) {
    s_config.m_data["mode"] = param->value();
  }
  param = request->getParam("phone_number");
  if (param) {
    s_config.m_data["phone_number"] = param->value();
  }
  if (request->hasParam("password")) {
    s_config.m_data["password"] = request->getParam("password")->value();
  }
  if (request->hasParam("bambu_mqtt_broker")) {
    s_config.m_data["bambu_mqtt_broker"] = request->getParam("bambu_mqtt_broker")->value();
  }
  if (request->hasParam("bambu_mqtt_password")) {
    s_config.m_data["bambu_mqtt_password"] = request->getParam("bambu_mqtt_password")->value();
  }
  if (request->hasParam("bambu_device_serial")) {
    const String& bambu_device_serial = request->getParam("bambu_device_serial")->value();
    s_config.m_data["bambu_device_serial"] = bambu_device_serial;
    s_config.m_data["bambu_topic_subscribe"] = "device/" + bambu_device_serial + "/report";
    s_config.m_data["bambu_topic_publish"] = "device/" + bambu_device_serial + "/request";
  }
  if (request->hasParam("servo1_init")) {
    ams_lite1.m_servo_init = request->getParam("servo1_init")->value().toInt();
    s_config.m_data["servo1_init"] = ams_lite1.m_servo_init;
  }
  if (request->hasParam("servo_power")) {
    ams_lite1.m_servo_power = request->getParam("servo_power")->value().toInt();
    s_config.m_data["servo_power"] = ams_lite1.m_servo_power;
  }

  if (request->hasParam("previous_extruder")) {
    s_config.m_data["previous_extruder"] = request->getParam("previous_extruder")->value().toInt();
  }
  if (request->hasParam("next_extruder")) {
    s_config.m_data["next_extruder"] = request->getParam("next_extruder")->value().toInt();
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

// 在屏幕上打印些信息
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void oled_update() {
  // 在OLED屏显示本机IP地址
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(WiFi.localIP());
  display.printf("zp_state: %d\n", zp_state);
  display.printf("previous_extruder: %d\n", previous_extruder);
  display.printf("next_extruder: %d\n", next_extruder);

  display.display();      // Show initial text
}

void display_setup() {
  // 初始化显示屏
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }

  // 在OLED屏显示本机IP地址
  oled_update();
}

void unload(AsyncWebServerRequest* request) {
  previous_extruder = get_arg(request, "previous_extruder", 0);
  bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_unload);
  request->send(200);
}

void load(AsyncWebServerRequest* request) {
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
  next_extruder = get_arg(request, "next_extruder", 0);
  ams_lite1.forward(next_extruder);
  previous_extruder = next_extruder;
  request->send(200);
}

void test_backward(AsyncWebServerRequest* request) {
  previous_extruder = get_arg(request, "previous_extruder", 0);
  ams_lite1.backward(previous_extruder);
  next_extruder = previous_extruder;
  request->send(200);
}

void wifi_setup() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("zhaipro-amslite", "zhaipro-amslite");
  if (!s_config.m_data.containsKey("WiFi_ssid") || !s_config.m_data.containsKey("WiFi_passphrase")) {
    return;
  }
  const String& ssid = s_config.m_data["WiFi_ssid"].as<String>();
  const String& passphrase = s_config.m_data["WiFi_passphrase"].as<String>();
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
    Serial.println(" failed");
  }
}

void wifi_begin(AsyncWebServerRequest* request) {
  const String& ssid = request->getParam("WiFi_ssid")->value();
  const String& passphrase = request->getParam("WiFi_passphrase")->value();
  s_config.m_data["WiFi_ssid"] = ssid;
  s_config.m_data["WiFi_passphrase"] = passphrase;
  s_config.save();
  WiFi.begin(ssid, passphrase);
  request->send(200);
}

void bambu_callback(char* topic, byte* payload, unsigned int length) {
  // https://arduinojson.org/v7/api/jsondocument/
  JsonDocument data;
  deserializeJson(data, payload, length);
  if (!data.containsKey("print")) {
    // 收到未知信息，直接不理睬
    return;
  }

  JsonDocument _data;

  const char* sequence_id = data["print"]["sequence_id"].as<const char*>();
  if (data["print"].containsKey("hw_switch_state")) {
    hw_switch_state = data["print"]["hw_switch_state"].as<int>();
    _data["hw_switch_state"] = hw_switch_state;
  }
  if (data["print"].containsKey("gcode_state")) {
    gcode_state = data["print"]["gcode_state"].as<String>();
    _data["gcode_state"] = gcode_state;
  }
  if (data["print"].containsKey("mc_percent")) {
    mc_percent = data["print"]["mc_percent"].as<int>();
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
  if (data["print"].containsKey("ams_status")) {
    ams_status = data["print"]["ams_status"].as<int>();
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
  
  if (data["print"].containsKey("print_error")) {
    print_error = data["print"]["print_error"].as<int>();
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
  oled_update();
}

void bambu_setup() {
  // https://pubsubclient.knolleary.net/
  wifi_client.setInsecure();
  bambu_client.setCallback(bambu_callback);
  bambu_client.setBufferSize(4096);   // 其默认值 256 太小啦
}

void wifi_server_setup() {
  server.on("/", homepage);
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
  server.on("/wifi_begin", wifi_begin);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");
}

void setup() {
  Serial.begin(115200);
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
  display_setup();
#ifndef __DEBUG__
  bambu_setup();
#endif
  wifi_server_setup();
  ams_lite1.setup(12, 13, 27, 26, 14);
  ams_lite1.m_servo_init = s_config.m_data["servo1_init"].as<int>();
  ams_lite1.m_servo_power = s_config.m_data["servo_power"].as<int>();
}

void loop() {
#ifndef __DEBUG__
  if (WiFi.status() == WL_CONNECTED && !bambu_client.connected()) {
    if (s_config.m_data["mode"] == "WAN") {
      const char* bambu_mqtt_id = "mqttx_c59bbf21";
      const char* username = s_config.m_data["username"].as<const char*>();
      const char* access_token = s_config.m_data["access_token"].as<const char*>();
      if (username && access_token) {
        Serial.printf("bambu_client.connect(<id>, \"%s\", <token>)\n", username);
        bambu_client.setServer("cn.mqtt.bambulab.com", 8883);
        if (bambu_client.connect(bambu_mqtt_id, username, access_token)) {
          Serial.println("Connecting to bambu .. connected!");
          bambu_client.subscribe(s_config.m_data["bambu_topic_subscribe"].as<const char*>());
          bambu_client.publish(s_config.m_data["bambu_topic_publish"].as<const char*>(), bambu_pushall);
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
        Serial.printf("The bambu connection(LAN) failed!\nbambu_client.state() => %d\n", bambu_client.state());
        s_config.m_data["mode"] = "";
      }
    }
  }
  bambu_client.loop();
#endif
}
