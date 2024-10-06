 
#include "setups.h"
#include <LittleFS.h>

void time_setup() {
  // 获取时间
  configTime(8 * 60 * 60, 0, "pool.ntp.org");
  struct tm now;
  if (getLocalTime(&now)) {
    Serial.print("Time synced successfully, ");
    Serial.println(&now);
  } else {
    Serial.println("Time synced failure!");
  }
}

void little_fs_setup() {
  // https://randomnerdtutorials.com/esp32-write-data-littlefs-arduino/
  //  You only need to format LittleFS the first time you run a
  //  test or else use the LITTLEFS plugin to create a partition
  //  https://github.com/lorol/arduino-esp32littlefs-plugin
  #define FORMAT_LITTLEFS_IF_FAILED true
  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
    Serial.println("LittleFS Mount Failed");
  }
}
