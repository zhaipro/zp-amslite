 
#include <WiFi.h>
#include "setups.h"

void time_setup() {
  // 获取时间
  configTime(28800, 0, "pool.ntp.org");
  while (!time(nullptr)) {
    delay(1000);
    Serial.println("Waiting for time sync...");
  }
  Serial.print("Time synced successfully, ");
  struct tm now;
  getLocalTime(&now);
  Serial.println(&now);
}
