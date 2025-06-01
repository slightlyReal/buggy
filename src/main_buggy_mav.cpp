// platformio run --target  upload

/*
1. Best part is no part
2. Question requirement and  make them less dump. Don't follow any rule that doesn't make sense
3. Delete any part of design process that isnt necessary
4. Optimize
5. Accelerate
6. Automate
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Define MAVLink Communication Protocol
#ifdef F
#undef F
#endif
#include <common/mavlink.h>

// For esp_timer_get_time()
#include "esp_timer.h"

#include "conf.h"
#include "wifi_pass_config.h"
#include "func.h"

// Add Logger
// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
static const char *LOGGER_TAG = "main_sampleUDP";

#include <math.h>

void setup()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);

  Serial.begin(MAVLINK_BAUD);

  init_wifi();
}

int64_t elapsed_us = 0;
void loop()
{
  unsigned long currentMillis = millis();
  int64_t start_us = 0;
  int64_t end_us = 0;

  float sine_value = 0.0f;
  float cosine_value = 0.0f;
  if (currentMillis - previousHeartbeatMillis >= heartbeat_interval)
  {
    previousHeartbeatMillis = currentMillis;

    start_us = esp_timer_get_time(); // Time before transmission
    send_heartbeat();
    send_systemstatus();
    send_radiostatus();
    send_position();

    sine_value = sin(currentMillis);
    cosine_value = cos(currentMillis);

    send_mavlink_attitude(currentMillis, (cosine_value * 10.0f * M_PI / 180.0f), (cosine_value * 20.0f * M_PI / 180.0f), (cosine_value * 30.0f * M_PI / 180.0f));
    send_mavlink_SCALED_IMU(currentMillis, (int16_t)(cosine_value * 10.0f), (int16_t)(cosine_value * 10.0f), (int16_t)(cosine_value * 10.0f));
    send_mavlink_local_position_ned(currentMillis, 1, 2, 3);
    send_mavlink_msg_rc_channels(static_cast<uint16_t>(elapsed_us));
    end_us = esp_timer_get_time(); // Time after transmission

    elapsed_us = end_us - start_us;
    ESP_LOGI(LOGGER_TAG, "UDP transmission took %lld microseconds", elapsed_us); // Warning level

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.print("\nReconnecting to ");
      Serial.println(WIFI_SSID);

      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);

      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  if (currentMillis - previousTimeoutMillis >= timeout_interval)
  {
    previousTimeoutMillis = millis();

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }

  while (Serial.available() > 0)
  {
    uint8_t serial_byte = Serial.read();
    parse_mavlink(serial_byte);
  }

  int packetSize = mavlink_udp.parsePacket();
  if (packetSize)
  {
    uint8_t packetBuffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_udp.read(packetBuffer, MAVLINK_MAX_PACKET_LEN);

    for (int16_t i = 0; i < packetSize; i++)
    {
      parse_mavlink(packetBuffer[i]);
    }
  }
}
