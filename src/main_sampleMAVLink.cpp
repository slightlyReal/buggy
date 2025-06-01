// To Upload and run : pio run -t upload -e [env_defined_in_platformio.ini] in our case :
// pio run -t upload -e sampleMAVLink

#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_pass_config.h"
#include "esp_timer.h" // For esp_timer_get_time()

// Add Logger
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
static const char *LOGGER_TAG = "main_sampleMavlink";

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

// Static IP configuration
// Set your desired static IP address (choose one outside the DHCP range, but within the same subnet)
IPAddress local_IP(192, 168, 137, 5); // ESP32 static IP
IPAddress gateway(192, 168, 137, 1);  // Gateway is usually your laptop's hotspot IP
IPAddress subnet(255, 255, 255, 0);   // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);     // Optional
IPAddress secondaryDNS(8, 8, 4, 4);   // Optional

// UDP configuration
const char *udpRemoteIp = "192.168.137.255"; // Broadcast UDP packets from ESP32
const int udpRemotePort = 14550;             // Target port
const int udpLocalPort = 1234;               // Local port to listen for UDP

WiFiUDP udp;

// UDP Sender Task
void udpSenderTask(void *pvParameters)
{
  char payload[900];
  unsigned int counter = 0;
  int64_t start_us = 0;
  int64_t end_us = 0;
  int64_t elapsed_us = 0;
  int16_t packet_length = 0;

  const TickType_t xSampleRate = pdMS_TO_TICKS(20); // 1 ms = 1 kHz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // HearthBeat
  uint8_t heartbeat[21] = {253, 9, 0, 0, 136, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 30, 13};
  while (1)
  {
    start_us = esp_timer_get_time(); // Time before transmission

    memcpy(payload, heartbeat, sizeof(heartbeat));
    packet_length = sizeof(heartbeat);
    udp.beginPacket(udpRemoteIp, udpRemotePort);
    udp.write((uint8_t *)payload, packet_length);
    udp.endPacket();

    end_us = esp_timer_get_time(); // Time after transmission

    elapsed_us = end_us - start_us;

    ESP_LOGI(LOGGER_TAG, "UDP transmission took %lld microseconds", elapsed_us); // Warning level

    start_us = esp_timer_get_time(); // Time before transmission
    Serial.println((char*)heartbeat);
    end_us = esp_timer_get_time(); // Time after transmission
    elapsed_us = end_us - start_us;

    ESP_LOGD(LOGGER_TAG, "Serial transmission took %lld microseconds", elapsed_us); // Warning level
    // Serial.printf("[Sender] Sent: %s\n", payload);
    // vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
    vTaskDelayUntil(&xLastWakeTime, xSampleRate);
  }
}

// UDP Receiver Task
void udpReceiverTask(void *pvParameters)
{
  char incomingPacket[256];
  udp.begin(udpLocalPort);
  Serial.printf("[Receiver] UDP receiver started on port: %d\n", udpLocalPort);
  while (1)
  {
    int packetSize = udp.parsePacket();
    if (packetSize)
    {
      int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0)
      {

        incomingPacket[len] = 0; // Null-terminate
        Serial.printf("[Receiver] Received %d bytes: %s\n", len, incomingPacket);
        Serial.printf("[Receiver] From %s:%d\n", udp.remoteIP().toString().c_str(), udp.remotePort());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{
  Serial.begin(2000000);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  ESP_LOGI(LOGGER_TAG, "[Receiver] UDP receiver started on port: %d", udpLocalPort); // Info level (Green)
  ESP_LOGE(LOGGER_TAG, "Error: %s", "This is an error message");                     // Error level (Red)
  ESP_LOGW(LOGGER_TAG, "Warning: %s", "This is a warning message");                  // Warning level (Yellow)
  ESP_LOGD(LOGGER_TAG, "Debug: %s", "This is a debug message");                      // Debug level (Blue)
  ESP_LOGV(LOGGER_TAG, "Verbose: %s", "This is a verbose message");                  // Verbose level(White)

  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet))
  {
    ESP_LOGE(LOGGER_TAG, "Error: %s", "STA Failed to configure"); // Error level (Red)
  }
  WiFi.begin(ssid, password);
  ESP_LOGI(LOGGER_TAG, "Connecting to WiFi"); // Info level (Green)
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  ESP_LOGI(LOGGER_TAG, "WiFi connected.");    // Info level (Green)
  ESP_LOGI(LOGGER_TAG, "ESP32 IP address: "); // Info level (Green)
  Serial.println(WiFi.localIP());

  // Start FreeRTOS tasks
  xTaskCreatePinnedToCore(udpSenderTask, "UDP Sender", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udpReceiverTask, "UDP Receiver", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  // Nothing here; everything runs in FreeRTOS tasks
}