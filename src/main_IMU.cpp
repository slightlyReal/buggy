// To Upload and run : pio run -t upload -e [env_defined_in_platformio.ini] in our case :
// pio run -t upload -e sampleIMU

#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_pass_config.h"
#include "esp_timer.h" // For esp_timer_get_time()

// Add Logger
#include "esp_log.h"
static const char *LOGGER_TAG = "main_sampleMavlink";

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define BAUD_RATE 2000000

#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

// I2C parameters
static const int32_t I2C_SDA_PIN = 21;
static const int32_t I2C_SCL_PIN = 22;
static const uint32_t I2C_FREQUENCY_HZ = 400000U; // in Hz // Set I2C frequency to 400 kHz


float calculate_ms2(int raw_value) {
  // raw_value = read_accelerometer(); // Read 12-bit value
  /*
     256 counts/g and a zero-g offset of 2048 (for a 12-bit value centered at 0g)
  */
  float sensitivity = 256.0;  // 2^12/(16g-force) (+-8g full scale)
  int16_t zero_g_offset = 0;

  float acceleration_g = (raw_value - zero_g_offset) / sensitivity;
  float acceleration_ms2 = acceleration_g * 9.81;
  return acceleration_ms2;
}

// Gyro instance
static L3G gyro;
static LSM303 compass;

// UDP Sender Task
void imu_read_Task(void *pvParameters)
{

  const TickType_t xSampleRate = pdMS_TO_TICKS(20); // 1 ms = 1 kHz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int64_t start_us = 0;
  int64_t end_us = 0;
  int64_t elapsed_us = 0;

  while (1)
  {
    start_us = esp_timer_get_time(); // Time before transmission

    gyro.read();
    compass.read();

    end_us = esp_timer_get_time(); // Time after transmission
    elapsed_us = end_us - start_us;

    ESP_LOGI(LOGGER_TAG, "Ax:%03d,Ay:%03d,Az:%03d\n", (int)compass.a.x, (int)compass.a.y, (int)compass.a.z);
    ESP_LOGI(LOGGER_TAG, "Gx:%03d,Gy:%03d,Gz:%03d\n", (int)gyro.g.x, (int)gyro.g.y, (int)gyro.g.z);
    ESP_LOGI(LOGGER_TAG, "Mx:%03d,My:%03d,Mz:%03d\n", (int)compass.m.x, (int)compass.m.y, (int)compass.m.z);
    ESP_LOGD(LOGGER_TAG, "Execution took %lld microseconds", elapsed_us); // Warning level


    ESP_LOGI(LOGGER_TAG,"acc_x:%5.2f,acc_y:%5.2f,acc_z:%5.2f", calculate_ms2((int)compass.a.x), calculate_ms2((int)compass.a.y), calculate_ms2((int)compass.a.z));
    // vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
    vTaskDelayUntil(&xLastWakeTime, xSampleRate);
  }
}

void setup()
{
  Serial.begin(2000000);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ); // SDA, SCL, Frequency in H
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  if (!gyro.init())
  {
    ESP_LOGE(LOGGER_TAG,"Failed to autodetect gyro type!");
    while (1)
      ;
  }

  compass.init();
  compass.enableDefault();

  gyro.enableDefault();

  ESP_LOGI(LOGGER_TAG, "[Receiver] UDP receiver started on port: %d", 10); // Info level (Green)
  ESP_LOGE(LOGGER_TAG, "Error: %s", "This is an error message");           // Error level (Red)
  ESP_LOGW(LOGGER_TAG, "Warning: %s", "This is a warning message");        // Warning level (Yellow)
  ESP_LOGD(LOGGER_TAG, "Debug: %s", "This is a debug message");            // Debug level (Blue)
  ESP_LOGV(LOGGER_TAG, "Verbose: %s", "This is a verbose message");        // Verbose level(White)

  // Start FreeRTOS tasks
  xTaskCreatePinnedToCore(imu_read_Task, "imu_read_Task", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  // Nothing here; everything runs in FreeRTOS tasks
}
