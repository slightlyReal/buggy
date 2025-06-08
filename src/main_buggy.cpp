/*
 * Board  : DOIT ESP32 DEVKIT V1
 * Purpose: SBUS to PPM signal conversion with FreeRTOS tasks
 * MISRA/AUTOSAR C++14 style improvements applied
 *
 * “Comply with the strictest safety and reliability standards,
 * specifically latest version of  MISRA, AUTOSAR, NASA JPL C, and ESA coding standards
 * MIL-STD-498, Military Standard Software Development and Documentation
 * ToDo : search  for the best-practice approach for refactoring the codebase accordingly.”
 */

// To Compile&Upload : pio run -t upload -e buggy

// References
/*
1. https://www.youtube.com/watch?v=Dia-r1UuCZk&ab_channel=MATLAB
2. Making a Simple DC Motor :  https://www.youtube.com/watch?v=CBI0Mz9xeqg&ab_channel=QuantumBoffin
3. https://www.youtube.com/watch?v=CWulQ1ZSE3c&ab_channel=JaredOwen
*/

#include <Arduino.h>
#include "sbus.h"
#include "constants/math_constants.h"
#include "buggy_config/buggy_config.h"

// Logger
#include "esp_log.h"
static const char *LOGGER_TAG = "main_buggy";

// SBUS RX on GPIO 16, TX unused, inverted signal
bfs::SbusRx sbus_rx(&Serial2, sbus_rx_hardwareSerial2_pin, -1, true);

// Global variables
int16_t radio_ch[NUM_CH] = {0};
bfs::SbusData sbus_data; // Structure from sbus.h
QueueHandle_t sbusQueue = nullptr;

#define BAUD_RATE 2000000

// Mavlink
#include <WiFi.h>
#include <WiFiUdp.h>
// Define MAVLink Communication Protocol
#ifdef F
#undef F
#endif
#include <common/mavlink.h>
#include "conf.h"
#include "wifi_pass_config.h"
#include "buggy_mavlink.h"

void motor_function(int16_t in_duty_main, int16_t in_duty_direction)
{
  int16_t duty_main = abs(in_duty_main);
  int16_t duty_direction = abs(in_duty_direction);
  if (in_duty_main > 0)
  {
    // Forward
    ledcWrite(motor1_pwmChannel1, 0);
    ledcWrite(motor1_pwmChannel2, duty_main);

    ESP_LOGI(LOGGER_TAG, "Forward : duty : %d\n", duty_main);
  }
  else if (in_duty_main < 0)
  {
    // Backward
    ledcWrite(motor1_pwmChannel1, duty_main);
    ledcWrite(motor1_pwmChannel2, 0);

    ESP_LOGI(LOGGER_TAG, "Backward : duty : %d\n", duty_main);
  }
  else
  {
    // Stop
    ledcWrite(motor1_pwmChannel1, 0);
    ledcWrite(motor1_pwmChannel2, 0);
    ESP_LOGI(LOGGER_TAG, "Stopped.\n");
  }

  if (in_duty_direction > 0)
  {
    // Forward
    ledcWrite(motor2_pwmChannel1, 0);
    ledcWrite(motor2_pwmChannel2, duty_direction);

    ESP_LOGI(LOGGER_TAG, "Right : duty : %d\n", duty_direction);
  }
  else if (in_duty_direction < 0)
  {
    // Backward
    ledcWrite(motor2_pwmChannel1, duty_direction);
    ledcWrite(motor2_pwmChannel2, 0);

    ESP_LOGI(LOGGER_TAG, "Left : duty : %d\n", duty_direction);
  }
  else
  {
    // Stop
    ledcWrite(motor2_pwmChannel1, 1);
    ledcWrite(motor2_pwmChannel2, 1);
    ESP_LOGI(LOGGER_TAG, "Stopped.\n");
  }
}

// Convert : SBUS values to PWM Duty Cycle Value
int16_t convert_SBUS_to_DUTY(int16_t in_throttle)
{
  int16_t out_duty = 0;
  //  Apply  DeadBand  Logic
  if ((in_throttle >= (SBUS_MID - SBUS_DEADBAND)) && (in_throttle <= (SBUS_MID + SBUS_DEADBAND)))
  {
    in_throttle = SBUS_MID;
  }
  else if (in_throttle > SBUS_MAX)
  {
    in_throttle = SBUS_MAX;
  }
  else if (in_throttle < SBUS_MIN)
  {
    in_throttle = SBUS_MIN;
  }
  else
  {
    in_throttle = in_throttle;
  }

  // Map Values
  if (in_throttle > SBUS_MID)
    out_duty = map(in_throttle, SBUS_MID, SBUS_MAX, 0, DUTY_MAX);
  else if (in_throttle < SBUS_MID)
    out_duty = map(in_throttle, SBUS_MIN, SBUS_MID, -DUTY_MAX, 0);
  else
    out_duty = 0;

  ESP_LOGI(LOGGER_TAG, "out_duty : %d\n", out_duty);
  return out_duty;
}

// Producer task: reads SBUS data and sends it to the queue
void sbusTask(void *const pvParameters)
{
  (void)pvParameters; // Unused parameter

  int64_t previous_time_us = esp_timer_get_time(); // Initialize to current time
  int64_t task_start_time_us = 0;
  int64_t total_time_us = 0;
  float update_rate_hz = 0.0f;

  for (;;)
  {
    if (sbus_rx.Read())
    {
      task_start_time_us = esp_timer_get_time(); // Time before transmission
      total_time_us = task_start_time_us - previous_time_us;
      previous_time_us = task_start_time_us;
      sbus_data = sbus_rx.data();

      BaseType_t result = xQueueSend(sbusQueue, &sbus_data, portMAX_DELAY);
      if (result != pdPASS)
      {
        ESP_LOGE(LOGGER_TAG, "SBUS queue send failed!");
      }

      // Calculate update rate in Hz, avoiding division by zero
      if (total_time_us > 0)
      {
        update_rate_hz = (MICROSECONDS_PER_SECOND_F) / static_cast<float>(total_time_us);
      }
      else
      {
        update_rate_hz = 0.0f;
      }

      // ESP_LOGI(LOGGER_TAG, "Function triggered %lld microseconds (%.2f Hz) ago.", total_time_us, update_rate_hz);
    }
    vTaskDelay(pdMS_TO_TICKS(SBUS_TASK_DELAY_MS)); // 50 Hz update rate
  }
}

// Producer task: reads SBUS data and sends it to the queue
void mavlink_telemetryTask(void *const pvParameters)
{
  (void)pvParameters; // Unused parameter

  unsigned long currentMillis = millis();
  float sine_value = 0.0f;
  float cosine_value = 0.0f;
  for (;;)
  {
    if (true)
    {
      send_heartbeat();
      send_systemstatus();
      send_radiostatus();
      send_position();
      send_mavlink_msg_rc_channels(10);

      currentMillis = millis();
      sine_value = sin(currentMillis);
      cosine_value = cos(currentMillis);

      send_mavlink_attitude(currentMillis, (cosine_value * 10.0f * M_PI / 180.0f), (cosine_value * 20.0f * M_PI / 180.0f), (cosine_value * 30.0f * M_PI / 180.0f));
      send_mavlink_SCALED_IMU(currentMillis, (int16_t)(cosine_value * 10.0f), (int16_t)(cosine_value * 10.0f), (int16_t)(cosine_value * 10.0f));
      send_mavlink_local_position_ned(currentMillis, 1, 2, 3);
    }

    vTaskDelay(pdMS_TO_TICKS(MAVLINK_TASK_DELAY_MS)); // update rate
  }
}

// Consumer task: receives SBUS data from the queue and processes it
void consumerTask(void *pvParameters)
{
  (void)pvParameters; // Unused parameter

  bfs::SbusData receivedData = {};

  int16_t in_throttle, duty, duty_direction = 0;

  for (;;)
  {
    BaseType_t result = xQueueReceive(sbusQueue, &receivedData, portMAX_DELAY);
    if (result == pdPASS)
    {

      ESP_LOGI(LOGGER_TAG, "SBUS : ");
      // Print raw SBUS channel data
      for (std::size_t i = 0U; i < NUM_CH; ++i)
      {
        Serial.printf("CH__%2u: %4d ", static_cast<unsigned>(i + 1U), receivedData.ch[i]);
      }
      Serial.printf("Lost Frame: %d\tFailsafe: %d\n", receivedData.lost_frame, receivedData.failsafe);

      duty = convert_SBUS_to_DUTY(receivedData.ch[RC_CHANNEL_THROTTLE]);
      duty_direction = convert_SBUS_to_DUTY(receivedData.ch[RC_CHANNEL_DIRECTION]);
      motor_function(duty, duty_direction);
      ESP_LOGI(LOGGER_TAG, "duty : %d\n", duty);

      if (false)
      {
        // Map SBUS channels to PPM microseconds and print
        for (std::size_t i = 0U; i < NUM_CH; ++i)
        {
          radio_ch[i] = map(receivedData.ch[i], SBUS_MIN, SBUS_MAX, PPM_MIN_US, PPM_MAX_US);
          Serial.printf("CHr_%2u: %4d ", static_cast<unsigned>(i + 0U), radio_ch[i]);
        }
        Serial.println();
      }
    }
    else
    {
      ESP_LOGE(LOGGER_TAG, "SBUS queue receive failed!");
    }
  }
}

void motorTask(void *parameter)
{
  uint8_t duty = 50;
  while (1)
  {
    // Forward
    ledcWrite(motor1_pwmChannel1, duty);
    ledcWrite(motor1_pwmChannel2, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop
    ledcWrite(motor1_pwmChannel1, 0);
    ledcWrite(motor1_pwmChannel2, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Backward
    ledcWrite(motor1_pwmChannel1, 0);
    ledcWrite(motor1_pwmChannel2, duty);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop
    ledcWrite(motor1_pwmChannel1, 0);
    ledcWrite(motor1_pwmChannel2, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);
  init_wifi();

  // Configure PWM channels
  ledcSetup(motor1_pwmChannel1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motor1_pwmChannel2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motor2_pwmChannel1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motor2_pwmChannel2, PWM_FREQ, PWM_RESOLUTION);

  // Attach channels to GPIO pins
  ledcAttachPin(motor1Pin1, motor1_pwmChannel1);
  ledcAttachPin(motor1Pin2, motor1_pwmChannel2);
  ledcAttachPin(motor2Pin1, motor2_pwmChannel1);
  ledcAttachPin(motor2Pin2, motor2_pwmChannel2);

  vTaskDelay(1000 / portTICK_PERIOD_MS); // Non Blocking Delay
  while (!Serial)
  { /* Wait for Serial port */
  }

  ESP_LOGI(LOGGER_TAG, "SBUS Communication: Initializing...");
  sbus_rx.Begin();
  ESP_LOGI(LOGGER_TAG, "SBUS Communication: Established.");
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Non Blocking Delay

  // Create the queue to hold up to SBUS_QUEUE_LENGTH SBUS data items
  sbusQueue = xQueueCreate(SBUS_QUEUE_LENGTH, sizeof(bfs::SbusData));
  if (sbusQueue == nullptr)
  {
    ESP_LOGE(LOGGER_TAG, "Error creating the SBUS queue!");
    while (true)
    { /* Halt if queue creation fails */
    }
  }

  // Create FreeRTOS tasks pinned to core 1
  BaseType_t taskResult;
  taskResult = xTaskCreatePinnedToCore(
      sbusTask,
      "SBUS Task",
      SBUS_TASK_STACK_SIZE,
      nullptr,
      1,
      nullptr,
      1);
  if (taskResult != pdPASS)
  {
    ESP_LOGE(LOGGER_TAG, "Failed to create sbusTask!");
    while (true)
    { /* Halt on failure */
    }
  }

  taskResult = xTaskCreatePinnedToCore(
      consumerTask,
      "Consumer Task",
      TASK_STACK_SIZE,
      nullptr,
      1,
      nullptr,
      1);
  if (taskResult != pdPASS)
  {
    ESP_LOGE(LOGGER_TAG, "Failed to create consumerTask!");
    while (true)
    { /* Halt on failure */
    }
  }

  /*
  // Create the FreeRTOS task
  xTaskCreatePinnedToCore(
      motorTask,    // Task function
      "Motor Task", // Name
      2048,         // Stack size
      NULL,         // Parameter
      1,            // Priority
      NULL,         // Task handle
      1             // Core (0 or 1)
  );
  */

  taskResult = xTaskCreatePinnedToCore(
      mavlink_telemetryTask,
      "mavlink_telemetryTask",
      TASK_STACK_SIZE,
      nullptr,
      1,
      nullptr,
      1);
  if (taskResult != pdPASS)
  {
    ESP_LOGE(LOGGER_TAG, "Failed to create mavlink_telemetryTask!");
    while (true)
    { /* Halt on failure */
    }
  }
}

void loop()
{
  // Main loop can remain empty or perform other tasks
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
