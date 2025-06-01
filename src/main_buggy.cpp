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

// Logger
#include "esp_log.h"
static const char *LOGGER_TAG = "main_buggy";

// SBUS RX on GPIO 16, TX unused, inverted signal
bfs::SbusRx sbus_rx(&Serial2, 16, -1, true);

// Constants
constexpr int16_t SBUS_MIN = 240;
constexpr int16_t SBUS_MID = 1025;
constexpr int16_t SBUS_DEADBAND = 10;
constexpr int16_t SBUS_MAX = 1807;
constexpr int16_t PPM_MIN_US = 1000;
constexpr int16_t PPM_MAX_US = 2000;
constexpr std::size_t NUM_CH = 10U;
constexpr std::size_t SBUS_QUEUE_LENGTH = 5U;
constexpr uint32_t SBUS_TASK_STACK_SIZE = 4096U;
constexpr uint32_t TASK_STACK_SIZE = 4096U;

constexpr int16_t THROTTLE_CHANNEL = 2;
constexpr int16_t DIRECTION_CHANNEL = 1;

// Global variables
int16_t radio_ch[NUM_CH] = {0};
bfs::SbusData sbus_data; // Structure from sbus.h
QueueHandle_t sbusQueue = nullptr;

// // Motor control pins
const int motor1Pin1 = 17;
const int motor1Pin2 = 18;

// PWM properties
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int PWM_FREQ = 100;
const int PWM_RESOLUTION = 8; // 8 Bit :  0-255
const int DUTY_MAX = 255;

#define BAUD_RATE 2000000

void motor_function(int16_t in_duty)
{
  int8_t duty = abs(in_duty);
  if (in_duty > 0)
  {
    // Forward
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, duty);

    ESP_LOGI(LOGGER_TAG, "Forward : duty : %d\n", duty);
  }
  else if (in_duty < 0)
  {
    // Backward
    ledcWrite(pwmChannel1, duty);
    ledcWrite(pwmChannel2, 0);

    ESP_LOGI(LOGGER_TAG, "Backward : duty : %d\n", duty);
  }

  else
  {
    // Stop
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    ESP_LOGI(LOGGER_TAG, "Stopped.\n");
  }
}

// Convert : SBUS values to PWM Duty Cycle Value
int16_t convert_SBUS_to_THROTTLE(int16_t in_throttle)
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

// Consumer task: receives SBUS data from the queue and processes it
void consumerTask(void *pvParameters)
{
  (void)pvParameters; // Unused parameter

  bfs::SbusData receivedData = {};

  int16_t in_throttle = 0;
  int16_t duty = 0;

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

      duty = convert_SBUS_to_THROTTLE(receivedData.ch[1]);
      motor_function(duty);
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
    ledcWrite(pwmChannel1, duty);
    ledcWrite(pwmChannel2, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Backward
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, duty);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);

  // Configure PWM channels
  ledcSetup(pwmChannel1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(pwmChannel2, PWM_FREQ, PWM_RESOLUTION);

  // Attach channels to GPIO pins
  ledcAttachPin(motor1Pin1, pwmChannel1);
  ledcAttachPin(motor1Pin2, pwmChannel2);

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
}

void loop()
{
  // Main loop can remain empty or perform other tasks
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
