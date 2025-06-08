#ifndef BUGGY_CONFIG_H
#define BUGGY_CONFIG_H

const int sbus_rx_hardwareSerial2_pin = 16;

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

constexpr int16_t RC_CHANNEL_THROTTLE = 1;
constexpr int16_t RC_CHANNEL_DIRECTION = 3;


// // Motor control pins
const int motor1Pin1 = 17;
const int motor1Pin2 = 18;
const int motor2Pin1 = 19;
const int motor2Pin2 = 23;

// PWM properties
const int motor1_pwmChannel1 = 0;
const int motor1_pwmChannel2 = 1;
const int motor2_pwmChannel1 = 2;
const int motor2_pwmChannel2 = 3;

const int PWM_FREQ = 400;
const int PWM_RESOLUTION = 8; // 8 Bit :  0-255
const int DUTY_MAX = 255;

#endif /* BUGGY_CONFIG_H */