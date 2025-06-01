#ifndef MATH_CONSTANTS_H
#define MATH_CONSTANTS_H

#define PI_F (3.14159265f)
#define DEG2RAD_F (PI_F / 180.0f)
#define RAD2DEG_F (180.0f / PI_F)
#define MILLISECONDS_PER_SECOND (1000.0f)
#define MICROSECONDS_PER_SECOND (1000000.0f)

constexpr uint32_t SBUS_TASK_DELAY_MS = 10U; // Task DelayTime in Milliseconds = 100 Hz
constexpr float MICROSECONDS_PER_SECOND_F = 1000000.0f;

#endif /* MATH_CONSTANTS_H */

