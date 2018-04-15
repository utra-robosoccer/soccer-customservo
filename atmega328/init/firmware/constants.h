#ifndef _CONSTANTS_H
#define _CONSTANTS_H 1

#define BUFF_SIZE 512
#define ADC_TRIGGER 111 // 10.7V after 220ohm-4.7kohm divider, for 1.1V bandgap reference

#define ENCODER_CPR 1600
#define MOTOR_PPR 14
#define ENCODER_DIV_MOTOR_GCD 800 // 800 ticks over 7 phase rotations

#define NUM_SAMPLES_PER_PWM 14
#define PWM_CYCLE_SKIP 32
#define SQRT3_OVER_2 0.8660254038
#define OFF_MASK_P 0b000
#define OFF_MASK_N 0b000
// #define F_CPU 32000000L

#define OPEN_LOOP_DRIVE_PER 8192 // 12-bit timer
#define OPEN_LOOP_DRIVE_THRESHOLD 2400 // 12.5% duty max
#define OPEN_LOOP_DRIVE_PRESCALER 0b100 // 1x prescaler

#define INDUCTANCE_ANGLE_SCHEDULER_PRESCALER 0b100 // 8x prescaler
#define INDUCTANCE_ANGLE_SAMPLES 10
#define SATURATION_TIME_THRESHOLD 70

#define TIMING_MU 682
// #define MU_B 682

#define NUM_HALF_SINE 256
const uint8_t HALF_SINE[NUM_HALF_SINE] = { 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 20, 21, 22, 23, 23, 24, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 39, 40, 41, 41, 42, 42, 43, 43, 44, 45, 45, 46, 46, 47, 47, 48, 48, 49, 49, 50, 50, 51, 51, 52, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 56, 57, 57, 57, 58, 58, 58, 58, 59, 59, 59, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 60, 60, 60, 60, 59, 59, 59, 58, 58, 58, 58, 57, 57, 57, 56, 56, 56, 55, 55, 54, 54, 54, 53, 53, 52, 52, 52, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 46, 46, 45, 45, 44, 43, 43, 42, 42, 41, 41, 40, 39, 39, 38, 38, 37, 36, 36, 35, 34, 34, 33, 32, 32, 31, 30, 30, 29, 28, 28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 20, 20, 19, 18, 18, 17, 16, 15, 15, 14, 13, 12, 12, 11, 10, 9, 8, 8, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0 };

#endif