#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/* Public Functions */
void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

/* Debugging Functions for Phase 2 Hardware Checks */
void Motor_Debug_DirectionTest(void); // Test CW and CCW logic
void Motor_Debug_PWMSweep(void);     // Test if PWM range is linear

#endif /* INC_MOTOR_H_ */