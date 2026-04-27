#include "motor.h"

extern TIM_HandleTypeDef htim3;

/**
 * @brief Initializes the PWM timer channels for the motors.
 */
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Right Motor PWM (PB4)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Left Motor PWM (PB5)
    Motor_Stop();
}

/**
 * @brief Sets direction and speed for both motors. 
 * Positive speed = Clockwise, Negative speed = Counterclockwise[cite: 292, 293].
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed) {
    // Left Motor Logic (PD10, PD11, TIM3_CH2)
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        left_speed = -left_speed; // Absolute value for PWM
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_speed);

    // Right Motor Logic (PD8, PD9, TIM3_CH1)
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
        right_speed = -right_speed; // Absolute value for PWM
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right_speed);
}

void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
}

/* === DEBUGGING FUNCTIONS === */

/**
 * @brief Use this to verify if the 8V mistake fried specific direction pins.
 * Toggles each motor Forward/Backward every 2 seconds.
 */
void Motor_Debug_DirectionTest(void) {
    Motor_SetSpeed(500, 500); // Both Forward
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(500);
    Motor_SetSpeed(-500, -500); // Both Backward
    HAL_Delay(2000);
    Motor_Stop();
}

/**
 * @brief Verifies if the PWM timers (PB4/PB5) are outputting correctly.
 * Slowly ramps speed from 0 to 1000.
 */
void Motor_Debug_PWMSweep(void) {
    for (int i = 0; i <= 1000; i += 100) {
        Motor_SetSpeed(i, i);
        HAL_Delay(500);
    }
    Motor_Stop();
}