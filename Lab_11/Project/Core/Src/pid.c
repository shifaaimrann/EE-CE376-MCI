#include "pid.h"

/* Initial Gain Values - to be tuned via Bluetooth */
float SETPOINT = 0.27f;
float Kp = 100.0f; 
float Ki = 0.02f;
float Kd = 2.0f;

static float integral = 0.0f;
static float prev_error = 0.0f;

/**
 * @brief Computes the PID output based on the tilt angle.
 * @param measured_angle Current filtered angle from IMU.
 * @retval control_signal PWM value for motors.
 */
float PID_Compute(float current_angle) 
{
    float error = SETPOINT - current_angle;
    static float integral = 0.0f;
    static float prev_error = 0.0f;

    // 1. Proportional (Muscle)
    float P = Kp * error;

    // 2. Integral (Drift Catcher) - WITH ANTI-WINDUP CLAMP
    integral += error;
    
    // Safety Limits: Do not let the memory build up too high!
    if (integral > 200.0f) integral = 200.0f;
    if (integral < -200.0f) integral = -200.0f;
    
    float I = Ki * integral;

    // 3. Derivative (Shock Absorber)
    float derivative = error - prev_error;
    float D = Kd * derivative;

    // Save error for next time
    prev_error = error;

    return P + I + D;
}
/**
 * @brief Resets the controller state to prevent jumpy starts.
 */
void PID_Reset(void) {
    integral = 0.0f;
    prev_error = 0.0f;
}