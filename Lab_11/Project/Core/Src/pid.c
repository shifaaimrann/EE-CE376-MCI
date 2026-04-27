#include "pid.h"

/* Initial Gain Values - to be tuned via Bluetooth */
float Kp = 100.0f; 
float Ki = 0.65f;
float Kd = 1.25f;

static float integral = 0.0f;
static float prev_error = 0.0f;

/**
 * @brief Computes the PID output based on the tilt angle.
 * @param measured_angle Current filtered angle from IMU.
 * @retval control_signal PWM value for motors.
 */
float PID_Compute(float measured_angle) {
    float error = SETPOINT - measured_angle; // [cite: 184]
    
    // Proportional term: Responds to current error [cite: 184]
    float p_term = Kp * error;

    // Integral term: Accumulates past errors to eliminate steady-state error [cite: 186]
    integral += error * DT;
    
    // Anti-windup: Limit the integral to prevent saturation 
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;
    
    float i_term = Ki * integral;

    // Derivative term: Predicts future error and adds damping [cite: 187, 188]
    float d_term = Kd * (error - prev_error) / DT;
    
    // Final PID calculation [cite: 181]
    float output = p_term + i_term + d_term;

    // Save error for next iteration
    prev_error = error;

    // Output Saturation: Ensure we don't exceed max PWM (usually 1000 for your timer)
    if (output > 1000.0f) output = 1000.0f;
    if (output < -1000.0f) output = -1000.0f;

    return output;
}

/**
 * @brief Resets the controller state to prevent jumpy starts.
 */
void PID_Reset(void) {
    integral = 0.0f;
    prev_error = 0.0f;
}