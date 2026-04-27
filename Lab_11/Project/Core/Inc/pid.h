#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

/* Controller constants from Lab Manual */
//#define SETPOINT 0.2f  /* Target is the upright vertical position [cite: 195] */
#define DT 0.005f      /* Must match IMU sampling rate [cite: 210] */

/* Global Gains for Bluetooth Tuning */
extern float Kp;
extern float Ki;
extern float Kd;

/* Prototypes */
float PID_Compute(float measured_angle);
void PID_Reset(void);

#endif