#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

/* Constants from Lab Manual */
#define DT              0.005f      /* 200 Hz sampling [cite: 23] */
#define RAD_TO_DEG      57.2957795f
#define ACC_SENS        0.00098f
#define GYRO_SENS       0.00875f

/* Structures for Sensor Data */
typedef struct {
    int16_t x_raw, y_raw, z_raw;
    float x_g, y_g, z_g;
} AccelData;

typedef struct {
    int16_t x_raw, y_raw, z_raw;
    float x_dps, y_dps, z_dps;
} GyroData;

/* Prototypes */
void IMU_Init(void);
void IMU_Process(void);
float IMU_GetAngle(void);
void IMU_Debug_Raw(void); // Validation function for Phase 2

#endif