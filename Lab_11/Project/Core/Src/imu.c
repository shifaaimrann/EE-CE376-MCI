#include "imu.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Register Defines */
#define ACC_ADDR            (0x19 << 1)
#define CTRL_REG1_A         0x20
#define CTRL_REG4_A         0x23
#define OUT_X_L_A           0x28
#define GYRO_CS_PORT        GPIOE
#define GYRO_CS_PIN         GPIO_PIN_3
#define GYRO_READ_BIT       0x80
#define GYRO_MULTI_BIT      0x40
#define CTRL_REG1_G         0x20
#define CTRL_REG4_G         0x23
#define OUT_X_L_G           0x28

/* External Handles */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

/* Internal Variables */
static AccelData acc;
static GyroData gyro;
static float angle = 0.0f;
static float acc_offset = 0.0f;

/* Low-Level Helpers (Your Original Logic) */
static void accel_write(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static void gyro_write(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

/* Initialization */
void IMU_Init(void) {
    /* Accel: 100Hz, ±16g */
    accel_write(CTRL_REG1_A, 0x57);
    accel_write(CTRL_REG4_A, 0x88);

    /* Gyro: Power on, 245 dps */
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    gyro_write(CTRL_REG1_G, 0x0F);
    gyro_write(CTRL_REG4_G, 0x80);

    /* Calibrate Initial Offset */
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    acc.y_raw = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    acc.z_raw = (int16_t)((buf[5] << 8) | buf[4]) >> 4;
    acc_offset = atan2f(acc.y_raw * ACC_SENS, acc.z_raw * ACC_SENS) * RAD_TO_DEG;
}

/* Validation Function: Check if pins/sensors are fried */
void IMU_Debug_Raw(void) {
    char msg[100];
    sprintf(msg, "RawAccY:%d RawGyroY:%d\r\n", acc.y_raw, gyro.y_raw);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
}

/* Core Processing Loop */
void IMU_Process(void) {
    uint8_t a_buf[6], g_tx[7] = {GYRO_READ_BIT | GYRO_MULTI_BIT | OUT_X_L_G, 0}, g_rx[7];

    /* Read Accel */
    HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, a_buf, 6, HAL_MAX_DELAY);
    acc.y_raw = (int16_t)((a_buf[3] << 8) | a_buf[2]) >> 4;
    acc.z_raw = (int16_t)((a_buf[5] << 8) | a_buf[4]) >> 4;

    /* Read Gyro */
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, g_tx, g_rx, 7, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    gyro.y_raw = (int16_t)((g_rx[4] << 8) | g_rx[3]);

    /* Convert & Filter  */
    float acc_angle = atan2f(acc.y_raw * ACC_SENS, acc.z_raw * ACC_SENS) * RAD_TO_DEG - acc_offset;
    float gyro_rate = gyro.y_raw * GYRO_SENS;

    /* Formula: 98% Gyro + 2% Accel  */
    angle = 0.98f * (angle + gyro_rate * DT) + 0.02f * acc_angle;
}

float IMU_GetAngle(void) {
    return angle;
}