/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
//TASK1--------------------------------------------------------------------------------------------
// #define LSM_READ_ADDR      0x33   // 8-bit read address
// #define WHO_AM_I_REG       0x0F   // WHO_AM_I register
// #define EXPECTED_ID        0x33   // Expected response

//TASK2------------------------------------------------------------------------------------------

/* --- SPI Gyroscope (L3GD20) --- */
#define GYRO_READ_BIT     0x80
#define CTRL_REG1         0x20
#define CTRL_REG1_VAL     0b00001111
#define CTRL_REG4         0x23
#define CTRL_REG4_VAL     0x00          // 245 dps, most sensitive
#define GYRO_SENS         0.00875f      // °/s per LSB at 245dps
#define OUT_X_L           0x28
#define OUT_X_H           0x29
#define OUT_Y_L           0x2A
#define OUT_Y_H           0x2B
#define OUT_Z_L           0x2C
#define OUT_Z_H           0x2D

#define CS_LOW()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

/* --- I2C Accelerometer (LSM303AGR) --- */
#define LSM_WRITE_ADDR    0x32
#define LSM_READ_ADDR     0x33
#define CTRL_REG1_A       0x20
#define CTRL_REG4_A       0x23
#define OUT_X_L_A         0x28
#define OUT_X_H_A         0x29
#define OUT_Y_L_A         0x2A
#define OUT_Y_H_A         0x2B
#define OUT_Z_L_A         0x2C
#define OUT_Z_H_A         0x2D
#define ACCEL_SCALE       3.9f
#define RAD_TO_DEG        57.2958f

/* --- Drift Analysis --- */
#define DRIFT_SAMPLES     100           // samples to measure drift over

typedef struct {
    // Raw gyro
    int16_t gx_raw, gy_raw, gz_raw;
    // Gyro in dps
    float   gx_dps, gy_dps, gz_dps;
    // Gyro offsets
    int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;

    // Raw accel
    int16_t ax_raw, ay_raw, az_raw;
    // Accel in G
    float   ax_g, ay_g, az_g;
    // Accel offsets
    float   acc_offset_x, acc_offset_y, acc_offset_z;

    // Angles
    float pitch, roll;

    // Drift metrics
    float gyro_drift_pct;
    float accel_drift_pct;
} SensorData;

SensorData sensor;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//TASK1----------------------------------------------------------------------------------
// void myPrintf(const char *fmt, ...)
// {
//     char buffer[256];
//     va_list args;
//     va_start(args, fmt);
//     vsnprintf(buffer, sizeof(buffer), fmt, args);
//     va_end(args);
//     HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
// }

//TASK2-----------------------------------------------------------------------------------

void myPrintf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

uint8_t spi_read(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = { 0, 0 };
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();
    return rx[1];
}

void spi_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg & 0x7F, value };
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

void Gyro_Init(void)
{
    uint8_t tx[2];

    // Power on, enable X/Y/Z
    tx[0] = CTRL_REG1; tx[1] = CTRL_REG1_VAL;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_HIGH();

    // Set full scale range
    tx[0] = CTRL_REG4; tx[1] = CTRL_REG4_VAL;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

void Gyro_Calibrate(void)
{
    int32_t xs = 0, ys = 0, zs = 0;
    const int samples = 30;

    myPrintf("Gyro calibrating... keep still\r\n");

    for (int i = 0; i < samples; i++)
    {
        int16_t x = (int16_t)(spi_read(OUT_X_H) << 8 | spi_read(OUT_X_L));
        int16_t y = (int16_t)(spi_read(OUT_Y_H) << 8 | spi_read(OUT_Y_L));
        int16_t z = (int16_t)(spi_read(OUT_Z_H) << 8 | spi_read(OUT_Z_L));
        xs += x; ys += y; zs += z;
        HAL_Delay(5);
    }

    sensor.gyro_offset_x = xs / samples;
    sensor.gyro_offset_y = ys / samples;
    sensor.gyro_offset_z = zs / samples;

    myPrintf("Gyro offsets: %d, %d, %d\r\n",
             sensor.gyro_offset_x,
             sensor.gyro_offset_y,
             sensor.gyro_offset_z);
}

void Gyro_Read(void)
{
    uint8_t buf[6];
    uint8_t reg = OUT_X_L | 0x80 | 0x40; // read + auto-increment

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
    CS_HIGH();

    sensor.gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
    sensor.gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
    sensor.gz_raw = (int16_t)((buf[5] << 8) | buf[4]);

    sensor.gx_dps = (sensor.gx_raw - sensor.gyro_offset_x) * GYRO_SENS;
    sensor.gy_dps = (sensor.gy_raw - sensor.gyro_offset_y) * GYRO_SENS;
    sensor.gz_dps = (sensor.gz_raw - sensor.gyro_offset_z) * GYRO_SENS;
}

/* ============================================================
   ACCELEROMETER FUNCTIONS
   ============================================================ */
static int16_t Accel_ReadAxis(uint8_t reg_l, uint8_t reg_h)
{
    uint8_t lo, hi;
    HAL_I2C_Mem_Read(&hi2c1, LSM_READ_ADDR, reg_l,
                     I2C_MEMADD_SIZE_8BIT, &lo, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM_READ_ADDR, reg_h,
                     I2C_MEMADD_SIZE_8BIT, &hi, 1, HAL_MAX_DELAY);
    return (int16_t)((hi << 8) | lo);  // as per lab sheet, no shift
}

void Accel_Init(void)
{
    uint8_t val = 0x67;  // ODR=100Hz, normal mode, X/Y/Z enabled
    HAL_I2C_Mem_Write(&hi2c1, LSM_WRITE_ADDR, CTRL_REG1_A,
                      I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
    val = 0x00;          // normal mode, ±2g
    HAL_I2C_Mem_Write(&hi2c1, LSM_WRITE_ADDR, CTRL_REG4_A,
                      I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

void Accel_Calibrate(void)
{
    float sx = 0, sy = 0, sz = 0;
    const int samples = 20;

    myPrintf("Accel calibrating... keep still\r\n");

    for (int i = 0; i < samples; i++)
    {
        sx += Accel_ReadAxis(OUT_X_L_A, OUT_X_H_A) * ACCEL_SCALE;
        sy += Accel_ReadAxis(OUT_Y_L_A, OUT_Y_H_A) * ACCEL_SCALE;
        sz += Accel_ReadAxis(OUT_Z_L_A, OUT_Z_H_A) * ACCEL_SCALE;
        HAL_Delay(5);
    }

    sensor.acc_offset_x = sx / samples;
    sensor.acc_offset_y = sy / samples;
    sensor.acc_offset_z = (sz / samples) - 1000.0f; // remove 1G gravity

    myPrintf("Accel offsets: %.2f, %.2f, %.2f\r\n",
             sensor.acc_offset_x,
             sensor.acc_offset_y,
             sensor.acc_offset_z);
}

void Accel_Read(void)
{
    sensor.ax_g = (Accel_ReadAxis(OUT_X_L_A, OUT_X_H_A) * ACCEL_SCALE
                  - sensor.acc_offset_x) / 1000.0f;
    sensor.ay_g = (Accel_ReadAxis(OUT_Y_L_A, OUT_Y_H_A) * ACCEL_SCALE
                  - sensor.acc_offset_y) / 1000.0f;
    sensor.az_g = (Accel_ReadAxis(OUT_Z_L_A, OUT_Z_H_A) * ACCEL_SCALE
                  - sensor.acc_offset_z) / 1000.0f;

    sensor.pitch = atan2f(sensor.ax_g, sensor.az_g) * RAD_TO_DEG;
    sensor.roll  = atan2f(sensor.ay_g, sensor.az_g) * RAD_TO_DEG;
}

void Sensor_AnalyseDrift(void)
{
    float gyro_min = 999999.0f, gyro_max = -999999.0f;
    float accel_min = 999999.0f, accel_max = -999999.0f;

    myPrintf("\r\n--- Drift Analysis (%d samples, board still) ---\r\n",
             DRIFT_SAMPLES);

    for (int i = 0; i < DRIFT_SAMPLES; i++)
    {
        Gyro_Read();
        Accel_Read();

        // Track gyroX range (should be ~0 when still)
        if (sensor.gx_dps < gyro_min) gyro_min = sensor.gx_dps;
        if (sensor.gx_dps > gyro_max) gyro_max = sensor.gx_dps;

        // Track pitch range (should be ~0 when still)
        if (sensor.pitch < accel_min) accel_min = sensor.pitch;
        if (sensor.pitch > accel_max) accel_max = sensor.pitch;

        HAL_Delay(10);
    }

    // Drift = peak-to-peak variation
    float gyro_drift  = gyro_max  - gyro_min;
    float accel_drift = accel_max - accel_min;

    // Percentage drift relative to full scale
    // Gyro full scale = 245 dps, Accel angle full scale = 180 deg
    sensor.gyro_drift_pct  = (gyro_drift  / 245.0f)  * 100.0f;
    sensor.accel_drift_pct = (accel_drift / 180.0f) * 100.0f;

    myPrintf("Gyro  drift: %.3f dps  (%.3f%% of 245dps range)\r\n",
             gyro_drift, sensor.gyro_drift_pct);
    myPrintf("Accel drift: %.3f deg  (%.3f%% of 180deg range)\r\n",
             accel_drift, sensor.accel_drift_pct);

    myPrintf("\r\nConclusion: ");
    if (sensor.accel_drift_pct < sensor.gyro_drift_pct)
    {
        myPrintf("Accelerometer is more stable for STATIC angle.\r\n");
        myPrintf("Preferred for slow/static tilt estimation.\r\n");
    }
    else
    {
        myPrintf("Gyroscope is more stable for DYNAMIC angle.\r\n");
        myPrintf("Preferred for fast/dynamic rotation detection.\r\n");
    }
    myPrintf("--- End Drift Analysis ---\r\n\r\n");
}

void Sensor_Print(void)
{
    myPrintf("aX:%.2f,aY:%.2f,aZ:%.2f,gX:%.2f,gY:%.2f,gZ:%.2f,pitch:%.2f\r\n",
             sensor.ax_g,
             sensor.ay_g,
             sensor.az_g,
             sensor.gx_dps,
             sensor.gy_dps,
             sensor.gz_dps,
             sensor.pitch);
}
void Sensor_Update(void)
{
    Gyro_Read();
    Accel_Read();
    Sensor_Print();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  //TASK 1----------------------------------------------------------------------------
  //  HAL_Delay(100);  // sensor boot time

  //   uint8_t who_am_i = 0;
  //   HAL_StatusTypeDef ret;

  //   // Read WHO_AM_I register (0x0F) from LSM303AGR
  //   ret = HAL_I2C_Mem_Read(&hi2c1, LSM_READ_ADDR, WHO_AM_I_REG,
  //                           I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);

  //   if (ret != HAL_OK)
  //   {
  //       myPrintf("I2C ERROR: %d\r\n", ret);
  //   }
  //   else
  //   {
  //       myPrintf("WHO_AM_I = 0x%02X\r\n", who_am_i);

  //       if (who_am_i == EXPECTED_ID)
  //           myPrintf("LSM303AGR detected successfully!\r\n");
  //       else
  //           myPrintf("Unexpected ID! Check wiring/address.\r\n");
  //   }
  //TASK 2-------------------------------------------------------------------------
    CS_HIGH();
    HAL_Delay(100);

    void myPrintf(const char *fmt, ...);

/* Gyro */
void Gyro_Init(void);
void Gyro_Calibrate(void);
void Gyro_Read(void);

/* Accel */
void Accel_Init(void);
void Accel_Calibrate(void);
void Accel_Read(void);

/* Sensor */
void Sensor_AnalyseDrift(void);
void Sensor_Print(void);
void Sensor_Update(void);

  /* USER CODE END 2 */
  


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    Sensor_Update();
    HAL_Delay(100);
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; //new
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
