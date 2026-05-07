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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ---- MPU6050 register map ---- */
#define MPU6050_ADDR        0xD0
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define ACCEL_CONFIG_REG    0x1C
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

#define RAD_TO_DEG  57.2957795f
#define INPUT_MAX   1000
#define PWM_MAX    999
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim20;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---- LIDAR 1 (USART1) ---- */
uint8_t  rx1_byte;
uint8_t  tf1_frame[9];
uint8_t  tf1_index      = 0;
volatile uint8_t  tf1_ready     = 0;
volatile uint16_t distance1_mm  = 0;

/* ---- LIDAR 2 (USART2) ---- */
uint8_t  rx2_byte;
uint8_t  tf2_frame[9];
uint8_t  tf2_index      = 0;
volatile uint8_t  tf2_ready     = 0;
volatile uint16_t distance2_mm  = 0;

/* ---- Distance display (integer + fractional metres) ---- */
uint16_t m_int1 = 0, m_frac1 = 0;
uint16_t m_int2 = 0, m_frac2 = 0;

/* ---- UART4 joystick receive ---- */
char     Rx_buff[64];
uint8_t  rx_index       = 0;
volatile uint8_t uart_line_ready = 0;

/* ---- Joystick axes ---- */
int lx = 0, ly = 0, rx_joy = 0, ry = 0;   /* renamed rx → rx_joy (avoids clash with UART handle) */
int l1 = 0, r1 = 0, l2 = 0, r2 = 0;

/* ---- Mecanum motor values ---- */
int32_t        mFL, mFR, mBL, mBR;
uint16_t       pwmFL, pwmFR, pwmBL, pwmBR;
GPIO_PinState  dirFL, dirFR, dirBL, dirBR;

/* ---- MPU6050 raw ---- */
int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;
int16_t Gyro_X_RAW  = 0, Gyro_Y_RAW  = 0, Gyro_Z_RAW  = 0;

/* ---- MPU6050 processed ---- */
float Ax, Ay, Az;
float Gx, Gy, Gz;

/* ---- Gyro offsets (set during calibration) ---- */
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

/* ---- Accelerometer angles ---- */
float roll_acc  = 0.0f;
float pitch_acc = 0.0f;

/* ---- Final angles (complementary filter) ---- */
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

/* yaw cannot be corrected by accel — pure gyro integration */
float yaw_gyro = 0.0f;

/* Complementary filter weight (0.96 = trust gyro 96 %, accel 4 %) */
#define CF_ALPHA  0.96f

/* ---- IMU timing ---- */
uint32_t imu_last_tick = 0;
float    imu_dt        = 0.0f;

/* ---- Yaw PID ---- */
float targetYaw     = 0.0f;
float yaw_error     = 0.0f;
float yaw_prev_error= 0.0f;
float yaw_integral  = 0.0f;
uint32_t prevYawTime = 0;

float kp = 7.0f;
float ki = 0.0f;
float kd = 0.0f;

/* ---- Misc ---- */
uint8_t check = 0;
uint8_t Data  = 0;
char    cdc_buf[160];   /* single shared CDC TX buffer — sized for full debug line */


/* ---------------- ODOMETRY ---------------- */

#define PI 3.14159265359f
#define CPR 1200.0f

float R_wheel = 3.0f;

/* Dead wheel offsets from robot center (cm) */
float x_offset = 8.0f;
float y_offset = 8.0f;

/* Encoder counts */
int32_t x_now = 0;
int32_t y_now = 0;

int32_t x_prev = 0;
int32_t y_prev = 0;

/* Delta ticks */
int32_t dx_ticks = 0;
int32_t dy_ticks = 0;

/* Raw wheel distances */
float dx_raw = 0.0f;
float dy_raw = 0.0f;

/* Corrected local robot motion */
float dx_local = 0.0f;
float dy_local = 0.0f;

/* Global odometry */
float odom_x = 0.0f;
float odom_y = 0.0f;

/* IMU heading */
float theta_rad = 0.0f;
float prev_theta_rad = 0.0f;
float dtheta = 0.0f;

/* Global delta motion */
float dX = 0.0f;
float dY = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM20_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Calibrate(void);
void MPU6050_Read_All(void);
void Calculate_Angles(void);
void Yaw_Stabilization_Update(void);
void Holonomic_Mix(void);
void Motor_Write(void);
void Update_Odometry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int fmt_float(char *out, float v, int width, int dec2)
{
    /* dec2: number of decimal places (max 2 here) */
    int neg = (v < 0.0f);
    if (neg) v = -v;

    int i_part = (int)v;
    int f_part;
    if (dec2 == 2) f_part = (int)((v - i_part) * 100.0f + 0.5f);
    else           f_part = (int)((v - i_part) * 10.0f  + 0.5f);

    /* handle rounding carry */
    int f_max = (dec2 == 2) ? 100 : 10;
    if (f_part >= f_max) { i_part++; f_part = 0; }

    if (dec2 == 2)
        return snprintf(out, (size_t)(width + 6), "%s%d.%02d", neg ? "-" : "", i_part, f_part);
    else
        return snprintf(out, (size_t)(width + 5), "%s%d.%01d", neg ? "-" : "", i_part, f_part);
}

/* ------------------------------------------------------------------ */
/*  MPU6050 init                                                       */
/* ------------------------------------------------------------------ */
void MPU6050_Init(void)
{
    HAL_StatusTypeDef ret;

    /* Brief delay so USB CDC has time to enumerate before we spam it */
    HAL_Delay(1500);

    int len = snprintf(cdc_buf, sizeof(cdc_buf), "\r\n=== MPU6050 Init ===\r\n");
    CDC_Transmit_FS((uint8_t*)cdc_buf, len);
    HAL_Delay(50);

    ret = HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR, 3, 1000);
    if (ret != HAL_OK)
    {
        len = snprintf(cdc_buf, sizeof(cdc_buf), "ERR: I2C not ready (%d)\r\n", ret);
        CDC_Transmit_FS((uint8_t*)cdc_buf, len);
        return;
    }

    ret = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    len = snprintf(cdc_buf, sizeof(cdc_buf), "WHO_AM_I = 0x%02X  (expect 0x68)  ret=%d\r\n", check, ret);
    CDC_Transmit_FS((uint8_t*)cdc_buf, len);
    HAL_Delay(50);

    if (check != 0x68)
    {
        len = snprintf(cdc_buf, sizeof(cdc_buf), "ERR: wrong WHO_AM_I — check wiring\r\n");
        CDC_Transmit_FS((uint8_t*)cdc_buf, len);
        return;
    }

    /* Wake up */
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG,  1, &Data, 1, 1000);
    /* Sample rate divider → 1 kHz / (7+1) = 125 Hz */
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG,  1, &Data, 1, 1000);
    /* Accel ±2 g */
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
    /* Gyro ±250 °/s */
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG,  1, &Data, 1, 1000);

    imu_last_tick = HAL_GetTick();   /* seed the dt timer here, after config */

    len = snprintf(cdc_buf, sizeof(cdc_buf), "MPU6050 OK\r\n");
    CDC_Transmit_FS((uint8_t*)cdc_buf, len);
    HAL_Delay(50);
}

/* ------------------------------------------------------------------ */
/*  Calibration — collect 500 samples for all 3 gyro axes             */
/*  Keep Delays short; no watchdog here so 500×4 ms = 2 s is safe     */
/* ------------------------------------------------------------------ */
void MPU6050_Calibrate(void)
{
    const int N = 500;
    float sx = 0, sy = 0, sz = 0;
    uint8_t rec[6];

    int len = snprintf(cdc_buf, sizeof(cdc_buf), "Calibrating gyro (%d samples)...\r\n", N);
    CDC_Transmit_FS((uint8_t*)cdc_buf, len);

    for (int i = 0; i < N; i++)
    {
        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec, 6, 100);
        sx += (int16_t)(rec[0] << 8 | rec[1]) / 131.0f;
        sy += (int16_t)(rec[2] << 8 | rec[3]) / 131.0f;
        sz += (int16_t)(rec[4] << 8 | rec[5]) / 131.0f;
        HAL_Delay(4);
    }

    gyro_x_offset = sx / N;
    gyro_y_offset = sy / N;
    gyro_z_offset = sz / N;

    len = snprintf(cdc_buf, sizeof(cdc_buf), "Gyro offsets: Gx=");
    len += fmt_float(cdc_buf + len, gyro_x_offset, 4, 2);
    len += snprintf(cdc_buf + len, sizeof(cdc_buf) - len, " Gy=");
    len += fmt_float(cdc_buf + len, gyro_y_offset, 4, 2);
    len += snprintf(cdc_buf + len, sizeof(cdc_buf) - len, " Gz=");
    len += fmt_float(cdc_buf + len, gyro_z_offset, 4, 2);
    len += snprintf(cdc_buf + len, sizeof(cdc_buf) - len, " deg/s\r\n");
    CDC_Transmit_FS((uint8_t*)cdc_buf, len);
    HAL_Delay(50);

    /* Seed the IMU timer so first dt is clean */
    imu_last_tick = HAL_GetTick();
}

/* ------------------------------------------------------------------ */
/*  Read all sensor data                                               */
/* ------------------------------------------------------------------ */
void MPU6050_Read_All(void)
{
    uint8_t rec[14];
    /* Read accel (6) + temp (2) + gyro (6) in one burst for consistency */
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec, 14, 100);

    Accel_X_RAW = (int16_t)(rec[0]  << 8 | rec[1]);
    Accel_Y_RAW = (int16_t)(rec[2]  << 8 | rec[3]);
    Accel_Z_RAW = (int16_t)(rec[4]  << 8 | rec[5]);
    /* rec[6..7] = temp, skip */
    Gyro_X_RAW  = (int16_t)(rec[8]  << 8 | rec[9]);
    Gyro_Y_RAW  = (int16_t)(rec[10] << 8 | rec[11]);
    Gyro_Z_RAW  = (int16_t)(rec[12] << 8 | rec[13]);

    Ax = Accel_X_RAW / 16384.0f;
    Ay = Accel_Y_RAW / 16384.0f;
    Az = Accel_Z_RAW / 16384.0f;

    /* Apply calibration offsets */
    Gx = (Gyro_X_RAW / 131.0f) - gyro_x_offset;
    Gy = (Gyro_Y_RAW / 131.0f) - gyro_y_offset;
    Gz = (Gyro_Z_RAW / 131.0f) - gyro_z_offset;
}

/* ------------------------------------------------------------------ */
/*  Complementary filter                                               */
/* ------------------------------------------------------------------ */
void Calculate_Angles(void)
{
    uint32_t now = HAL_GetTick();
    imu_dt = (now - imu_last_tick) / 1000.0f;
    imu_last_tick = now;

    /* Guard against first-call spike or stall */
    if (imu_dt <= 0.0f || imu_dt > 0.5f) imu_dt = 0.01f;

    /* Accel angles */
    roll_acc  =  atan2f(Ay, sqrtf(Ax*Ax + Az*Az)) * RAD_TO_DEG;
    pitch_acc = -atan2f(Ax, sqrtf(Ay*Ay + Az*Az)) * RAD_TO_DEG;

    /* Complementary filter — single integration, offsets already removed */
    roll  = CF_ALPHA * (roll  + Gx * imu_dt) + (1.0f - CF_ALPHA) * roll_acc;
    pitch = CF_ALPHA * (pitch + Gy * imu_dt) + (1.0f - CF_ALPHA) * pitch_acc;

    /* Yaw — pure gyro, no accel correction possible */
    yaw_gyro += Gz * imu_dt;
    yaw = yaw_gyro;
}

/* ------------------------------------------------------------------ */
/*  Yaw stabilisation PID (rx_joy is the rotation demand)             */
/* ------------------------------------------------------------------ */
void Yaw_Stabilization_Update(void)
{
    uint32_t now  = HAL_GetTick();
    float    dt_y = (now - prevYawTime) / 1000.0f;
    prevYawTime   = now;
    if (dt_y <= 0.0f || dt_y > 0.5f) dt_y = 0.01f;

    uint8_t rotating    = (abs(rx_joy) > 8);
    uint8_t translating = (abs(lx) > 5 || abs(ly) > 5);

    if (rotating)
    {
        rx_joy = (int)(rx_joy*0.9f);
        targetYaw      = yaw;
        yaw_integral   = 0.0f;
        yaw_prev_error = 0.0f;
    }
    else
    {
        rx_joy = 0;

        if (translating)
        {
            yaw_error = targetYaw - yaw;
            if (yaw_error >  180.0f) yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;

            yaw_integral += yaw_error * dt_y;
            float deriv   = (yaw_error - yaw_prev_error) / dt_y;
            rx_joy        = -(int)(kp*yaw_error + ki*yaw_integral + kd*deriv);
            yaw_prev_error = yaw_error;
        }
        else
        {
            targetYaw      = yaw;
            yaw_integral   = 0.0f;
            yaw_prev_error = 0.0f;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Mecanum kinematics                                                 */
/* ------------------------------------------------------------------ */
void Holonomic_Mix(void)
{
//    mFL =  ly - lx + rx_joy;
//    mFR =  ly + lx - rx_joy;
//    mBL =  ly + lx + rx_joy;
//    mBR =  ly - lx - rx_joy;

    mFL = ly+lx+rx_joy;
    mFR = ly-lx-rx_joy;
    mBL = ly-lx+rx_joy;
    mBR = ly+lx-rx_joy;

    int32_t maxVal = abs(mFL);
    if (abs(mFR) > maxVal) maxVal = abs(mFR);
    if (abs(mBL) > maxVal) maxVal = abs(mBL);
    if (abs(mBR) > maxVal) maxVal = abs(mBR);

    if (maxVal > INPUT_MAX)
    {
        mFL = (mFL * INPUT_MAX) / maxVal;
        mFR = (mFR * INPUT_MAX) / maxVal;
        mBL = (mBL * INPUT_MAX) / maxVal;
        mBR = (mBR * INPUT_MAX) / maxVal;
    }

    dirFL = (mFL >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    dirFR = (mFR >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    dirBL = (mBL >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    dirBR = (mBR >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    pwmFL = (uint16_t)abs(mFL);
    pwmFR = (uint16_t)abs(mFR);
    pwmBL = (uint16_t)abs(mBL);
    pwmBR = (uint16_t)abs(mBR);
}

void Motor_Write(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, dirFL);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  dirFR);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  dirBL);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  dirBR);

    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, pwmFL);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_3, pwmFR);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pwmBL);
    __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_1, pwmBR);
}

void Update_Odometry(void)
{
    /* ---------------- READ ENCODERS ---------------- */

    x_now = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    y_now = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    /* Delta ticks */
    dx_ticks = x_now - x_prev;
    dy_ticks = y_now - y_prev;

    x_prev = x_now;
    y_prev = y_now;

    /* ---------------- TICKS TO DISTANCE ---------------- */

    dx_raw = ((2.0f * PI * R_wheel) / CPR) * dx_ticks;
    dy_raw = ((2.0f * PI * R_wheel) / CPR) * dy_ticks;

    /* ---------------- YAW ---------------- */

    theta_rad = yaw * (PI / 180.0f);

    dtheta = theta_rad - prev_theta_rad;

    prev_theta_rad = theta_rad;

    /* ---------------- YAW COMPENSATION ---------------- */

    dx_local = dx_raw - (x_offset * dtheta);
    dy_local = dy_raw - (y_offset * dtheta);

    /* ---------------- CROSS-AXIS SUPPRESSION ---------------- */

    /* Dominant Y motion */
    if (fabsf(dy_local) > 3.0f * fabsf(dx_local))
    {
        if (fabsf(dx_local) < fabsf(dy_local) * 0.08f)
        {
            dx_local = 0.0f;
        }
    }

    /* Dominant X motion */
    if (fabsf(dx_local) > 3.0f * fabsf(dy_local))
    {
        if (fabsf(dy_local) < fabsf(dx_local) * 0.08f)
        {
            dy_local = 0.0f;
        }
    }

    /* ---------------- LOCAL TO GLOBAL ---------------- */

    dX = (dx_local * cosf(theta_rad))
       - (dy_local * sinf(theta_rad));

    dY = (dx_local * sinf(theta_rad))
       + (dy_local * cosf(theta_rad));

    /* ---------------- GLOBAL POSITION ---------------- */

    odom_x += dX;
    odom_y += dY;
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
  MX_USART1_UART_Init();
  MX_USB_Device_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM17_Init();
  MX_TIM20_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  HAL_UART_Receive_IT(&huart4, (uint8_t*)&Rx_buff[rx_index], 1);

  MPU6050_Init();
  MPU6050_Calibrate();

  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);

  prevYawTime = HAL_GetTick();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  /* --- IMU --- */

      MPU6050_Read_All();
      Calculate_Angles();
      Update_Odometry();

      /* --- Yaw PID --- */


      /* --- LIDAR 1 --- */
      if (tf1_ready)
      {
          tf1_ready = 0;
          m_int1  = distance1_mm / 1000;
          m_frac1 = distance1_mm % 1000;
      }

      /* --- LIDAR 2 --- */
      if (tf2_ready)
      {
          tf2_ready = 0;
          m_int2  = distance2_mm / 1000;
          m_frac2 = distance2_mm % 1000;
      }

      /* --- Joystick / motor --- */
      if (uart_line_ready)
      {
          uart_line_ready = 0;
          if (sscanf(Rx_buff, "%d,%d,%d,%d,%d,%d,%d,%d",
                     &lx, &ly, &rx_joy, &ry, &l2, &r2, &l1, &r1) == 8)
          {
        	  Yaw_Stabilization_Update();
              Holonomic_Mix();
              Motor_Write();

          }
      }

      /* --- CDC debug print @ 10 Hz --- */
      static uint32_t dbgTick = 0;
      if (HAL_GetTick() - dbgTick >= 100)
      {
          dbgTick = HAL_GetTick();

          /*
           * Output format (fixed-width columns, easy to parse or watch on terminal):
           *
           * D1:  1.234m  D2:  0.876m | LX: -123  LY:  456  RX:   12 | R: -12.34  P:   5.67  Y: 123.45
           *
           * Floats are formatted without %f to be safe on all ARM-GCC configs.
           */

          /* Build roll/pitch/yaw strings */
          char s_roll[10], s_pitch[10], s_yaw[10];
          fmt_float(s_roll,  roll,  5, 2);
          fmt_float(s_pitch, pitch, 5, 2);
          fmt_float(s_yaw,   yaw,   6, 2);
          int len = snprintf(cdc_buf, sizeof(cdc_buf),
          "D1:%2u.%03um D2:%2u.%03um | LX:%5d LY:%5d RX:%5d | R:%s P:%s Y:%s | OX:%0.2f OY:%0.2f\r\n",
          m_int1, m_frac1,
          m_int2, m_frac2,
          lx, ly, rx_joy,
          s_roll, s_pitch, s_yaw,
          odom_x, odom_y);
          CDC_Transmit_FS((uint8_t*)cdc_buf, (uint16_t)len);
      }

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 500;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 500;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 8;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 500;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB11 PB4 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* ---- UART4: joystick data ---- */
    if (huart->Instance == UART4)
    {
        uint8_t b = Rx_buff[rx_index];

        if (b == '\n' || b == '\r')
        {
            Rx_buff[rx_index] = '\0';
            rx_index = 0;
            uart_line_ready = 1;
        }
        else
        {
            if (rx_index < sizeof(Rx_buff) - 2)
                rx_index++;
            else
                rx_index = 0;   /* overflow guard */
        }
        HAL_UART_Receive_IT(&huart4, (uint8_t*)&Rx_buff[rx_index], 1);
    }

    /* ---- USART1: TFMini LIDAR 1 ---- */
    else if (huart->Instance == USART1)
    {
        static uint8_t state1 = 0;
        switch (state1)
        {
            case 0:
                if (rx1_byte == 0x59) { tf1_frame[0] = rx1_byte; state1 = 1; }
                break;
            case 1:
                if (rx1_byte == 0x59) { tf1_frame[1] = rx1_byte; tf1_index = 2; state1 = 2; }
                else                  { state1 = 0; }
                break;
            case 2:
                tf1_frame[tf1_index++] = rx1_byte;
                if (tf1_index >= 9)
                {
                    uint8_t sum = 0;
                    for (int i = 0; i < 8; i++) sum += tf1_frame[i];
                    if (sum == tf1_frame[8])
                    {
                        distance1_mm = (uint16_t)(tf1_frame[2] | (tf1_frame[3] << 8)) * 10;
                        tf1_ready = 1;
                    }
                    state1 = 0;
                }
                break;
        }
        HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
    }

    /* ---- USART2: TFMini LIDAR 2 ---- */
    else if (huart->Instance == USART2)
    {
        static uint8_t state2 = 0;
        switch (state2)
        {
            case 0:
                if (rx2_byte == 0x59) { tf2_frame[0] = rx2_byte; state2 = 1; }
                break;
            case 1:
                if (rx2_byte == 0x59) { tf2_frame[1] = rx2_byte; tf2_index = 2; state2 = 2; }
                else                  { state2 = 0; }
                break;
            case 2:
                tf2_frame[tf2_index++] = rx2_byte;
                if (tf2_index >= 9)
                {
                    uint8_t sum = 0;
                    for (int i = 0; i < 8; i++) sum += tf2_frame[i];
                    if (sum == tf2_frame[8])
                    {
                        distance2_mm = (uint16_t)(tf2_frame[2] | (tf2_frame[3] << 8)) * 10;
                        tf2_ready = 1;
                    }
                    state2 = 0;
                }
                break;
        }
        HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
    }
}
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
#ifdef USE_FULL_ASSERT
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
