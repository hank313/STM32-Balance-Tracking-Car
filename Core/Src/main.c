/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body (Synchronized Dual Motor Forward Tracking)

  ******************************************************************************

  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#include "i2c.h"

#include "tim.h"

#include "usart.h"

#include "gpio.h"



/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <math.h>  // 引入數學庫以使用 atan2 計算角度

/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */



/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

#define MPU6050_ADDR         (0x68 << 1) // MPU6050 I2C 七位元地址，需左移一位

#define SMPLRT_DIV_REG       0x19

#define PWR_MGMT_1_REG       0x6B

#define WHO_AM_I_REG         0x75

#define ACCEL_XOUT_H_REG     0x3B

#define GYRO_XOUT_H_REG      0x43

/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */



/* USER CODE END PM */



/* Private variables ---------------------------------------------------------*/



/* USER CODE BEGIN PV */

// MPU6050 原始數據與轉換變數

int16_t raw_ax, raw_az, raw_gy;

float accel_angle, gyro_rate;

float Current_Angle = 0.0; // 融合後的當前車身角度

float Gyro_Y = 0.0;        // Y 軸陀螺儀角速度



// =================================================================

// ??? PID 參數調試戰場 (平衡反應速度與煞車阻尼)

// =================================================================

float Kp = 22.0;   // 比例常數：拉高此值提升肌肉力量，讓小車反應變快

float Ki = 0.0;    // 積分常數：固定保持 0 即可

float Kd = 1.2;    // 微分常數：提供阻尼作為煞車，抑制前後劇烈搖晃



float Target_Angle = 0.0; // 目標直直角度 (理想是 0 度)

float Error = 0.0;

float Integrated_Error = 0.0;

int16_t Motor_PWM = 0;



// =================================================================

// ?? 循跡與強行前進參數區

// =================================================================

int16_t Forward_Speed = 200; // ??【前進核心】設定基礎前進速度

                             // 配合下方死區補償，200 就能跑得很順。太快可調小，太慢可調大。



float K_Track = 200.0;  // 轉彎修正量。從 150 提高到 200，讓小車過彎甩頭更快、更暴力

int16_t Turn_Value = 0; // 轉彎疊加值



// ??? ??? ??? 【已修正】將左輪力量補償係數改回 1.0，讓左右雙輪輸出完全均等 ??? ??? ???

float Left_Motor_Scale = 1.00; 

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */



/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

// MPU6050 初始化

uint8_t MPU6050_Init(void) {

    uint8_t check, data;

    // 讀取 WHO_AM_I 暫存器，預設應回傳 0x68

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) {

        data = 0; // 喚醒感測器 (將電源管理暫存器清零)

        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        data = 0x07; // 設定取樣率為 1KHz

        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        return 0; // 初始化成功

    }

    return 1; // 初始化失敗

}



// 連續讀取感測器資料

void MPU6050_Read_Raw(int16_t *Accel_X, int16_t *Accel_Z, int16_t *Gyro_Y) {

    uint8_t Rec_Data[14];

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    *Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    *Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *Gyro_Y  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);

}



// PWM 限幅防呆 (解開限制至 850，給予前進平衡足夠的瞬間扭力)

int16_t PWM_Limit(int16_t pwm) {

    if (pwm > 850)  return 850;

    if (pwm < -850) return -850;

    return pwm;

}



// 直立平衡 PID 運算

int16_t Balance_PID(float Angle, float Gyro) {

    Error = Angle - Target_Angle;

    Integrated_Error += Error;

    

    // 限制積分上限，防積分飽和

    if (Integrated_Error > 2000)  Integrated_Error = 2000;

    if (Integrated_Error < -2000) Integrated_Error = -2000;



    // 微分項直接採用陀螺儀量到的角速度 Gyro

    float pwm_out = (Kp * Error) + (Ki * Integrated_Error) + (Kd * Gyro);

    return (int16_t)pwm_out;

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

  MX_TIM2_Init();

  MX_USART2_UART_Init();

  MX_TIM3_Init();

  MX_I2C1_Init();

  

  /* USER CODE BEGIN 2 */

  // 1. 啟動 TIM2 Channel 1 的 PWM 訊號 (控制馬達 A - 實體右輪)

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);



  // 2. 啟動 TIM3 Channel 1 的 PWM 訊號 (控制馬達 B - 實體左輪)

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);



  // 3. 將 STBY 腳位拉高，解除驅動板待機模式

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);



  // 4. 初始化 MPU6050 陀螺儀

  MPU6050_Init();

  /* USER CODE END 2 */



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

    // 1. 讀取感測器原始數值

    MPU6050_Read_Raw(&raw_ax, &raw_az, &raw_gy);



    // 2. 資料物理轉換與互補濾波 (Complementary Filter)

    accel_angle = atan2((float)raw_ax, (float)raw_az) * 57.29578; // 算出加速度計夾角 (180/PI)

    gyro_rate = (float)raw_gy / 131.0;                            // 轉換為度/秒 (預設量程)

    

    // 互補濾波公式：dt 設為固定控制週期 0.005 秒 (5 毫秒)

    Current_Angle = 0.98 * (Current_Angle + gyro_rate * 0.005) + 0.02 * accel_angle;

    Gyro_Y = gyro_rate; 



    // 3. 讀取紅外線循跡狀態 (通常 0 代表偵測到黑線，1 代表白底地板)

    uint8_t Left_Sensor  = HAL_GPIO_ReadPin(TRACK_LEFT_GPIO_Port, TRACK_LEFT_Pin);

    uint8_t Right_Sensor = HAL_GPIO_ReadPin(TRACK_RIGHT_GPIO_Port, TRACK_RIGHT_Pin);



    // 4. ??? 循跡轉彎邏輯（維持原本實測完全正確的原始方向）

    if (Left_Sensor == 0 && Right_Sensor == 1) {

        // 左側紅外線壓到黑線時 -> 執行「往左修正」

        Turn_Value = K_Track;  

    } 

    else if (Left_Sensor == 1 && Right_Sensor == 0) {

        // 右側紅外線壓到黑線時 -> 執行「往右修正」

        Turn_Value = -K_Track;

    } 

    else {

        // 兩邊都在白底上 -> 維持直行

        Turn_Value = 0;

    }



    // 5. 安全斷電機制與雙輪輸出分配

    if (Current_Angle > 45.0 || Current_Angle < -45.0) {

        // 倒了就斷電，強制停止馬達

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

        Integrated_Error = 0; // 重置積分誤差

    } else {

        // 計算基礎平衡 PID 輸出值

        Motor_PWM = Balance_PID(Current_Angle, Gyro_Y);

        

        // 融合平衡、前進速度與轉彎修正

        int16_t Left_Motor_PWM  = Motor_PWM - Forward_Speed + Turn_Value;  // 實體右輪 (TIM2)

        int16_t Right_Motor_PWM = Motor_PWM - Forward_Speed - Turn_Value;  // 實體左輪 (TIM3)



        // ??? ??? ??? 【核心修改】Left_Motor_Scale 已設為 1.0，此處左右輸出維持完全對稱 ??? ??? ???

        Right_Motor_PWM = (int16_t)(Right_Motor_PWM * Left_Motor_Scale);



        // 獨立對左右輪進行安全限幅

        Left_Motor_PWM  = PWM_Limit(Left_Motor_PWM);

        Right_Motor_PWM = PWM_Limit(Right_Motor_PWM);



        // ??【硬體死區補償與強制換向機制】

        // ----------------- 驅動右輪馬達 (TIM2) -----------------

        if (Left_Motor_PWM >= 0) {

            int16_t final_pwm = Left_Motor_PWM + 80; 

            if(final_pwm > 850) final_pwm = 850;

            

            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, final_pwm);

        } else {

            int16_t final_pwm = -Left_Motor_PWM + 80;

            if(final_pwm > 850) final_pwm = 850;

            

            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, final_pwm);

        }



        // ----------------- 驅動左輪馬達 (TIM3) -----------------

        if (Right_Motor_PWM >= 0) {

            int16_t final_pwm = Right_Motor_PWM + 80;

            if(final_pwm > 850) final_pwm = 850;

            

            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_pwm);

        } else {

            int16_t final_pwm = -Right_Motor_PWM + 80;

            if(final_pwm > 850) final_pwm = 850;

            

            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_pwm);

        }

    }



    HAL_Delay(5); // 維持大約 5ms (200Hz) 的平衡控制頻率

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



  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)

  {

    Error_Handler();

  }



  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = 64;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  RCC_OscInitStruct.PLL.PLLM = 1;

  RCC_OscInitStruct.PLL.PLLN = 10;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;

  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;

  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

  {

    Error_Handler();

  }



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

  __disable_irq();

  while (1)

  {

  }

  /* USER CODE END Error_Handler_Debug */

}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)

{

}

#endif /* USE_FULL_ASSERT */

