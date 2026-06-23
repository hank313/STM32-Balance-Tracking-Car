# 🚗 STM32 Balance Tracking Car

![MCU](https://img.shields.io/badge/MCU-STM32L452RETx-blue)
![Sensor](https://img.shields.io/badge/Sensor-MPU6050-green)
![Motor Driver](https://img.shields.io/badge/Driver-TB6612FNG-orange)
![IDE](https://img.shields.io/badge/IDE-Keil%20uVision-purple)
![Tool](https://img.shields.io/badge/Tool-STM32CubeMX-lightgrey)

STM32 自平衡循跡車控制程式。

本專案使用 **STM32CubeMX** 建立初始化設定，並使用 **Keil uVision** 撰寫與編譯主要控制程式。

---

## 📌 專案簡介

本專案為一台基於 STM32L452RETx 的自平衡循跡車。

系統會透過 **MPU6050 六軸感測器** 取得車體姿態資料，並使用 **PID 控制器** 計算馬達輸出，使車體維持平衡。

此外，本專案加入左右兩組 **TCRT5000 循跡感測器**，可偵測地面黑白線或障礙物邊緣，並修正車體行進方向。

---

## ✨ 主要功能

| 功能   | 說明                        |
| ---- | ------------------------- |
| 姿態感測 | 使用 MPU6050 讀取加速度與陀螺儀資料    |
| 角度估測 | 使用互補濾波器估測車體傾斜角            |
| 平衡控制 | 使用 PID 控制器計算馬達 PWM 輸出     |
| 馬達控制 | 使用 TB6612FNG 控制左右兩顆 DC 馬達 |
| 循跡控制 | 使用左右 TCRT5000 感測器修正行進方向   |
| 安全保護 | 車體傾斜超過 ±45° 時停止馬達輸出       |

---

## 🛠️ 開發環境

| 項目     | 使用工具              |
| ------ | ----------------- |
| MCU 設定 | STM32CubeMX       |
| 程式開發   | Keil uVision      |
| 程式語言   | C Language        |
| 函式庫    | STM32 HAL Library |
| 版本控制   | Git / GitHub      |

---

## 🔧 使用硬體

| 硬體            | 用途          |
| ------------- | ----------- |
| STM32L452RETx | 主控制器        |
| MPU6050       | 讀取車體角度與角速度  |
| TB6612FNG     | 雙通道直流馬達驅動模組 |
| DC 馬達 x2      | 左右輪動力輸出     |
| TCRT5000 x2   | 左右循跡 / 邊線偵測 |
| MB102 電源模組    | 麵包板電源分配     |
| 18650 電池盒     | 馬達與系統供電     |

---

## 📂 專案結構

```text
STM32-Balance-Tracking-Car/
│
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── gpio.h
│   │   ├── i2c.h
│   │   ├── tim.h
│   │   └── usart.h
│   │
│   └── Src/
│       ├── main.c          # 主要控制程式
│       ├── gpio.c
│       ├── i2c.c
│       ├── tim.c
│       └── usart.c
│
├── Drivers/                # STM32 HAL 與 CMSIS 驅動
├── MDK-ARM/                # Keil uVision 專案設定
├── RTE/
├── Motor_test.ioc          # STM32CubeMX 設定檔
├── Motor_test.uvprojx      # Keil uVision 專案檔
├── README.md
└── .gitignore
```

---

## 🔌 STM32L452RETx 全局腳位配置表

本表依照 `STM32_Balance_Robot_Hardware_Blueprint(1)` 整理。

| STM32 腳位 | CubeMX 標籤 / 功能 | 連接目標           | 功能描述          |
| -------- | -------------- | -------------- | ------------- |
| PA0      | GPIO_Output    | TB6612FNG AIN1 | 左輪方向控制 1      |
| PA1      | GPIO_Output    | TB6612FNG AIN2 | 左輪方向控制 2      |
| PA5      | GPIO_Output    | TB6612FNG STBY | 馬達驅動模組致能      |
| PA6      | TIM3_CH1       | TB6612FNG PWMA | 左輪速度控制 PWM    |
| PA9      | I2C1_SCL       | MPU6050 SCL    | 姿態感測器 I2C 時脈線 |
| PA10     | I2C1_SDA       | MPU6050 SDA    | 姿態感測器 I2C 資料線 |
| PA15     | TIM2_CH1       | TB6612FNG PWMB | 右輪速度控制 PWM    |
| PB0      | GPIO_Output    | TB6612FNG BIN1 | 右輪方向控制 1      |
| PB1      | GPIO_Output    | TB6612FNG BIN2 | 右輪方向控制 2      |
| PC2      | TRACK_LEFT     | 左 TCRT5000 OUT | 左紅外線輸入        |
| PC3      | TRACK_RIGHT    | 右 TCRT5000 OUT | 右紅外線輸入        |

---

## 🔌 MPU6050 姿態感測器接線

MPU6050 使用 I2C 介面與 STM32 通訊，負責讀取加速度與陀螺儀資料。

| MPU6050 腳位 | STM32 腳位  | CubeMX 功能 | 說明       |
| ---------- | --------- | --------- | -------- |
| VCC        | 5V / 3.3V | Power     | 感測器供電    |
| GND        | GND       | Ground    | 系統接地     |
| SCL        | PA9       | I2C1_SCL  | I2C 時脈線  |
| SDA        | PA10      | I2C1_SDA  | I2C 資料線  |
| INT        | 不接        | None      | 本程式未使用中斷 |

---

## 🔌 TB6612FNG 馬達驅動控制訊號接線

TB6612FNG 用來控制左右兩顆 DC 馬達的轉速與正反轉。

### 全域控制腳位

| TB6612FNG 腳位 | STM32 腳位 | CubeMX 功能   | 說明           |
| ------------ | -------- | ----------- | ------------ |
| STBY         | PA5      | GPIO_Output | 馬達驅動致能，高電位啟用 |

### 左輪控制區塊

| TB6612FNG 腳位 | STM32 腳位 | CubeMX 功能   | 說明          |
| ------------ | -------- | ----------- | ----------- |
| PWMA         | PA15      | TIM3_CH1    | 左輪 PWM 速度控制 |
| AIN1         | PA0      | GPIO_Output | 左輪方向控制 1    |
| AIN2         | PA1      | GPIO_Output | 左輪方向控制 2    |

### 右輪控制區塊

| TB6612FNG 腳位 | STM32 腳位 | CubeMX 功能   | 說明          |
| ------------ | -------- | ----------- | ----------- |
| PWMB         | PA6      | TIM2_CH1     | 右輪 PWM 速度控制 |
| BIN1         | PB0      | GPIO_Output | 右輪方向控制 1    |
| BIN2         | PB1      | GPIO_Output | 右輪方向控制 2    |

---

## 🔌 TB6612FNG 電源與馬達輸出接線

| TB6612FNG 腳位 | 連接位置      | 說明                   |
| ------------ | --------- | -------------------- |
| VM           | 5V        | 馬達獨立大電源，禁止直接接入 STM32 |
| VCC          | 3V        | TB6612FNG 邏輯供電       |
| GND          | 系統 GND    | 必須與 STM32、電池、感測器共地   |
| AO1          | 左馬達其中一端   | 左側實體馬達線              |
| AO2          | 左馬達另一端    | 左側實體馬達線              |
| BO1          | 右馬達其中一端   | 右側實體馬達線              |
| BO2          | 右馬達另一端    | 右側實體馬達線              |

> 注意：`VM` 是馬達用的大電源，接電池正極，例如 7.4V。
> 不可以把 `VM` 直接接到 STM32 的 5V 或 3.3V。

---

## 🔌 TCRT5000 循跡感測器接線

TCRT5000 用來偵測黑白線或障礙物邊線，輸出 Digital Input 給 STM32 判斷。

### 左側 TCRT5000

| TCRT5000 腳位 | STM32 腳位  | CubeMX 標籤  | 說明       |
| ----------- | --------- | ---------- | -------- |
| VCC         | 5V / 3.3V | Power      | 感測器供電    |
| GND         | GND       | Ground     | 系統接地     |
| OUT         | PC2       | TRACK_LEFT | 左側循跡訊號輸入 |

### 右側 TCRT5000

| TCRT5000 腳位 | STM32 腳位  | CubeMX 標籤   | 說明       |
| ----------- | --------- | ----------- | -------- |
| VCC         | 5V / 3.3V | Power       | 感測器供電    |
| GND         | GND       | Ground      | 系統接地     |
| OUT         | PC3       | TRACK_RIGHT | 右側循跡訊號輸入 |

---

## 🔋 電源分配

| 模組 / 位置       | 供電來源      | 說明                         |
| ------------- | --------- | -------------------------- |
| STM32 開發板     | USB / 5V  | 控制器供電                      |
| MPU6050       | 3.3V 或 5V | 依模組規格選擇供電                  |
| TCRT5000 x2   | 3.3V 或 5V | 依感測器模組規格選擇供電               |
| TB6612FNG VCC | 5V        | 馬達驅動邏輯電源                   |
| TB6612FNG VM  | 電池正極 7.4V | 馬達動力電源                     |
| 全系統 GND       | 系統共地      | STM32、感測器、TB6612FNG、電池必須共地 |

---

## 🧠 控制流程

```text
啟動系統
   ↓
初始化 GPIO / I2C / TIM / USART
   ↓
啟動 TIM2、TIM3 PWM
   ↓
初始化 MPU6050
   ↓
讀取 MPU6050 原始資料
   ↓
計算車體角度
   ↓
互補濾波器修正角度
   ↓
PID 計算平衡 PWM
   ↓
讀取左右 TCRT5000 循跡感測器
   ↓
修正左右馬達 PWM
   ↓
輸出 PWM 控制 TB6612FNG
   ↓
驅動左右 DC 馬達
```

---

## ⚙️ 主要控制參數

### PID 平衡控制參數

```c
float Kp = 35.0;
float Ki = 0.0;
float Kd = 2.2;
```

| 參數 | 說明             |
| -- | -------------- |
| Kp | 比例控制，影響車體回正速度  |
| Ki | 積分控制，用於修正長時間誤差 |
| Kd | 微分控制，用於抑制震盪    |

---

### 循跡控制參數

```c
int16_t Forward_Speed = 200;
float K_Track = 200.0;
```

| 參數            | 說明       |
| ------------- | -------- |
| Forward_Speed | 車體前進基礎速度 |
| K_Track       | 循跡轉向修正量  |

---

### PWM 輸出限制

```c
if (pwm > 850)  return 850;
if (pwm < -850) return -850;
```

PWM 輸出限制在 `-850 ~ 850`，避免馬達輸出過大造成車體失控。

---

## 🧩 程式功能說明

### 1. MPU6050 初始化

程式會透過 I2C 讀取 MPU6050 的 `WHO_AM_I` 暫存器，確認感測器是否正常連接。

若讀取值正確，程式會解除睡眠模式並設定取樣頻率。

---

### 2. 姿態角度估測

程式使用加速度計計算角度：

```c
accel_angle = atan2((float)raw_ax, (float)raw_az) * 57.29578;
```

再使用陀螺儀角速度與互補濾波器修正角度：

```c
Current_Angle = 0.98 * (Current_Angle + gyro_rate * 0.005) + 0.02 * accel_angle;
```

---

### 3. PID 平衡控制

PID 控制器會根據目前角度與目標角度之間的誤差，計算馬達 PWM 輸出：

```c
float pwm_out = (Kp * Error) + (Ki * Integrated_Error) + (Kd * Gyro);
```

---

### 4. 循跡控制

程式會讀取左右 TCRT5000 感測器狀態：

```c
uint8_t Left_Sensor  = HAL_GPIO_ReadPin(TRACK_LEFT_GPIO_Port, TRACK_LEFT_Pin);
uint8_t Right_Sensor = HAL_GPIO_ReadPin(TRACK_RIGHT_GPIO_Port, TRACK_RIGHT_Pin);
```

並依照感測器狀態決定轉向修正量。

| 左感測器 | 右感測器 | 動作     |
| ---- | ---- | ------ |
| 0    | 1    | 向一側修正  |
| 1    | 0    | 向另一側修正 |
| 其他   | 其他   | 直行     |

---

### 5. 安全保護

若車體傾斜角度超過 ±45°，系統會停止馬達輸出：

```c
if (Current_Angle > 45.0 || Current_Angle < -45.0)
```

此設計可避免車體倒下後馬達仍持續高速轉動。

---

## ▶️ 如何使用

1. 使用 **STM32CubeMX** 開啟 `Motor_test.ioc`
2. 檢查 GPIO、I2C、TIM PWM 腳位設定
3. 使用 **Keil uVision** 開啟 `Motor_test.uvprojx`
4. 編譯專案
5. 將程式燒錄到 STM32 開發板
6. 依照本 README 的接線表連接 MPU6050、TB6612FNG、TCRT5000 與馬達
7. 啟動系統並測試自平衡與循跡功能

---

## ⚠️ 注意事項

* 測試時建議先將車體架高，確認馬達方向正確後再落地測試。
* MPU6050 的方向會影響角度正負號，若車體反應相反，需要調整感測器方向或程式符號。
* 左右馬達轉速若不一致，可調整馬達比例補償。
* PID 參數需要依照車體重心、馬達轉速、輪徑與電池電壓進行微調。
* TB6612FNG 的 `VM` 為馬達大電源，禁止直接接入 STM32。
* 馬達電源、STM32、MPU6050、TCRT5000、TB6612FNG 必須共地。

---

## 📷 系統示意圖

```text
        ┌────────────────────────────┐
        │        STM32L452RETx        │
        │                            │
        │  PA9  ───── I2C1_SCL ───── MPU6050 SCL
        │  PA10 ───── I2C1_SDA ───── MPU6050 SDA
        │                            │
        │  PA6  ───── TIM3_CH1 ───── TB6612FNG PWMA
        │  PA0  ───── GPIO     ───── TB6612FNG AIN1
        │  PA1  ───── GPIO     ───── TB6612FNG AIN2
        │                            │
        │  PA15 ───── TIM2_CH1 ───── TB6612FNG PWMB
        │  PB0  ───── GPIO     ───── TB6612FNG BIN1
        │  PB1  ───── GPIO     ───── TB6612FNG BIN2
        │                            │
        │  PC2  ───── TRACK_LEFT ─── 左 TCRT5000 OUT
        │  PC3  ───── TRACK_RIGHT ── 右 TCRT5000 OUT
        └────────────────────────────┘
```

---

## 📄 授權

本專案為課程作業與學習用途。
