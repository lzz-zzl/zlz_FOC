# 天空星 STM32F407VET6 引脚分配 v2（含冲突检查 + 排针映射）

> **基于实际代码验证** — 所有引脚数据来自 `tim.c / adc.c / spi.c / i2c.c / usart.c / gpio.c / DRV8313.c / check_I.c`  
> 硬件：**立创天空星 LCKFB-SkyStar STM32F407VET6**  
> 日期：2026-06-10 | 本文档对应工程文件：`ITCAP_STM.ioc`

---

## 目录

1. [当前引脚占用（来自代码）](#1-当前引脚占用来自代码)
2. [已发现的冲突](#2-已发现的冲突)
3. [组件表勘误](#3-组件表勘误)
4. [外设分配方案](#4-外设分配方案)
5. [P1/P2 排针物理映射](#5-p1p2-排针物理映射)
6. [备用接口汇总](#6-备用接口汇总)
7. [必须处理的问题](#7-必须处理的问题)

---

## 1. 当前引脚占用（来自代码）

以下是 **从实际 CubeMX 生成代码中提取** 的配置，分为"已用"和"预留"两类。

### 1.1 已激活的功能

| 功能 | 引脚 | 模式/外设 | 所在代码文件 |
|------|------|----------|------------|
| **FOC 电机三相 PWM** | PE9, PE11, PE13 | TIM1_CH1/CH2/CH3 (AF1) | `tim.c` |
| **编码器 1** | PC6, PC7 | TIM3_CH1/CH2 (AF2) | `tim.c` |
| **编码器 2** | PD12, PD13 | TIM4_CH1/CH2 (AF2) | `tim.c` |
| **15kHz ADC 触发** | PA0 | TIM5_CH1 OC1REF (AF2) | `check_I.c` |
| **MT6816 磁编** | PA5(SCK), PB4(MISO), PB5(MOSI) | SPI1 (AF5) | `spi.c` |
| **MT6816 CS** | PA4 | GPIO_OUT_PP | `main.c` |
| **OLED (I2C1)** | PB6(SCL), PB7(SDA) | I2C1 (AF4) | `i2c.c` |
| **调试串口 USART1** | PA9(TX), PA10(RX) | USART1 (AF7) | `usart.c` |
| **电流采样 ADC1** | PA6(IN6), PA7(IN7) + PB0(IN8), PB1(IN9) | ADC1 analog | `adc.c`, `check_I.c` |
| **指示灯** | PB2 | GPIO_OUT_PP | `gpio.c` |
| **未知用途** | PC4, PB11 | GPIO_OUT_PP | `gpio.c` |

### 1.2 已配置但未使用的预留

| 外设 | 引脚 | 当前状态 | 所在文件 |
|------|------|---------|---------|
| **TIM2** (电机2 FOC) | PA0(CH1), PA1(CH2), PA2(CH3) | ⚠️ 与 DRV8313 + check_I.c 冲突, 实际不可用 | `tim.c` |
| **SPI2** | PB10(SCK), PC2(MISO), PC3(MOSI) | 已配未用, 可回收 | `spi.c` |
| **I2C3** | PA8(SCL), PC9(SDA) | 已配未用, 可回收 | `i2c.c` |
| **USART2** | PA3(RX), PD5(TX) | 已配未用, 可回收 | `usart.c` |

### 1.3 DRV8313 驱动芯片引脚（当前未完全配置）

| 功能 | 引脚 | 问题 |
|------|------|------|
| **DRV8313_EN** | PB15 | ❌ `gpio.c` 未配, `DRV8313_Init()` 被注释 — **实际悬空** |
| **DRV8313_FAULT** | PA11 | ❌ 同上 — **实际悬空** |
| **DRV8313_RESET** | PA1 | ❌ `check_I.c` 将其配为 ADC1_IN9 (analog) — **与 GPIO 冲突** |
| **DRV8313_SLEEP** | PA2 | ❌ `gpio.c` 未配 — **实际悬空** |

---

## 2. 已发现的冲突

### 🔴 严重冲突

| 冲突 | 引脚 | 竞争者 | 后果 |
|------|------|--------|------|
| **C1** | **PA0** | TIM2_CH1 (AF1) vs TIM5_CH1 (AF2) vs 板载KEY | `check_I.c` 取走, KEY+TIM2 不可用 |
| **C2** | **PA1** | TIM2_CH2 (AF1) vs ADC1_IN2 vs DRV8313_RESET | `check_I.c` 配为 analog, GPIO+TIM2 不可用 |
| **C3** | **PA2** | TIM2_CH3 (AF1) vs DRV8313_SLEEP | `gpio.c` 全没配, **悬空** |

### 🟡 潜在问题

| 问题 | 说明 |
|------|------|
| **PB15 (EN)** 未配置 | 若 DRV8313 EN 内部下拉, 驱动永远不使能 |
| **PA11 (FAULT)** 未配置 | 无法读取驱动故障 |
| **PC4 / PB11** 用途不明 | 在 `gpio.c` 配为 OUTPUT, 但项目代码中未引用 |

### ✅ 已确认无冲突的正常分配

PE9/PE11/PE13, PC6/PC7, PD12/PD13, PA4/PA5, PB4/PB5/PB6/PB7, PA6/PA7, PB0/PB1, PA9/PA10, PB2

---

## 3. 组件表勘误

针对你提供的表格逐项评审：

### 原表 → 修正

| # | 原表字段 | 勘误修正 | 原因 |
|---|---------|---------|------|
| 1 | **驱动FOC: PWM x6, ADC x4** | → **PWM x3 + 3N (互补), ADC x2~4** | FOC 需要 3 对互补 PWM (6路), 但电流采样最少只需 2 相 (第3相可算), 4 通道用于双电机 |
| 2 | **磁编MT6816: SPI x4, 数量 2** | → **SPI x3 (共享) + CS x2, 共 5 引脚** | 两个 MT6816 可共享 SPI 总线, 不同 CS 片选, 省 3 个引脚 |
| 3 | **陀螺仪: I2C x2, 数量 2** | → **I2C x1 总线, 挂 2 个设备** | I2C 是多地址总线, 两个同型号陀螺仪需不同 AD0 地址 |
| 4 | **OLED: I2C x2, 数量 1** | → **I2C x1 (与陀螺仪共享)** | OLED 与陀螺仪同挂 I2C1, 仅用 2 个引脚 |
| 5 | **驱动电机: PWMx1 outputx2, 数量 2** | → **PWM x2 + GPIO x4, 共 6 引脚** | 每个电机 1 路 PWM + 2 路方向控制 |
| 6 | **串口: uart x2, 数量 2** | → 明确: **1 路调试口 + 1 路外设通信** | USART1 已用 (PA9/PA10), 再开 USART2 或 USART3 |
| 7 | **按钮: (空)** | → **1 个, 板载 KEY (PA0)** | 板载 PA0 可作为按钮, 但需注意 PA0 已配为 TIM5_CH1 |
| 8 | **缺少: FOC DRV8313 控制线** | → **EN + FAULT + 可选 nSleep** | 至少需要 EN 和 FAULT 两个引脚 |

### 修正后的资源需求总计

| 组件 | 接口类型 | 引脚数 | 说明 |
|------|---------|:------:|------|
| **驱动FOC** (×1) | TIM1 CH1/CH2/CH3 + ADC1 | **5** | PE9/PE11/PE13 + PA6/PA7 |
| **磁编MT6816** (×2) | SPI1 (共享) + CS×2 | **5** | PA5/PB4/PB5 + PA4/新CS |
| **寻迹** (×1) | GPIO Input ×8 | **8** | 8路 数字量输入 |
| **陀螺仪** (×2) | I2C1 (共享) | **2** | PB6/PB7, 与 OLED 共享 |
| **OLED** (×1) | I2C1 (共享) | **0** (已计) | 同上总线 |
| **串口** (×2) | USART1 + USART3 | **4** | PA9/PA10 + PB10/PB11 |
| **驱动电机** (×2) | TIM2_CH1/CH2 + GPIO×4 | **6** | 2路PWM + 4路方向 |
| **霍编** (×2) | TIM3_CH1/CH2 + TIM4_CH1/CH2 | **4** | ✅ 已配 (PC6/PC7 + PD12/PD13) |
| **按钮** (×1) | GPIO Input | **1** | 板载 PA0 或额外 |
| **FOC 驱动控制** | GPIO (EN + FAULT) | **2** | PB15 + PA11 (需重新配置) |
| **总计** | | **~37** | |

> 当前可用 GPIO 约 **70 个**, 足够分配。

---

## 4. 外设分配方案

### 4.1 驱动FOC — 不变 (5 引脚)

> ✅ **已实现, 无需更改**

```text
功能       引脚     外设          备注
FOC_UH     PE9     TIM1_CH1     三相上桥 U
FOC_UL     PE8     TIM1_CH1N    (若需互补)
FOC_VH     PE11    TIM1_CH2     三相上桥 V
FOC_VL     PE10    TIM1_CH2N    (若需互补)
FOC_WH     PE13    TIM1_CH3     三相上桥 W
FOC_WL     PE12    TIM1_CH3N    (若需互补)
ADC_I_A    PA6     ADC1_IN6     U 相电流
ADC_I_B    PA7     ADC1_IN7     V 相电流
ADC_I_C    PB0     ADC1_IN8     W 相电流 (可选)
ADC_VBUS   PB1     ADC1_IN9     总线电压 (可选)
```

> **注意**: 当前 TIM1 只用了 CH1/CH2/CH3 (上桥), 没配 CH1N/CH2N/CH3N (下桥)。如果驱动芯片需要 6 路独立 PWM 输入才需配互补通道。

### 4.2 FOC 控制信号 — 需修复 (2 引脚)

| 功能 | 当前引脚 | 问题 | 建议 |
|------|---------|------|------|
| **DRV8313_EN** | PB15 | ❌ 未配置 | 保留 PB15, 在代码中手动配为 GPIO_OUT_PP, 上电拉高 |
| **DRV8313_nFAULT** | PA11 | ❌ 未配置 | 保留 PA11, 配为 GPIO_IN, 中断检测 |
| DRV8313_RESET | ~~PA1~~ | ❌ 冲突 | 放弃, 或改到空闲引脚 (如 PC4) |
| DRV8313_SLEEP | ~~PA2~~ | ❌ 冲突 | 放弃, 或改到空闲引脚 (如 PB9) |

### 4.3 磁编 MT6816 × 2 — 共享 SPI1 (5 引脚)

> ✅ **1 号已配, 增加 2 号 CS**

```text
功能       引脚     外设          备注
SPI1_SCK   PA5     SPI1_SCK     共享时钟
SPI1_MISO  PB4     SPI1_MISO    共享数据入
SPI1_MOSI  PB5     SPI1_MOSI    共享数据出
CS_ENC1    PA4     GPIO_OUT     编码器1 片选 (已配 ✅)
CS_ENC2    PC1     GPIO_OUT     编码器2 片选 (新分配)
```

### 4.4 寻迹传感器 × 8 — GPIO Input (8 引脚)

推荐 **PD 端口** (避开 PD5 = USART2_TX, 但 USART2 可回收):

```text
功能       引脚     备注
TRACK_1    PD0     GPIO_IN
TRACK_2    PD1     GPIO_IN
TRACK_3    PD2     GPIO_IN
TRACK_4    PD3     GPIO_IN
TRACK_5    PD4     GPIO_IN
TRACK_6    PD6     GPIO_IN  (跳过 PD5 = USART2_TX)
TRACK_7    PD7     GPIO_IN
TRACK_8    PD8     GPIO_IN
```

### 4.5 陀螺仪 × 2 + OLED × 1 — I2C1 共享 (2 引脚)

> ✅ **已配 PB6/PB7**

```text
功能       引脚     地址          备注
I2C1_SCL   PB6     ──            共享时钟
I2C1_SDA   PB7     ──            共享数据
OLED       0x3C    ──            SSD1306/SH1106
陀螺仪1     0x68    ──            MPU6050/ICM-20948
陀螺仪2     0x69    ──            AD0 拉高
```

### 4.6 串口 × 2 (4 引脚)

```text
功能       引脚     外设          备注
USART1_TX  PA9     USART1_TX    调试口 (已配 ✅)
USART1_RX  PA10    USART1_RX    调试口 (已配 ✅)
USART2_TX  PD5     USART2_TX    外设通信 (已配, 回收用)
USART2_RX  PA3     USART2_RX    外设通信 (已配, 回收用)
```

或者也可用 USART3 (PB10/PB11), 但 PB10 当前是 SPI2_SCK。用 USART2 更省事因为 `usart.c` 已配好。

### 4.7 驱动电机 × 2 — PWM + DIR (6 引脚)

回收 **TIM2** (当前因冲突不可用, 改为 DC 电机控制):

```text
功能       引脚     外设          备注
MOT1_PWM   PA0     TIM2_CH1     DC 电机 1 速度 PWM
MOT1_DIR1  PA7     GPIO_OUT     DC 电机 1 方向 A
MOT1_DIR2  PC5     GPIO_OUT     DC 电机 1 方向 B
MOT2_PWM   PA1     TIM2_CH2     DC 电机 2 速度 PWM
MOT2_DIR1  PC0     GPIO_OUT     DC 电机 2 方向 A
MOT2_DIR2  PC8     GPIO_OUT     DC 电机 2 方向 B
```

> ⚠️ **注意**: PA0/PA1 当前被 check_I.c 占用 (TIM5_CH1 + ADC)。若要用作 DC 电机PWM, 必须修改 check_I.c:
> - PA0: 放弃 TIM5_CH1 的 OC1REF 输出 (改为纯内部触发, 或用 TIM5 的 Update 事件触发 ADC)
> - PA1: 放弃 ADC1_IN9 (PB1 还在就行), 重新配为 TIM2_CH2 或 GPIO
> 
> **或改用其他定时器** (如 TIM3/PB4 或 TIM8/PC6):
> - MOT1_PWM = PB4 (TIM3_CH1), PB4 当前是 SPI1_MISO, 冲突
> - **更干净的方案**: 把 DC 电机 PWM 放到未用的定时器通道如 TIM12/PB14, TIM12_CH1 on PB14

### 4.8 霍编 × 2 — 定时器编码器模式 (4 引脚)

> ✅ **已配 TIM3(PC6/PC7) + TIM4(PD12/PD13)**

```text
功能       引脚     定时器       备注
ENC1_A     PC6     TIM3_CH1    霍尔编码器 1 A 相 (已配 ✅)
ENC1_B     PC7     TIM3_CH2    霍尔编码器 1 B 相 (已配 ✅)
ENC2_A     PD12    TIM4_CH1    霍尔编码器 2 A 相 (已配 ✅)
ENC2_B     PD13    TIM4_CH2    霍尔编码器 2 B 相 (已配 ✅)
```

### 4.9 按钮 (1 引脚)

```text
功能       引脚     类型        备注
KEY_USER   PE8     GPIO_IN     板载按键 (外部下拉)
```

> PA0 已被 TIM5_CH1 占用, 所以用 PE8 (板载第二个按键, 根据搜索资料天空星有 PA0 + PE8 两个按键)。

---

## 5. P1/P2 排针物理映射

> ⚠️ **注意**: 以下 P1-x / P2-x 编号基于天空星 2×40P 排针通用布局。**请以官方原理图为准核实**。  
> 交互式引脚图: https://openkits-wiki.easyeda.com/storage/html/fdb-pinout/index.html

### 5.1 天空星排针布局

```
                    ┌──────┐
 P1 (左排针)        │MCU   │      P2 (右排针)
 ┌─ 40P ─┐          │LQFP100│     ┌─ 40P ─┐
 │P1-1   │          │       │     │P2-1   │
 │P1-2   │          └──────┘     │P2-2   │
 │...    │                        │...    │
 │P1-39  │                        │P2-39  │
 │P1-40  │                        │P2-40  │
 └───────┘                        └───────┘
```

### 5.2 全部引脚 → 排针映射

#### P1 左侧排针 (40P)

| P1# | MCU引脚 | 分配功能 | 备注 |
|:---:|:-------:|---------|------|
| 1 | VCC_5V | — | 板载 5V 输入 |
| 2 | VCC_3V3 | — | 板载 3.3V |
| 3 | GND | — | 地 |
| 4 | **PA0** | MOT1_PWM / ~~TIM5_CH1~~ | ⚠️ 冲突, 需决定用途 |
| 5 | **PA1** | MOT2_PWM / ~~ADC/DRV~~ | ⚠️ 冲突, 需决定用途 |
| 6 | **PA2** | ~~TIM2/DRV_SLEEP~~ | ❌ 冲突未决 |
| 7 | **PA3** | USART2_RX | ✅ |
| 8 | **PA4** | MT6816 CS1 | ✅ |
| 9 | **PA5** | SPI1_SCK | ✅ |
| 10 | **PA6** | ADC1_IN6 (电流A) | ✅ |
| 11 | **PA7** | ADC1_IN7 (电流B) | ✅ |
| 12 | **PA8** | I2C3_SCL (预留) | 或 GPIO 备用 |
| 13 | **PA9** | USART1_TX | ✅ 调试口 |
| 14 | **PA10** | USART1_RX | ✅ 调试口 |
| 15 | **PA11** | DRV8313_nFAULT | ❌ 需配置 |
| 16 | PA12 | USB_DP | 板载 USB, 不引出 |
| 17 | PA13 | SWDIO | 调试口, 不引出 |
| 18 | PA14 | SWCLK | 调试口, 不引出 |
| 19 | **PA15** | 备用 GPIO | 仅 SWD 模式下可用 |
| 20 | GND | — | 地 |
| 21 | **PB0** | ADC1_IN8 (电流C) | ✅ |
| 22 | **PB1** | ADC1_IN9 (电压) | ✅ |
| 23 | **PB2** | 指示灯 | ✅ |
| 24 | **PB3** | 备用 GPIO | SWO/GPIO |
| 25 | **PB4** | SPI1_MISO | ✅ |
| 26 | **PB5** | SPI1_MOSI | ✅ |
| 27 | **PB6** | I2C1_SCL (OLED+陀螺仪) | ✅ |
| 28 | **PB7** | I2C1_SDA (OLED+陀螺仪) | ✅ |
| 29 | **PB8** | 板载 LED | 低电平点亮, 可用作 GPIO |
| 30 | **PB9** | 备用 GPIO | ✅ |
| 31 | **PB10** | ~~SPI2_SCK~~ | 可回收为 USART3_TX 或 GPIO |
| 32 | **PB11** | GPIO_OUT (未知) | 或 USART3_RX |
| 33 | **PB12** | 备用 | ✅ |
| 34 | **PB13** | 备用 | ✅ |
| 35 | **PB14** | 备用 (TIM12_CH1) | 可作 DC 电机 PWM |
| 36 | **PB15** | DRV8313_EN | ❌ 需配置 |
| 37 | VCC_3V3 | — | |
| 38 | GND | — | |
| 39 | VCC_5V | — | |
| 40 | GND | — | |

#### P2 右侧排针 (40P)

| P2# | MCU引脚 | 分配功能 | 备注 |
|:---:|:-------:|---------|------|
| 1 | VCC_5V | — | |
| 2 | VCC_3V3 | — | |
| 3 | GND | — | |
| 4 | **PC0** | MOT2_DIR1 (备用) | ✅ 或 ADC 备用 |
| 5 | **PC1** | MT6816 CS2 | ✅ (新分配) |
| 6 | **PC2** | ~~SPI2_MISO~~ | 可回收使用 |
| 7 | **PC3** | ~~SPI2_MOSI~~ | 可回收使用 |
| 8 | **PC4** | GPIO_OUT (未知用途) | 或 DRV8313_RESET |
| 9 | **PC5** | MOT1_DIR1 | ✅ (新分配) |
| 10 | **PC6** | TIM3_CH1 (霍编1A) | ✅ |
| 11 | **PC7** | TIM3_CH2 (霍编1B) | ✅ |
| 12 | **PC8** | MOT2_DIR2 | ✅ (新分配) |
| 13 | **PC9** | I2C3_SDA (预留) | 或 GPIO 备用 |
| 14 | **PC10** | 备用 | ✅ |
| 15 | **PC11** | 备用 | ✅ |
| 16 | **PC12** | 备用 | ✅ |
| 17 | PC13 | 备用 (驱动弱) | 部分板用作 LED |
| 18 | PC14 | 备用/LSE | 若 32kHz 晶振未贴装 |
| 19 | PC15 | 备用/LSE | 若 32kHz 晶振未贴装 |
| 20 | GND | — | |
| 21 | **PD0** | 寻迹 TRACK_1 | ✅ |
| 22 | **PD1** | 寻迹 TRACK_2 | ✅ |
| 23 | **PD2** | 寻迹 TRACK_3 | ✅ |
| 24 | **PD3** | 寻迹 TRACK_4 | ✅ |
| 25 | **PD4** | 寻迹 TRACK_5 | ✅ |
| 26 | **PD5** | USART2_TX | 或寻迹 TRACK_6 (二选一) |
| 27 | **PD6** | 寻迹 TRACK_6 | ✅ |
| 28 | **PD7** | 寻迹 TRACK_7 | ✅ |
| 29 | **PD8** | 寻迹 TRACK_8 | ✅ |
| 30 | **PD9** | 备用 | ✅ |
| 31 | **PD10** | 备用 | ✅ |
| 32 | **PD11** | 备用 | ✅ |
| 33 | **PD12** | TIM4_CH1 (霍编2A) | ✅ |
| 34 | **PD13** | TIM4_CH2 (霍编2B) | ✅ |
| 35 | **PD14** | 备用 | ✅ |
| 36 | **PD15** | 备用 | ✅ |
| 37 | **PE8** | 板载按键 | ✅ |
| 38 | **PE9** | TIM1_CH1 (FOC U相) | ✅ |
| 39 | PE10 | TIM1_CH2N | 可选互补输出 |
| 40 | **PE11** | TIM1_CH2 (FOC V相) | ✅ |
| 41 | PE12 | TIM1_CH3N | 可选互补输出 |
| 42 | **PE13** | TIM1_CH3 (FOC W相) | ✅ |
| 43 | PE0 | 备用 | ✅ |
| 44 | PE1 | 备用 | ✅ |
| 45 | PE2 | 备用 | ✅ |
| 46 | PE4 | 备用 | ✅ |
| 47 | PE5 | 备用 | ✅ |
| 48 | PE6 | 备用 | ✅ |
| 49 | GND | — | |
| 50 | VCC_3V3 | — | |

> ⚠️ **注意**: P2 的编号顺序 (1-50) 取决于排针定义方向, 以上基于常见天空星核心板布局。P2 实际上也是 40P (非 50P), 需要从原理图中确认 PE0/PE1/PE2/PE4/PE5/PE6 的确切位置。

---

## 6. 备用接口汇总

### 6.1 分配总账

| 功能组 | 占用 | 已用引脚 |
|--------|:----:|----------|
| **FOC** | 3+2 | PE9/PE11/PE13, PA6/PA7 |
| **FOC 控制** | 2 | PB15(EN), PA11(FAULT) |
| **MT6816** | 3+2 | PA5/PB4/PB5, PA4/PC1 |
| **寻迹** | 8 | PD0~PD4, PD6~PD8 |
| **I2C 总线** | 2 | PB6/PB7 |
| **串口** | 4 | PA9/PA10, PD5/PA3 |
| **DC 电机** | 6 | PA0/PA1, PA7/PC5/PC0/PC8 |
| **霍编** | 4 | PC6/PC7, PD12/PD13 |
| **按键** | 1 | PE8 |
| **已用小计** | **~35** | |
| **可用总 GPIO** | **~70** | |
| **备用** | **~35** | |

### 6.2 备用引脚清单

```text
端口   可用引脚                                     数量
─────── ────────────────────────────────────────── ────
PORTA  PA15 (JTDI)                                 1
PORTB  PB3 (SWO), PB8 (LED), PB9, PB12, PB13, PB14  6
PORTC  PC2 (SPI2), PC3 (SPI2), PC9~PC12, PC13~PC15  9
PORTD  PD9~PD11, PD14, PD15                         5
PORTE  PE0~PE2, PE4~PE6                              6
─────── ────────────────────────────────────────── ────
合计                                                27
```

### 6.3 备用引脚推荐用途

| 用途 | 推荐引脚 | 说明 |
|------|---------|------|
| 蜂鸣器 (PWM) | PB14 (TIM12_CH1) | 有定时器支持 |
| RGB LED (WS2812) | PD9 | 任意 GPIO, 单线协议 |
| CAN 总线 | PB8(RX), PB9(TX) (CAN1) | 需 CAN 收发器 |
| 舵机 (PWM) | PC9 (TIM8_CH4) | 50Hz PWM |
| 额外按键 | PE0, PE1 | 简单 GPIO 输入 |
| 超声波 Trig/Echo | PD10, PD11 | GPIO OUT + GPIO IN |
| 测速传感器 | PE4, PE5 | 外部中断输入 |
| I2C 备用 | PB8/PB9 (I2C1 重映射) | 或软件模拟 |

---

## 7. 必须处理的问题

### 优先级 P0 — 不改无法工作的

| # | 问题 | 建议解决方式 |
|---|------|------------|
| 1 | **PA0 被 TIM5_CH1 + TIM2_CH1 争夺** | 在 TIM5 中改用 **Update 事件** 触发 ADC 而非 OC1REF, 释放 PA0 引脚。TIM5 纯内部运行不需要 GPIO。修改 `check_I.c` |
| 2 | **PA1 被 ADC1_IN9 + TIM2_CH2 + DRV_RESET 争夺** | 决定用途: 若不需要第 4 路 ADC, 删除 PA1 的 analog 配置, 配为 TIM2_CH2 或 GPIO |
| 3 | **PB15 (EN) 未配置** | 在 `main.c` 或 `FOC_2::init()` 中加: `HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)` (使能驱动) |
| 4 | **PA11 (FAULT) 未配置** | 在 `gpio.c` 添加: `GPIO_MODE_INPUT, GPIO_NOPULL` |

### 优先级 P1 — 需要确认

| # | 问题 | 建议 |
|---|------|------|
| 5 | **P1/P2 排针映射需确认** | 在浏览器打开 `openkits-wiki.easyeda.com/storage/html/fdb-pinout/index.html` 核对 |
| 6 | **寻迹 vs USART2 的选择** | PD5 要么是 USART2_TX, 要么是 TRACK_6。若只需 1 路外设串口则用 USART2, 寻迹 PD0~PD7 全用 |
| 7 | **DC 电机 PWM 定时器选择** | 方案 A: 回收 TIM2 (PA0/PA1) — 需改 check_I.c。方案 B: 用 TIM12 (PB14) + TIM13 (PC8) — 纯新增无冲突 |

### 优先级 P2 — 备用的高质量选项

| # | 选项 | 说明 |
|---|------|------|
| 8 | PB8/PB9 改 I2C1 | 目前 I2C1 在 PB6/PB7。天空星板载 LED 在 PB8, 如果 OLED+陀螺仪想用不同引脚可改映射 |
| 9 | SPI Flash 贴装 | 如果天空星板上贴了 W25Q128, PB13/PB14/PB15 被占用, 需避开 |

---

## 参考资源

| 资源 | 链接 |
|------|------|
| 天空星交互式引脚图 | https://openkits-wiki.easyeda.com/storage/html/fdb-pinout/index.html |
| 天空星硬件文档 | https://wiki.lckfb.com/zh-hans/tkx/hardware/explain.html |
| 飞书硬件文档(含原理图PDF) | https://lceda001.feishu.cn/wiki/D4cqwUkiTi6723knO2cczSThnYb |
| STM32F407VET6 数据手册 | ST 官网 RM0090 |
| STM32F407VET6 LQFP100 引脚图 | 芯片手册 Table 4 |
