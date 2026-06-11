# 立创天空星 LCKFB-SkyStar-STM32F407VET6 引脚分配文档

> 芯片: **STM32F407VET6** (LQFP100, 512KB Flash, 192KB SRAM)  
> 开发板: **立创天空星核心板** (LCKFB-LSPI-SkyStar)  
> 文档版本: 1.0 | 日期: 2026-06-10

---

## 目录

1. [开发板概览](#1-开发板概览)
2. [MCU 引脚资源总览](#2-mcu-引脚资源总览)
3. [板载外设消耗](#3-板载外设消耗)
4. [组件表勘误](#4-组件表勘误)
5. [引脚分配方案](#5-引脚分配方案)
6. [备用接口汇总](#6-备用接口汇总)
7. [参考资源](#7-参考资源)

---

## 1. 开发板概览

### 硬件特性

| 项目 | 说明 |
|------|------|
| **排针引出** | 2×40P (P1 左侧, P2 右侧) + 2×5P 调试口 |
| **板载 LED** | PB8 (推挽输出, **低电平点亮**) |
| **板载按键** | PA0 (KEY1, 外部下拉, 按下高电平), 复位键, BOOT键 |
| **调试口** | SWDIO(PA13), SWCLK(PA14), SWO(PB3), USART1(PA9/PA10) |
| **USB** | PA11(USB_DM), PA12(USB_DP) — Type-C |
| **高速晶振** | 8MHz (HSE) |
| **低速晶振** | 32.768kHz (LSE, VET6版未贴装, PC14/PC15 理论上可用作GPIO) |
| **SPI Flash** | W25Q128 (部分版本已贴装, 接于 SPI 总线) |
| **TF 卡座** | SDIO 4-bit 接口 |
| **供电** | Type-C (5V) → 3.3V LDO, 最大输出 500mA |

### 排针布局示意

```
 ┌─────────────────────────────────────────┐
 │ Type-C                                  │
 │  [P1]  ┌─────────────┐  [P2]            │
 │  P1-1  │ STM32F407   │  P2-20    P2-1   │
 │  P1-2  │   VET6      │           P2-2   │
 │  ...   │   LQFP100   │  ...      ...    │
 │  P1-20 └─────────────┘  P2-40    P2-21  │
 │         [DEBUG] 2×5P                     │
 └─────────────────────────────────────────┘

 P1 = 左侧排针 (40P)   P2 = 右侧排针 (40P)
```

---

## 2. MCU 引脚资源总览

STM32F407VET6 LQFP100 封装可用 GPIO：

| 端口 | 可用引脚 | 数量 |
|------|---------|:----:|
| GPIOA | PA0 ~ PA15 | 16 |
| GPIOB | PB0 ~ PB15 | 16 |
| GPIOC | PC0 ~ PC15 | 16 |
| GPIOD | PD0 ~ PD15 | 16 |
| GPIOE | PE0 ~ PE2, PE4 ~ PE6 | 6 |
| **总计** | | **70** |

### 外设资源（LQFP100）

| 外设 | 通道/数量 | 可用引脚 |
|------|:--------:|---------|
| **TIM1** | 4CH (+ 3CH_N) | PA8~PA11, PB13~PB15, PE9/PE11... |
| **TIM2** | 4CH | PA0~PA3, PB2, PB3, PA15 |
| **TIM3** | 4CH | PA6, PA7, PB0~PB5, PC6~PC9 |
| **TIM4** | 4CH | PB6~PB9, PD12~PD15 |
| **TIM5** | 4CH | PA0~PA3, PH6, PH7... |
| **TIM8** | 4CH (+ 3CH_N) | PC6~PC9, PA6, PA7... |
| **ADC1/2/3** | 各16CH | PA0~PA7, PB0, PB1, PC0~PC5 |
| **USART1/2/3/6** | TX+RX | PA9/PA10, PD5/PD6, PB10/PB11... |
| **UART4/5/7/8** | TX+RX | PD0/PD1, PC12/PD2... |
| **SPI1/2/3** | SCK+MISO+MOSI | PA5/PA6/PA7, PB13/PB14/PB15, PC10/PC11/PC12... |
| **I2C1/2/3** | SCL+SDA | PB6/PB7, PB8/PB9, PA8/PC9... |
| **SDIO** | 4-bit | PC8~PC12, PD2 |
| **CAN1/2** | RX+TX | PB8/PB9, PB5/PB6... |
| **DAC1/2** | 2CH | PA4, PA5 |

---

## 3. 板载外设消耗

| 引脚 | 功能 | 备注 |
|------|------|------|
| PA0 | KEY1 (按键) | 可复用为用户输入, 外部下拉 |
| PA9 | USART1_TX → 调试口 | **占用**, 连接到调试口CH340 |
| PA10 | USART1_RX → 调试口 | **占用** |
| PA11 | USB_DM | **占用** (Type-C 直连) |
| PA12 | USB_DP | **占用** (Type-C 直连) |
| PA13 | SWDIO | **占用** (调试) |
| PA14 | SWCLK | **占用** (调试) |
| PB3 | SWO (可选) | 调试接口引出, 可作为 GPIO 使用 |
| PB8 | LED | 低电平点亮, 可复用为 GPIO |
| PB13/PB14/PB15 | SPI Flash (如果有) | W25Q128 占用 SPI2, **确认版本是否贴装** |
| PH0/PH1 | OSC_IN/OUT (8MHz) | **非GPIO**, 晶振专用 |

> ⚠️ 以下引脚也需确认：
> - **PC14/PC15**: 若 32.768kHz 未贴装, 可作为 GPIO (需软件配置)
> - **SPI Flash CS**: 如果贴装了 SPI Flash，CS 会占用一个额外 GPIO
> - **BOOT 引脚**: PB2(BOOT1), 由跳线/按键控制

### 扣除板载消耗后的可用 GPIO

| 端口 | 可用 | 被占用 |
|:----|:----:|:------:|
| PORTA | 8 (PA1~PA8, PA15) | PA0, PA9~PA14 |
| PORTB | 15 (PB0~PB7, PB9~PB15) | PB8(LED) |
| PORTC | 16 | - (全可用) |
| PORTD | 16 | - (全可用) |
| PORTE | 6 | - (全可用) |
| **合计** | **~61** | |

---

## 4. 组件表勘误

你的原始表：

| 组件 | 接口 | 数量 | 电压 | 备注（原） |
|------|------|:----:|:----:|:---------:|
| 驱动FOC | PWM x6, ADC x4 | 1 | 12V | ✅ 准确 |
| 磁编MT6816 | SPI x4 | 2 | 3v3 | ⚠️ 见下方 |
| 寻迹 | Input x8 | 1 | - | ✅ |
| 陀螺仪 | I2C x2 | 2 | 3v3 | ⚠️ 见下方 |
| 串口 | uart x2 | 2 | 3v3 | ✅ |
| OLED | I2C x2 | 1 | 3v3 | ⚠️ 见下方 |
| 驱动电机 | PWM x1 output x2 | 2 | 12v | ✅ |
| 霍编 | tim x2 | 2 | 3v3 | ✅ |

### 修正建议

| # | 问题 | 修正 |
|---|------|------|
| 1 | **SPI x4** — 磁编 MT6816×2 可共享 SPI 总线 | 1路 SPI（SCK+MOSI+MISO）+ 2路独立 CS = **5引脚**，除非你需要同时高速读取才开 2 路 SPI |
| 2 | **I2C x2 × (陀螺仪+OLED)** — 三者可共享同一 I2C 总线 | 所有 I2C 设备挂一路 I2C1（SCL+SDA），只需 **2引脚**；保留 I2C2 备用 |
| 3 | **I2C x2** 表意不清晰 | 建议写为 `I2C × 1`（总线数），后面备注设备数量 |
| 4 | 缺少 **按键/按钮** 的接口说明 | 板载 PA0 已有 1 个按键，如需更多需额外引脚 |
| 5 | **PWM x1 output x2** 命名不统一 | 建议写作 `PWM × 1 + GPIO × 2` 更清晰 |
| 6 | 缺少 **电源管理/使能** 引脚 | FOC 驱动通常需要 EN、nFAULT 等信号 |

### 修正后的资源需求

| 组件 | 所需引脚 | 说明 |
|------|:--------:|------|
| **FOC 驱动** (×1) | 6 PWM + 4 ADC + 2 GPIO | PWM: 3相上桥+下桥(6路); ADC: 电流×3 + 总线电压; GPIO: EN + nFAULT |
| **磁编 MT6816** (×2) | **5** (共享 SPI + 2 CS) | 或 8 (双 SPI + 双 CS) |
| **寻迹** (×1) | 8 GPIO (Input) | 红外/灰度传感器 |
| **陀螺仪** (×2) | 2 (共享 I2C) | + 可选 INT 引脚 |
| **串口** (×2) | 4 (TX×2 + RX×2) | 与 PC/外设通信 |
| **OLED** (×1) | 2 (共享 I2C) | 与陀螺仪同总线 |
| **驱动电机** (×2) | 2 PWM + 4 GPIO | PWM: 速度; GPIO: DIR 正反转 |
| **霍编** (×2) | 4 (TIM CH1+CH2 ×2) | A/B 相编码器输入 |
| **按键** | 1~2 | 用户交互 |
| **总计** | **~34~42 引脚** | 远小于可用 GPIO |

---

## 5. 引脚分配方案

### 5.1 FOC 驱动 — PWM + ADC + EN/nFAULT (共 12 引脚)

FOC 需要 6 路互补 PWM（高级定时器 TIM1 或 TIM8），以及 ADC 采样电流。

```plaintext
功能       引脚     定时器        备注
───────────────────────────────────────────────
PWM_UH    PA8     TIM1_CH1     上桥 U 相
PWM_UL    PB13    TIM1_CH1N    下桥 U 相
PWM_VH    PA9     TIM1_CH2     上桥 V 相
PWM_VL    PB14    TIM1_CH2N    下桥 V 相
PWM_WH    PA10    TIM1_CH3     上桥 W 相
PWM_WL    PB15    TIM1_CH3N    下桥 W 相
───────────────────────────────────────────────
ADC_I_U   PA4     ADC1_IN4     U 相电流
ADC_I_V   PA5     ADC1_IN5     V 相电流
ADC_I_W   PA6     ADC1_IN6     W 相电流 (可选)
ADC_VBUS  PC0     ADC1_IN10    总线电压
───────────────────────────────────────────────
EN_DRV    PC1     GPIO_OUT     驱动使能
nFAULT    PC2     GPIO_IN      驱动故障报警
```

> ⚠️ **注意**: PB13/PB14/PB15 如果被 SPI Flash 占用，改用 TIM8:
> - PC6 (TIM8_CH1), PC7 (TIM8_CH2), PC8 (TIM8_CH3)
> - 互补输出需查 TIM8_CH1N/2N/3N 对应引脚

### 5.2 磁编 MT6816 × 2 — SPI + CS (共 5 引脚)

共享同一 SPI 总线，独立片选：

```plaintext
功能       引脚     外设        备注
────────────────────────────────────────
SPI3_SCK   PC10    SPI3_SCK    共享时钟
SPI3_MISO  PC11    SPI3_MISO   共享数据输入
SPI3_MOSI  PC12    SPI3_MOSI   共享数据输出
CS_ENC1    PC3     GPIO_OUT    编码器1 片选
CS_ENC2    PC4     GPIO_OUT    编码器2 片选
```

> **替代方案**: 如需更高速度可开两路 SPI:
> - SPI1: PA5(SCK), PA6(MISO), PA7(MOSI) + CS1/CS2
> - SPI3: PC10(SCK), PC11(MISO), PC12(MOSI) + CS (备用)

### 5.3 寻迹传感器 × 1 — 8路数字输入 (共 8 引脚)

```plaintext
功能       引脚     类型        备注
─────────────────────────────────────────
TRACK_1    PD0     GPIO_IN     寻迹传感器 1
TRACK_2    PD1     GPIO_IN     寻迹传感器 2
TRACK_3    PD2     GPIO_IN     寻迹传感器 3
TRACK_4    PD3     GPIO_IN     寻迹传感器 4
TRACK_5    PD4     GPIO_IN     寻迹传感器 5
TRACK_6    PD5     GPIO_IN     寻迹传感器 6
TRACK_7    PD6     GPIO_IN     寻迹传感器 7
TRACK_8    PD7     GPIO_IN     寻迹传感器 8
```

### 5.4 陀螺仪 × 2 + OLED × 1 — 共享 I2C (共 2 引脚)

三者共享 I2C1 总线，不同设备地址区分：

```plaintext
功能       引脚     外设        备注
────────────────────────────────────────
I2C1_SCL   PB6     I2C1_SCL    共享时钟
I2C1_SDA   PB7     I2C1_SDA    共享数据
────────────────────────────────────────
设备1      陀螺仪1  I2C 地址 0x68   (如 MPU6050/ICM-20948)
设备2      陀螺仪2  I2C 地址 0x69   (AD0 拉高)
设备3      OLED     I2C 地址 0x3C   (SSD1306/SH1106)
```

> 可选: 陀螺仪 **INT 引脚**:
> - INT1_GYRO → PB9 (GPIO_EXTI)
> - INT2_GYRO → PB8（与 LED 复用, 不直接用 LED 则释放）

### 5.5 串口 × 2 — UART (共 4 引脚)

```plaintext
功能       引脚     外设        备注
────────────────────────────────────────
UART2_TX   PD5     USART2_TX   PC 通信/调试 (USART1已被调试口占用)
UART2_RX   PD6     USART2_RX
UART3_TX   PB10    USART3_TX   外设通信 (如蓝牙/数传)
UART3_RX   PB11    USART3_RX
```

### 5.6 直流电机驱动 × 2 — PWM + DIR (共 6 引脚)

```plaintext
功能       引脚     定时器/GPIO  备注
─────────────────────────────────────────
MOT1_PWM   PA1     TIM2_CH2    电机1 速度 PWM
MOT1_DIR1  PA2     GPIO_OUT    电机1 方向 A
MOT1_DIR2  PA3     GPIO_OUT    电机1 方向 B
MOT2_PWM   PA7     TIM3_CH2    电机2 速度 PWM
MOT2_DIR1  PC5     GPIO_OUT    电机2 方向 A
MOT2_DIR2  PC6     GPIO_OUT    电机2 方向 B
```

### 5.7 霍编 × 2 — 定时器输入捕获 (共 4 引脚)

```plaintext
功能       引脚     定时器       备注
─────────────────────────────────────────
ENC1_A     PA6     TIM3_CH1    编码器1 A 相
ENC1_B     PA7     TIM3_CH2    编码器1 B 相
ENC2_A     PB4     TIM3_CH1?   编码器2 A 相
ENC2_B     PB5     TIM3_CH2?   编码器2 B 相
```

> ⚠️ **注意**: PA6/PA7 与 FOC 的 ADC 冲突！替代方案:
> - TIM4: PB6(PB6_CH1?), PB7(PB7_CH2?) — 但与 I2C1 冲突！
> - **推荐方案**: 改霍编用 TIM5:
>   - ENC1_A → PA0 (TIM5_CH1) — 但 PA0 是板载按键
>   - 改用 TIM8: PC6(PC6_CH1?), PC7(PC7_CH2?)
>
> **最终推荐 — 无冲突方案**:
> - ENC1_A → PD12 (TIM4_CH1)
> - ENC1_B → PD13 (TIM4_CH2)
> - ENC2_A → PD14 (TIM4_CH3)
> - ENC2_B → PD15 (TIM4_CH4)

```
→ 更新后:
功能       引脚     定时器       备注
─────────────────────────────────────────
ENC1_A     PD12    TIM4_CH1    编码器1 A 相
ENC1_B     PD13    TIM4_CH2    编码器1 B 相
ENC2_A     PD14    TIM4_CH3    编码器2 A 相
ENC2_B     PD15    TIM4_CH4    编码器2 B 相
```

### 5.8 按键 (共 1 引脚, 板载)

```plaintext
功能       引脚     类型        备注
─────────────────────────────────────────
KEY_USER   PA0     GPIO_IN     板载按键 (外部下拉)
```

---

## 6. 备用接口汇总

分配完上述外设后，剩余可用接口：

### 最终引脚总账

| 功能组 | 占用引脚 | 已用引脚 |
|--------|:--------:|:--------:|
| FOC PWM | 6 | PA8, PB13, PA9, PB14, PA10, PB15 |
| FOC ADC | 4 | PA4, PA5, PA6, PC0 |
| FOC GPIO | 2 | PC1, PC2 |
| 磁编 SPI | 5 | PC10, PC11, PC12, PC3, PC4 |
| 寻迹 | 8 | PD0~PD7 |
| 陀螺仪+OLED I2C | 2 | PB6, PB7 |
| 串口 | 4 | PD5, PD6, PB10, PB11 |
| 电机驱动 | 6 | PA1, PA2, PA3, PA7, PC5, PC6 |
| 霍编 | 4 | PD12~PD15 |
| **已用小计** | **41** | |
| **可用总计** | **~70** | |
| **备用** | **~29** | |

### 备用引脚清单

```plaintext
端口    可用引脚                           数量
─────────────────────────────────────────────
PORTA   PA15*                              1
PORTB   PB0, PB1, PB2, PB4, PB5, PB9       6
PORTC   PC7, PC8, PC9, PC13**, PC14**, PC15**  6
PORTD   PD8, PD9, PD10, PD11                4
PORTE   PE0, PE1, PE2, PE4, PE5, PE6        6
─────────────────────────────────────────────
合计                                       23
```

> `* PA15` — JTDI 引脚，用作 GPIO 需注意 JTAG 配置  
> `** PC13/PC14/PC15` — 注意：PC13 驱动能力较弱，PC14/PC15 若 LSE 未贴装才可用

### 备用引脚推荐用途

| 用途 | 推荐引脚 | 说明 |
|------|---------|------|
| 额外按键/拨码开关 | PB0, PB1 | 简单 GPIO 输入 |
| 蜂鸣器 (PWM) | PB4 (TIM3_CH1) | 可通过定时器输出 PWM |
| RGB LED (WS2812) | PC7 | 单线协议, 任意 GPIO |
| CAN 总线 | PB8(CAN1_RX), PB9(CAN1_TX) | 需 CAN 收发器 |
| 舵机信号 (PWM) | PC8(TIM8_CH3), PC9(TIM8_CH4) | 50Hz PWM |
| 编码器/测速备用 | PD8, PD9, PD10, PD11 | 可作额外 GPIO 输入 |
| SPI/I2C 备用 | PE0~PE6 (软件模拟) | 任意 IO 均可模拟 |

---

## 7. 参考资源

| 资源 | 链接 |
|------|------|
| 天空星交互式引脚图 | https://openkits-wiki.easyeda.com/storage/html/fdb-pinout/index.html |
| 天空星硬件文档 | https://wiki.lckfb.com/zh-hans/tkx/hardware/explain.html |
| 飞书硬件文档(含原理图) | https://lceda001.feishu.cn/wiki/D4cqwUkiTi6723knO2cczSThnYb |
| STM32F407VET6 数据手册 | ST 官网 RM0090 |
| 立创商城商品页 | https://item.szlcsc.com/23849165.html |
| MT6816 数据手册 | — |
| 系统引脚定义 (当前项目) | `Core/Inc/gpio.h` + `Core/Inc/main.h` |

> **⚠️ 重要**: 此文档基于 STM32F407VET6 LQFP100 标准引脚分配，以及天空星核心板已知板载资源。  
> **P1/P2 排针物理位置对应关系** 请以上方飞书文档中的官方原理图为准。  
> 分配如需调整，请优先改动"备用库"中的引脚，避免牵动已确定的外设组。
