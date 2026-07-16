# CLAUDE.md — STM32 平衡小车 (car_example)

> 电赛 STM32 两轮自平衡小车项目。基于 STM32F103C8T6，Keil MDK (ARMCC V5) 编译，**纯寄存器级编程，无 HAL/标准外设库/LL 库**。

---

## 快速速览

**当前运行状态**：双轮编码器速度闭环 + 传感器数据采集。增量式 PID（P=500, I=10, D=0），10ms 控制周期。目标 A=2.05/B=2.0 脉冲/10ms。MPU6050 + HMC5883L 已正常工作，yaw/roll/pitch 实时采集，yaw 包裹到 ±180°。HMC 磁干扰可忽略。陀螺补偿系数 3.82。

**下一步**：角度外环 PID（Step 2）。`pid_t angle` 已声明待初始化，`angle_error_normalize()` 已就绪。

**关键校准值**：`GYRO_SCALE_COMPENSATION = 3.82f`，`gz_bias` 由 `MPU6050_Calibrate()` 自动采集。

**已修复的底层问题**：[ml_i2c.c](ml_libs/ml_i2c.c)/[ml_oled.c](ml_libs/ml_oled.c) I2C 时序加 `delay_us(2)`；[ml_gpio.c](ml_libs/ml_gpio.c) `gpio_set()` 改用 BSRR 原子写。

**诊断开关**：`B_PWM_PIN_TEST 0` → 1 跳过 PID 直驱 B 电机。

---

## 项目概览

- **MCU**: STM32F103C8T6 (Cortex-M3, 128KB Flash, 20KB SRAM, 72MHz)
- **编译器**: ARMCC V5.06 (Keil MDK uVision 5)
- **启动文件**: [startup_stm32f10x_md.s](sys/startup_stm32f10x_md.s) (中等容量)
- **C 标准**: C99, Optimization Level 1
- **库依赖**: 无。所有驱动为寄存器直接操作
- **包含模式**: 所有 `.c` 文件通过 `#include "headfile.h"` 引入一切

---

## 当前运行时状态

### 活跃功能

[main.c](user/main.c) `B_PWM_PIN_TEST 0` 分支（`#else`）：

1. **传感器初始化**：`I2C_Init()` → `MPU6050_Init()` → `HMC5883L_Init()` → `MPU6050_Calibrate()`
2. **速度 PID**：`pid_init(&motorA/B, DELTA_PID, 500, 10, 0)`，`motor_target_set(2.05f, 2.0f)`
3. **10ms 控制周期**：`tim_interrupt_ms_init(TIM_3, 10, 0)` → `pid_control()` 在 TIM3 ISR 中
4. **100Hz 姿态更新**：主循环内 10×(`Attitude_Update()` + `yaw_gyro` 积分 + `delay_ms(10)`)
5. **10Hz yaw 融合**：`HMC5883L_GetData()` → `atan2(hmc_y, hmc_x)` → 95%陀螺+5%磁力计 → 包裹到 ±180°
6. **OLED 切屏**：2 秒速度环 / 2 秒姿态屏交替
7. **串口输出**：~550ms 一次，含 yaw/roll/pitch + 电机数据 + HMC原始值 + gz

PID 输出被限制在 `[0, PID_TEST_MAX_DUTY=30000]`，PWM 全量程为 `MAX_DUTY=50000`。

### 已实现但未接入

| 模块 | 位置 | 状态 |
|------|------|------|
| `pid_t angle` | [pid.c:7](code/pid.c#L7) | 已声明，待 Step 2 初始化 |
| `gray_init()` / `track()` | [gray_track.c](code/gray_track.c) | 8 通道查表法循迹，待 Step 4-5 接入 |
| `datavision_send()` | [pid.c:17](code/pid.c#L17) | 二进制协议发送函数，未使用 |
| Mahony 互补滤波 | [filter.c](code/filter.c) | 未使用（已选用互补滤波管线） |
| `EXTI9_5_IRQHandler` (PB7) | [isr.c:103](user/isr.c#L103) | 已禁用（与主循环 I2C 冲突） |

### 传感器管线（当前方案）

选用 attitude.c 互补滤波，主循环中调用：
- `Attitude_Update()` @ 100Hz → roll/pitch（互补滤波）+ 更新 gx/gy/gz
- `yaw_gyro` 积分 @ 100Hz（内循环同步）
- `HMC5883L_GetData()` @ ~10Hz → `atan2(hmc_y, hmc_x)` → 95%陀螺+5%磁力计融合

`MPU6050_Calibrate()` 在上电时调用（1 秒静止校准陀螺零偏）。

---

## 目录结构

```
car_example/
├── user/                    # 应用层入口
│   ├── main.c               # 主函数 (硬件初始化 + 主循环 + OLED 调试)
│   ├── isr.c                # 所有中断服务函数 (TIM3 PID 控制 + 编码器 EXTI + 传感器 EXTI9_5)
│   ├── attitude.c/h         # 姿态解算 (互补滤波, 磁力计倾斜补偿) — 当前未接入
│   ├── Project.uvprojx      # Keil 工程文件
│   └── Objects/ Listings/ DebugConfig/  # 构建产物
├── code/                    # 通用算法与设备驱动
│   ├── motor.c/h            # 直流电机 PWM + 方向 + 编码器初始化
│   ├── pid.c/h              # PID 控制器 (位置式 + 增量式), pid_control() 主控循环
│   ├── filter.c/h           # 卡尔曼滤波 + Mahony 互补滤波
│   └── gray_track.c/h       # 8 路灰度循迹传感器 (查表法差速)
├── ml_libs/                 # 自研硬件抽象层 (寄存器级)
│   ├── headfile.h           # 总头文件 (所有 .c 都 include 它, 级联包含)
│   ├── ml_gpio.c/h          # GPIO (CRL/CRH/ODR/IDR)
│   ├── ml_delay.c/h         # SysTick 延时 (us/ms/s)
│   ├── ml_pwm.c/h           # PWM (TIM2/3/4, MAX_DUTY=50000, 约 1kHz)
│   ├── ml_tim.c/h           # 定时器中断 (ms 级周期)
│   ├── ml_uart.c/h          # 串口 (USART1/2/3, printf 重定向)
│   ├── ml_adc.c/h           # ADC1/2
│   ├── ml_exti.c/h          # 外部中断 (PA0-PC7)
│   ├── ml_nvic.c/h          # NVIC 中断优先级分组
│   ├── ml_i2c.c/h           # 软件 I2C (PB10=SCL, PB11=SDA — 传感器总线)
│   ├── ml_mpu6050.c/h       # MPU6050 六轴 IMU 驱动
│   ├── ml_hmc5883l.c/h      # HMC5883L 三轴磁力计驱动
│   ├── ml_oled.c/h          # SSD1306 OLED 驱动 (独立 I2C: PB8/PB9)
│   └── ml_oled_font.h       # OLED 8×16 ASCII 字库
├── sys/                     # CMSIS 系统层
│   ├── core_cm3.h           # ARM CMSIS Core (Cortex-M3)
│   ├── core_cmFunc.h / core_cmInstr.h
│   ├── stm32f10x.h          # STM32F10x 外设寄存器定义
│   ├── system_stm32f10x.c/h # 时钟初始化 (HSE 8MHz → PLL×9 → 72MHz)
│   ├── startup_stm32f10x_md.s  # 中等容量启动文件 (当前使用)
│   └── startup_stm32f10x_hd.s  # 高容量启动文件 (备用)
├── scripts/
│   └── verify-deepseek-api.ps1   # DeepSeek API 密钥验证脚本
└── .vscode/                 # VS Code IntelliSense 配置 (不影响 Keil 编译)
```

---

## 分层架构

```
Layer 4: user/      — main.c, isr.c, attitude.c       (应用入口)
Layer 3: code/      — motor, pid, filter, gray_track   (算法/设备级驱动)
Layer 2: ml_libs/   — GPIO, PWM, I2C, UART, ADC...     (硬件抽象, 寄存器操作)
Layer 1: sys/       — startup, system_stm32f10x        (CMSIS 核心)
```

所有源文件统一 `#include "headfile.h"`，级联顺序：CMSIS → ML 库 → code 层 → user 层。

---

## 代码规范

> 以下规范总结自当前代码实际风格。添加新代码时请保持一致。

### 命名约定

**文件**：全小写 + 下划线。`ml_libs/` 下统一 `ml_` 前缀（`ml_gpio.c`、`ml_pwm.c`）。应用层无前缀（`motor.c`、`gray_track.c`）。

**函数**：snake_case 为主。分级前缀：
- `ml_libs/` 外设驱动：`外设名_动作` → `gpio_init()`、`uart_sendbyte()`、`pwm_update()`
- `ml_libs/` 芯片驱动：`芯片名_Function` → `MPU6050_Write()`、`HMC5883L_Read()`（匹配数据手册命名）
- `code/` / `user/` 层：`模块_动作` → `motor_init()`、`pid_control()`、`motor_target_set()`
- 区分 A/B 电机：`motorA_duty()`、`motorB_duty()`、`motorB_stop()`

**变量**：
- 局部变量：snake_case（`temp`、`gx_sum`、`print_count`）
- 全局变量：snake_case，调试变量用 `pid_debug_` 前缀（`pid_debug_a_target`）
- 外设索引数组：`外设名_index[3]` → `gpio_index[3]`、`tim_index[3]`、`uart_index[3]`
- ⚠️ 注意：`Encoder_count1/2` 首字母大写，是全局变量命名中唯一的 PascalCase 异常

**宏 / `#define`**：全大写 + 下划线。功能开关定义在 `.c` 内（`PID_TEST_MAX_DUTY`、`MOTOR_B_PWM_INVERTED`），公共常量定义在 `.h` 内（`MAX_DUTY`、`MPU6050_ADDR`）。⚠️ 唯一例外：[filter.c:4](code/filter.c#L4) 的 `#define alpha 0.95238` 是小写。

**结构体 typedef**：`小写_t` 后缀 → `pid_t`、`KF_t`。

**枚举 typedef**：`Peripheral_enum` → `GPIOn_enum`、`Pinx_enum`、`UARTn_enum`。枚举值用大写/帕斯卡（`GPIO_A = 0x00`、`Pin_0 = 0x00`、`RISING`、`FALLING`），值为显式十六进制以便直接用于位移。

**头文件保护**：⚠️ 风格不统一，存在 `_name_h`、`_name_h_`、`__NAME_H`、`_NAME_H` 四种写法。新模块建议用 `_模块名_h_` 格式。

### 注释风格

- **语言**：代码注释以**中文**为主，文档（CLAUDE.md、PROJECT_SUMMARY.md）为中英混合
- **ml_libs 层函数**：每个公开函数用 `//----` 分隔线 + `@brief/@param/@return/@Sample usage` 文档块：
  ```c
  //-------------------------------------------------------------------------------------------------------------------
  // @brief     GPIO初始化
  // @param     GPIOn    选择GPIO端口
  // @param     Pinx     选择引脚号
  // @param     mode     选择引脚模式
  // @return    void
  // Sample usage:    gpio_init(GPIO_A, Pin_0, OUT_PP);
  //-------------------------------------------------------------------------------------------------------------------
  ```
- **寄存器操作**：行尾中文注释解释寄存器位含义
  ```c
  RCC->APB2ENR |= 4<<GPIOn;    //使能时钟
  ```
- **引脚标注**：行尾注释标注信号名 → `gpio_init(GPIO_B, Pin_12, IU);   // D1`
- **分段标记**：[attitude.c](user/attitude.c) 用 `// ================= 标题 =================` 分隔代码段

### 包含模式

[headfile.h](ml_libs/headfile.h) 是唯一入口，级联包含：`stm32f10x.h` → 标准库 → 所有 `ml_*.h` → 所有 `code/*.h` → `user/attitude.h`。

大多数 `.c` 只需 `#include "headfile.h"`。以下文件自己 include 自己的 `.h`（而非依赖 headfile.h）：
- `motor.c` → `#include "motor.h"`
- `filter.c` → `#include "filter.h"`
- `ml_i2c.c` → `#include "ml_i2c.h"`
- `ml_mpu6050.c` → `#include "ml_mpu6050.h"`
- `ml_hmc5883l.c` → `#include "ml_hmc5883l.h"`
- `attitude.c` → `#include "attitude.h"`

⚠️ 不一致：`pid.c` 和 `gray_track.c` 只 include `headfile.h`，不 include 自己的 `.h`。

### 全局变量约定

**声明与定义分离**：`.h` 中 `extern` 声明（放在函数声明之后、`#endif` 之前），`.c` 中定义（放在文件顶部、所有函数之前，带初始化器）。

```c
// motor.h（声明）
extern volatile int Encoder_count1, Encoder_count2;
extern uint8_t motorA_dir, motorB_dir;

// motor.c（定义）
volatile int Encoder_count1 = 0;
volatile int Encoder_count2 = 0;
uint8_t motorA_dir = 1;
```

**`volatile` 使用**：**必须**用于 ISR 与主循环共享的变量（`Encoder_count1/2`、`pid_debug_*`）。⚠️ 当前 `motorA_dir`/`motorB_dir` 和 `pid_t` 结构体实例（`motorA`/`motorB`）被 ISR 和主循环共享但未标注 `volatile`——这是潜在隐患。

**外设索引数组**：用枚举索引的外设基址指针数组，在 `.c` 文件顶部定义：
```c
GPIO_TypeDef *gpio_index[3] = { GPIOA, GPIOB, GPIOC };
TIM_TypeDef *tim_index[3] = { TIM2, TIM3, TIM4 };
USART_TypeDef *uart_index[3] = { USART1, USART2, USART3 };
```

### 模块组织

每个模块 `.h` 按以下顺序组织：
1. 头文件保护 `#ifndef ... #define`
2. `#include`
3. 宏定义
4. 枚举 / 结构体 / typedef
5. 函数原型
6. `extern` 全局变量声明
7. `#endif`

每个模块 `.c` 按以下顺序组织：
1. `#include`（自己的 `.h` 或 `headfile.h`）
2. 模块私有宏
3. 全局变量定义（带初始化器）
4. 函数定义（通常按 `.h` 中声明顺序）

⚠️ 项目中**没有 `static` 函数**——所有函数都是全局可见的，没有模块私有辅助函数。

### 寄存器访问模式

全部**直接寄存器操作**，通过 CMSIS 结构体指针：`外设->REG = value`。

**通用模式**：外设基址放入 `_index[]` 指针数组，用枚举索引，避免 switch-case：
```c
tim_index[timn]->ARR = 1000000 / fre - 1;
gpio_index[GPIOn]->CRL &= ~((0x0000000F) << (Pinx * 4));
```

**位操作三板斧**：
1. **读-改-写**：`reg &= ~(mask << shift); reg |= (value << shift);`
2. **位测试**：`if (reg & mask)`、`while ((reg & mask) == 0);`
3. **写 1 清零**（EXTI）：`EXTI->PR = 1 << n;`

⚠️ 位掩码多为内联十六进制字面量（`0x202C`、`0x0F`），未定义为命名宏。

### 类型使用

| 用途 | 类型 | 示例 |
|------|------|------|
| 字节 / 标志 / 优先级 | `uint8_t` | `motorA_dir`、`Byte` |
| 传感器原始数据 | `int16_t` | `ax, ay, az, gx, gy, gz` |
| PWM 占空比 / ADC 值 | `uint16_t` | `duty`、`adc_get()` |
| 计数器 / 速度 / 通用 | `int` | `Encoder_count1`、`duty` 参数 |
| 寄存器暂存 | `uint32_t` | `temp` |
| 浮点运算 | `float`（**不用 double**） | 所有角度/滤波/PID 浮点值 |

⚠️ `gray_track.c` 中混用 `unsigned char` 和 `u8`（来自 `stm32f10x.h` 的别名），与项目主流的 `uint8_t` 不一致。

### ISR 编写规范

所有 ISR 集中在 [user/isr.c](user/isr.c)（13 个处理函数），函数名严格匹配 CMSIS 向量表名称。

**标准 ISR 模板**：
```c
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 1)           // 检查中断标志
    {
        pid_control();          // 处理
        TIM3->SR &= ~1;         // 清零标志
    }
}
```

**清零方式因外设而异**：
- TIM：`TIMx->SR &= ~1;`（位清零）
- USART：`USARTx->SR &= ~0x20;`（位清零）
- EXTI：`EXTI->PR = 1 << n;`（**写 1 清零**，不是 `&= ~`）

**编码器 ISR 方向判读**（[isr.c:69-79](user/isr.c#L69-L79)）：
```c
if (gpio_get(GPIO_A, Pin_3))
    Encoder_count1--;   // 高电平 = 反转
else
    Encoder_count1++;   // 低电平 = 正转
EXTI->PR = 1 << 2;
```

⚠️ `EXTI9_5_IRQHandler` 在 ISR 中做浮点 `atan()` 和卡尔曼滤波——不应模仿此做法，ISR 应尽量轻量。

### 初始化模式

**ml_libs 外设驱动遵循两阶段初始化**：
1. **内部引脚初始化**（用户不调用）：`uart_pin_init()`、`pwm_pin_init()`……函数注释标"内部调用 无需手动调用"
2. **公开初始化函数**：`uart_init(UART_1, 115200, 0)`、`pwm_init(TIM_2, TIM2_CH1, 1000)`
3. **使能/启动**（部分模块需要）：`tim_interrupt_ms_init(TIM_3, 10, 0)`

**结构体初始化**用行内注释标注字段（[filter.c:7-13](code/filter.c#L7-L13)）：
```c
KF_t KF_Yaw = {
    0.001,            // Q_angle
    0.003,            // Q_bias
    0.5,              // R
    {{1, 0}, {0, 1}}, // P[2][2]
    0.01              // dt
};
```

### 代码格式

- **缩进**：空格缩进（大部分文件 4 空格，`filter.c` 用 2 空格）
- **大括号**：K&R 风格（左括号与语句同行）
- **`if`/`for`/`while`**：关键字与 `(` 间加空格（`if (x)`）⚠️ `pid.c` 中有不加空格的例外（`if(spe1 >= 0)`）
- **单行 if**：有时加括号有时不加，不强制
- **`switch-case`**：`case` 与 `switch` 同级缩进，case 体缩进一层

### 错误处理

⚠️ 项目中**没有错误处理机制**——无返回值检查、无 `assert()`、无看门狗。I2C 的 `I2C_WaitAck()` 返回 ACK 位但所有调用方忽略返回值。

### 魔数约定

**公共常量**定义为 `.h` 中的宏 → `MAX_DUTY 50000`、`MPU6050_ADDR 0xd0`

**模块私有常量**定义为 `.c` 中的宏 → `PID_TEST_MAX_DUTY 30000`、`PCLK1 36000000`

**控制参数和算法常量**：大多数是内联字面量——PID 增益（`500, 10, 0`）、互补滤波系数（`0.98f`、`0.02f`）、校准样本数（`500`）、弧度转换（`57.29578f`）。添加新参数时优先定义为命名宏。

### 已知不一致之处（新代码应规避）

| 问题 | 位置 | 建议做法 |
|------|------|----------|
| 头文件保护格式不统一（4 种写法） | 多个 `.h` | 统一用 `_模块名_h_` |
| `alpha` 宏小写 | [filter.c:4](code/filter.c#L4) | 改为 `ALPHA` |
| `Encoder_count1` PascalCase | [motor.c:8](code/motor.c#L8) | 应为 `encoder_count1` |
| `unsigned char` / `u8` 混用 | [gray_track.c](code/gray_track.c) | 统一用 `uint8_t` |
| `void f()` vs `void f(void)` | motor.c/pid.c 等 | 统一用 `(void)` |
| `pid.c` 不 include 自己的 `.h` | [pid.c:1](code/pid.c#L1) | 应 `#include "pid.h"` |
| `motorA_dir` 缺 `volatile` | [motor.c:5](code/motor.c#L5) | ISR 共享变量应加 `volatile` |
| ISR 中做浮点运算 | [isr.c:103-145](user/isr.c#L103-L145) | ISR 应只设标志位，主循环做计算 |

---

## 控制架构

### 当前活跃：纯速度闭环

```
motor_target_set(2.05f, 2.0f)
    → motorA.target / motorB.target（绝对值） + motorX_dir（方向位）
    → 10ms TIM3 ISR 调用 pid_control()
    → 读 Encoder_count1/2（EXTI 累计脉冲）, 按方向取符号 → motorX.now
    → 增量式 PID 计算 (P=500, I=10, D=0)
    → 输出限幅 [0, 30000]
    → motorA_duty() / motorB_duty() → TIM2 CCR → TB6612 → 直流电机
    → 编码器 EXTI (PA2/PA4 下降沿) → 方向检测 (PA3/PA5) → Encoder_count 更新
```

### 设计目标：级联 PID（待接入）

```
目标角度 (-20°)  ──→ [位置式 PID] ──→ 目标速度 ──→ [增量式 PID ×2] ──→ PWM 驱动电机
       ↑                angle               ↑              motorA/B
       │                                     │
  卡尔曼滤波 ←── MPU6050 + HMC5883L     编码器脉冲计数 (EXTI)
```

- **外环** (位置式 PID)：角度控制 — `pid_t angle` 已声明，参数待定
- **内环** (增量式 PID)：速度控制 — **已调试完成并运行中**
- 传感器融合代码已存在但（1）MPU6050/HMC5883L 未初始化，（2）卡尔曼输出未接入控制回路

---

## 完整引脚映射

| 功能 | 引脚/外设 | 说明 |
|------|-----------|------|
| 电机 A PWM | PA0 / TIM2_CH1 | 1000Hz, TB6612 PWMA |
| 电机 B PWM | PA1 / TIM2_CH2 | 1000Hz, TB6612 PWMB |
| 电机 A 方向 | PA6 (AIN1), PA7 (AIN2) | GPIO 推挽输出 |
| 电机 B 方向 | PB0 (BIN1), PB1 (BIN2) | GPIO 推挽, **代码已按实际接线反转** |
| 编码器 A 脉冲 | PA2 / EXTI2 (下降沿) | 左轮 |
| 编码器 A 方向 | PA3 (GPIO 输入上拉) | 判向: 高→反转, 低→正转 |
| 编码器 B 脉冲 | PA4 / EXTI4 (下降沿) | 右轮 |
| 编码器 B 方向 | PA5 (GPIO 输入上拉) | 判向: 高→正转, 低→反转 |
| 传感器 I2C | PB10 (SCL), PB11 (SDA) | 软件 I2C, MPU6050 + HMC5883L 共用 |
| OLED I2C | PB8 (SCL), PB9 (SDA) | 独立软件 I2C, SSD1306 |
| MPU6050 INT | PB7 / EXTI (上升沿) | 触发传感器数据读取 + 卡尔曼滤波 |
| 灰度传感器 D1-D4 | PB12, PB13, PB14, PB15 | GPIO 输入上拉 |
| 灰度传感器 D5-D8 | PA8, PC13, PC14, PC15 | GPIO 输入上拉 |
| 调试串口 TX | PA9 / USART1 | 115200, 8N1 |
| 调试串口 RX | PA10 / USART1 | 当前代码未使用 |

**硬件注意**：TB6612 的 STBY 必须保持高电平；STM32 与电机电源必须共地。

---

## 关键源文件速查

| 文件 | 作用 | 活跃 |
|------|------|:----:|
| [user/main.c](user/main.c) | 入口: 硬件初始化, PID 初始化, OLED/UART 调试循环 | ✅ |
| [user/isr.c](user/isr.c) | 所有 ISR: TIM3 PID 主控, 编码器 EXTI, 传感器 EXTI9_5 | ✅ |
| [user/attitude.c](user/attitude.c) | MPU 校准, 互补滤波 roll/pitch, 磁力计倾斜补偿 yaw | ❌ |
| [code/pid.c](code/pid.c) | PID 结构体, pid_control() 主控循环, 输出限幅, 调试快照 | ✅ |
| [code/motor.c](code/motor.c) | PWM + 方向映射, 编码器引脚初始化, motorB_stop() | ✅ |
| [code/filter.c](code/filter.c) | 3 个卡尔曼滤波实例 (KF_Roll/Pitch/Yaw), Mahony 函数 | ⚠️ ISR 用卡尔曼, Mahony 未用 |
| [code/gray_track.c](code/gray_track.c) | 8 通道查表法循迹差速控制 | ❌ |
| [ml_libs/ml_pwm.c](ml_libs/ml_pwm.c) | TIM2/3/4 PWM 初始化和占空比更新 | ✅ |
| [ml_libs/ml_exti.c](ml_libs/ml_exti.c) | PA0-PC7 外部中断初始化 | ✅ |
| [ml_libs/ml_i2c.c](ml_libs/ml_i2c.c) | 软件 I2C 位操作 (PB10/PB11) | ✅ (传感器 ISR 使用) |
| [ml_libs/ml_mpu6050.c](ml_libs/ml_mpu6050.c) | MPU6050 寄存器读写 | ✅ (传感器 ISR 使用) |
| [ml_libs/ml_hmc5883l.c](ml_libs/ml_hmc5883l.c) | HMC5883L 寄存器读写 | ✅ (传感器 ISR 使用) |
| [ml_libs/ml_oled.c](ml_libs/ml_oled.c) | SSD1306 OLED 驱动 (PB8/PB9 独立 I2C) | ✅ |
| [ml_libs/ml_uart.c](ml_libs/ml_uart.c) | USART1/2/3 初始化和收发 (含 printf 重定向) | ✅ |

---

## `motor_target_set()` 行为

```c
void motor_target_set(float spe1, float spe2);  // [pid.c:35](code/pid.c#L35)
```

- 接受 **float** 参数（非 int），支持小幅速度微调
- **正数** → 设置 `motorX_dir = 1`（前进），`motorX.target = speX`（绝对值）
- **负数** → 设置 `motorX_dir = 0`（后退），`motorX.target = -speX`（绝对值）
- `pid_control()` 中按方向位给编码器计数值赋符号：前进 → `+Encoder_count`，后退 → `-Encoder_count`
- 多次调用会覆盖之前的目标值

### Motor B 零目标特殊处理

[pid_control()](code/pid.c#L79-L95) 中对 motorB 有特殊逻辑：

- 当 `motorB.target == 0`：清零所有 PID 状态（error 历史、pout/iout/dout、out）并调用 `motorB_stop()`（拉低 BIN1/BIN2，PWM=0）
- 当 `motorB.target != 0`：正常 PID 计算

Motor A 没有此特殊处理（即使 target=0 也会正常走 PID 计算）。

### 方向引脚极性差异

- `motorA_duty()`: `PA6 = motorA_dir`, `PA7 = !motorA_dir`
- `motorB_duty()`: `PB0 = !motorB_dir`, `PB1 = motorB_dir` — **B 方向引脚逻辑取反**，补偿物理接线差异

---

## PWM 架构

- TIM2 同时驱动两路电机 PWM：CH1 = PA0 (Motor A), CH2 = PA1 (Motor B)
- 频率 1000Hz (`pwm_init(TIM_2, channel, 1000)`)
- `MAX_DUTY = 50000`（抽象占空比量程），`pwm_update()` 内部映射：`CCR = duty / 50000 * (ARR+1)`
- ARR = 999 (1000000/1000 - 1)，CCR 全量程约 0~1000
- `PID_TEST_MAX_DUTY = 30000` 限制 PID 输出，所以实际 PWM 范围是 0~60% 全量程
- `MOTOR_B_PWM_INVERTED` 宏（[motor.c:3](code/motor.c#L3)）当前为 `0`，即不反转 B 通道 PWM

---

## OLED 调试显示

正常闭环模式下，OLED 显示如下（[main.c](user/main.c) while 循环）：

```
A T:0002 N:+002
A D:1 O:11960
B T:0002 N:+002
B C:0238 O:11900
```

| 字段 | 含义 |
|------|------|
| `T` | 目标速度，按整数显示（A 的 2.05 显示为 `0002`） |
| `N` | 最近 10ms 内**有符号**编码器计数，即速度反馈 |
| `D` | Motor A 逻辑方向位（1=前进，0=后退） |
| `O` | PID 输出值，范围 0~30000 |
| `C` | TIM2 CCR2 寄存器值（B 通道实际 PWM 比较值），用于验证 PWM 硬件 |

**正常现象**：`N` 在目标值附近小幅波动（如 +001~+003）。若 `O` 长期饱和到 30000 但 `N` 无法回到目标，首先检查机械负载、供电和电机方向极性，而非继续加大积分参数。

---

## B PWM 硬件诊断开关

[main.c](user/main.c) 第 3 行：

```c
#define B_PWM_PIN_TEST 0
```

- **0** — 正常闭环运行（双轮速度 PID）
- **1** — 硬件旁路模式：PA1 直接拉低为 GPIO，B 方向输入固定，TIM2 被跳过。TB6612 正常时 B 电机必须停止。OLED 显示 "B PWM PIN TEST" 模式提示。

仅用于排查 PWMB/驱动器硬件故障。

---

## 构建与烧录

1. 用 Keil uVision 5 打开 [user/Project.uvprojx](user/Project.uvprojx)
2. 编译目标：STM32F103C8
3. 编译器：ARMCC V5.06
4. 输出：[user/Objects/Project.hex](user/Objects/Project.hex)
5. 通过 ST-Link / J-Link 烧录到 STM32F103C8T6

---

## VS Code 开发注意事项

- [`.vscode/c_cpp_properties.json`](.vscode/c_cpp_properties.json) 仅供 IntelliSense 使用（代码提示、跳转），**不影响 Keil 编译**
- `compilerPath` 设为 `D:/MinGW64/bin/gcc.exe` 仅用于消除 IntelliSense 警告，实际编译仍用 Keil 的 armcc
- 项目无 Makefile/CMake，**不能在 VS Code 中直接编译**
- `includePath` 使用 `${workspaceFolder}/../ml_libs` 等相对路径，因为 VS Code workspace 是 `user/` 目录

---

## 已知问题 / 踩坑记录

1. **MPU6050 和 HMC5883L 未初始化**：`MPU6050_Init()` 和 `HMC5883L_Init()` 在 main.c 中从未调用。传感器 ISR (EXTI9_5) 虽然会读取寄存器并跑卡尔曼滤波，但读到的可能是上电默认值或上次烧录遗留的配置值。如果要重新接入传感器融合，必须先在 main.c 中添加这两个初始化调用。

2. **传感器融合代码有两套重复实现**：ISR 中内联计算（atan + Kalman）和 [attitude.c](user/attitude.c) 中的互补滤波是完全独立的两套管线。需要选定一套并统一。

3. **Motor B 零目标会硬停车**：`motorB.target == 0` 时 pid_control() 会清零 PID 状态并刹车。从正常速度直接设 target=0 可能导致急停。

4. **B 通道极性**：motorB 的方向引脚逻辑与 motorA 相反（`PB0 = !motorB_dir`），如果更换电机驱动板或重新接线，需要检查此映射。

5. **OLED 和传感器使用不同的 I2C 总线**：OLED 在 PB8/PB9，传感器在 PB10/PB11。这样设计是为了避免 OLED 刷新阻塞传感器读取。修改 I2C 相关代码时注意不要混用。
