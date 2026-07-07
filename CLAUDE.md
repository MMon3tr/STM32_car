# CLAUDE.md — STM32 平衡小车 (car_example)

> 电赛 STM32 两轮自平衡小车项目。基于 STM32F103C8T6，Keil MDK (ARMCC V5) 编译，无 HAL/Cube 库，纯寄存器级自研驱动。

---

## 项目概览

- **MCU**: STM32F103C8T6 (Cortex-M3, 128KB Flash, 20KB SRAM, 72MHz)
- **编译器**: ARMCC V5.06 (Keil MDK uVision 5)
- **启动文件**: `startup_stm32f10x_md.s` (中等容量)
- **C 标准**: C99, Optimization Level 1
- **库依赖**: 无。纯寄存器操作，不用 ST 标准外设库/HAL/LL
- **调试器配置**: 项目中有 STM32F103C6、C8、C8_1.0.0 三套 DebugConfig

---

## 目录结构

```
car_example/
├── user/                    # 应用层入口
│   ├── main.c               # 主函数 (硬件初始化 + 主循环)
│   ├── isr.c                # 所有中断服务函数
│   ├── attitude.c/h         # 姿态解算 (欧拉角 + 磁力计校准)
│   ├── Project.uvprojx      # Keil 工程文件
│   └── Objects/ Listings/ DebugConfig/  # 构建产物
├── code/                    # 通用算法与驱动
│   ├── motor.c/h            # 直流电机 + 编码器驱动
│   ├── pid.c/h              # PID 控制器 (位置式 + 增量式)
│   ├── filter.c/h           # 卡尔曼滤波 + Mahony 互补滤波
│   └── gray_track.c/h       # 8 路灰度循迹传感器
├── ml_libs/                 # 自研硬件抽象层 (寄存器级)
│   ├── headfile.h           # 总头文件 (所有 .c 都 include 它)
│   ├── ml_gpio.c/h          # GPIO (CRL/CRH/ODR/IDR)
│   ├── ml_delay.c/h         # SysTick 延时 (us/ms/s)
│   ├── ml_pwm.c/h           # PWM (TIM2/3/4, MAX_DUTY=50000)
│   ├── ml_tim.c/h           # 定时器中断 (ms 级周期)
│   ├── ml_uart.c/h          # 串口 (USART1/2/3)
│   ├── ml_adc.c/h           # ADC1/2
│   ├── ml_exti.c/h          # 外部中断 (PA0-PC7 全覆盖)
│   ├── ml_nvic.c/h          # NVIC 中断优先级分组
│   ├── ml_i2c.c/h           # 软件 I2C (PB10=SCL, PB11=SDA)
│   ├── ml_mpu6050.c/h       # MPU6050 六轴 IMU 驱动
│   ├── ml_hmc5883l.c/h      # HMC5883L 三轴磁力计驱动
│   ├── ml_oled.c/h          # SSD1306 OLED 驱动 (I2C, PB8/PB9)
│   └── ml_oled_font.h       # OLED 字库
├── sys/                     # CMSIS 系统层
│   ├── core_cm3.h           # ARM CMSIS Core (Cortex-M3)
│   ├── core_cmFunc.h / core_cmInstr.h
│   ├── stm32f10x.h          # STM32F10x 外设寄存器定义
│   ├── system_stm32f10x.c/h # 时钟初始化 (HSE 8MHz → PLL×9 → 72MHz)
│   ├── startup_stm32f10x_md.s  # 中等容量启动文件 (当前使用)
│   └── startup_stm32f10x_hd.s  # 高容量启动文件 (备用)
├── .vscode/                 # VS Code C/C++ 扩展配置 (仅 IntelliSense)
│   └── c_cpp_properties.json
├── .gitignore
└── scripts/                 # (空)
```

---

## 核心架构

### 分层架构

```
Layer 4: user/      — main.c, isr.c, attitude.c       (应用入口)
Layer 3: code/      — motor, pid, filter, gray_track   (算法/驱动)
Layer 2: ml_libs/   — GPIO, PWM, I2C, UART, ADC...     (硬件抽象)
Layer 1: sys/       — startup, system_stm32f10x        (CMSIS)
```

所有源文件通过 `#include "headfile.h"` 引入一切（CMSIS → ML 库 → 应用层）。

### 控制架构：级联 PID

```
目标角度 (-20°)  ──→ [位置式 PID] ──→ 目标速度 ──→ [增量式 PID x2] ──→ PWM 驱动电机
       ↑                angle               ↑              motorA/B
       │                                     │
  卡尔曼滤波 ←── MPU6050 + HMC5883L     编码器脉冲计数 (EXTI)
```

- **外环 (位置式 PID)**：角度控制，`pid_init(&angle, POSITION_PID, 2, 0, 0.5)`
- **内环 (增量式 PID)**：速度控制，`pid_init(&motorA/B, DELTA_PID, 10, 10, 5)`
- PID 控制周期：TIM3 中断驱动，在 `isr.c` 的 `TIM3_IRQHandler` 中调用 `pid_control()`

### 传感器融合 (isr.c → PB7 上升沿 EXTI)

每次 PB7 收到上升沿触发时：
1. 读取 MPU6050 原始数据
2. 读取 HMC5883L 磁力计数据
3. `attitude_update()` — 互补滤波：陀螺仪 98% + 加速度计 2% → Roll/Pitch
4. 磁力计倾斜补偿 + 互补滤波 (95% 陀螺仪 + 5% 磁力计) → Yaw
5. 卡尔曼滤波 (`filter.c`) 融合得到最终姿态角

### 循迹控制 (gray_track.c)

8 路灰度传感器 (D1-D8) 通过查表法映射到左右轮差速，叠加在角度环输出上。当前已实现但注释掉了。

---

## 关键引脚映射

| 功能 | 引脚 | 说明 |
|------|------|------|
| 电机 A PWM | TIM2 CH1 (PA0) | 1000Hz |
| 电机 B PWM | TIM2 CH2 (PA1) | 1000Hz |
| 电机 A 方向 | PA6, PA7 | GPIO |
| 电机 B 方向 | PB0, PB1 | GPIO |
| 编码器 A | EXTI2 (PA2), PA3 (方向) | 脉冲计数 |
| 编码器 B | EXTI4 (PA4), PA5 (方向) | 脉冲计数 |
| I2C SCL/SDA | PB10, PB11 | 软件 I2C |
| OLED | PB8, PB9 | I2C SSD1306 |
| 传感器触发 | EXTI PB7 (上升沿) | 触发姿态读取 |
| 灰度 D1-D4 | PB12-PB15 | 循迹传感器 |
| 灰度 D5-D8 | PA8, PC13-PC15 | 循迹传感器 |
| 调试串口 | USART1 | 115200 baud |

---

## 当前代码状态

`main()` **当前处于测试模式**：只执行 `motorA_duty(10000)` 驱动单个电机。
以下功能已实现但被注释掉了：
- `pid_control()` — 级联 PID 主控
- `track()` — 灰度循迹
- 完整的传感器→滤波→PID→电机闭环

这是因为项目从工作仓库迁出时保留了测试状态，正常运行时需要取消注释 `main()` 底部的被封装的循环代码。

---

## 构建与烧录

1. 用 Keil uVision 5 打开 `user/Project.uvprojx`
2. 编译目标：STM32F103C8
3. 编译器：ARMCC V5.06
4. 输出：`user/Objects/Project.hex`
5. 通过 ST-Link / J-Link 烧录到 STM32F103C8T6

## VS Code 开发注意事项

- `.vscode/c_cpp_properties.json` 仅供 IntelliSense 使用（代码提示、跳转），**不影响 Keil 编译**
- `compilerPath` 设为 `D:/MinGW64/bin/gcc.exe` 仅用于消除 IntelliSense 警告，实际编译仍用 Keil 的 armcc
- 项目无 Makefile/CMake，不能在 VS Code 中直接编译
