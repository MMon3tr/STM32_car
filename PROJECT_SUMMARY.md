# STM32 小车项目速读总结

这份文档是给“以后第一次打开项目的人/助手”看的。先读它，再看源码，会快很多。

## 一句话概览

这是一个基于 STM32F103 的两轮小车工程，使用 Keil MDK/ARMCC V5 编译，不使用 HAL/Cube，外设基本都是寄存器级自写驱动。代码目标是把电机、编码器、灰度循迹、MPU6050、HMC5883L、OLED、串口调试和 PID 控制串起来，形成小车运动控制闭环。

## 当前最重要的状态

当前 `user/main.c` 更像是硬件测试状态，不是完整闭环状态：

- 初始化了 OLED、电机、编码器、串口、PID、灰度、I2C、MPU6050、HMC5883L、PB7 外部中断。
- `tim_interrupt_ms_init(TIM_3, 10, 0);` 被注释掉，所以 TIM3 周期控制中断当前没有启动。
- 主循环里只执行 `motorA_duty(10000);`，也就是固定给 A 电机一个 PWM。
- `Attitude_Update()`、`HMC5883L_UpdateYaw()`、OLED 显示、调试输出等都处于注释状态。
- `pid_control()` 已写在 `TIM3_IRQHandler()` 里，但因为 TIM3 初始化被注释，当前默认不会周期执行。

如果要恢复“闭环控制/循迹/姿态控制”，第一时间检查 `user/main.c` 和 `user/isr.c`，尤其是 TIM3 初始化、主循环测试代码、`pid_control()` 中的控制目标。

## 目录分工

```text
car_example/
  user/
    main.c          程序入口，负责初始化和主循环
    isr.c           中断服务函数：定时器、串口、编码器、姿态触发
    attitude.c/h    姿态解算辅助：MPU6050 零偏校准、roll/pitch/yaw 计算
    Project.uvprojx Keil 工程文件

  code/
    motor.c/h       电机方向、PWM 输出、编码器 EXTI 初始化与计数
    pid.c/h         位置式/增量式 PID，以及级联控制逻辑
    gray_track.c/h  8 路灰度循迹传感器读取和差速决策
    filter.c/h      互补滤波、卡尔曼滤波

  ml_libs/
    headfile.h      总头文件，几乎所有业务代码都会 include 它
    ml_gpio.*       GPIO 寄存器封装
    ml_pwm.*        TIM2/3/4 PWM 输出，MAX_DUTY = 50000
    ml_tim.*        TIM2/3/4 定时器中断初始化
    ml_exti.*       外部中断初始化
    ml_uart.*       USART1/2/3，printf 重定向到 USART1
    ml_i2c.*        软件 I2C，默认 PB10=SCL、PB11=SDA
    ml_mpu6050.*    MPU6050 加速度计/陀螺仪驱动
    ml_hmc5883l.*   HMC5883L 磁力计驱动
    ml_oled.*       OLED 驱动，独立软件 I2C，默认 PB8=SCL、PB9=SDA
    ml_delay.*      SysTick 延时
    ml_adc.*        ADC 封装
    ml_nvic.*       NVIC 优先级封装

  sys/
    CMSIS、启动文件、系统时钟配置等 STM32 基础文件
```

## 启动流程

`main()` 的初始化顺序大致是：

1. `OLED_Init()`，OLED 第 2 行显示 `Ciallo`。
2. `motor_init()`，初始化两路电机 PWM 和方向 GPIO。
3. `encoder_init()`，初始化两个编码器的外部中断计数。
4. `uart_init(UART_1, 115200, 0)`，打开 USART1 调试串口。
5. 初始化三个 PID：
   - `motorA`：增量式 PID，参数 `10, 10, 5`
   - `motorB`：增量式 PID，参数 `10, 10, 5`
   - `angle`：位置式 PID，参数 `2, 0, 0.5`
6. `gray_init()`，初始化 8 路灰度传感器输入。
7. `I2C_Init()`、`MPU6050_Init()`、`HMC5883L_Init()`。
8. `exti_init(EXTI_PB7, RISING, 0)`，用 PB7 上升沿触发姿态数据读取/融合。
9. `MPU6050_Calibrate()`，采 500 次陀螺仪数据计算零偏。
10. 进入 `while(1)`，当前只固定输出 A 电机 PWM。

## 控制链路

项目里已经写好了一个级联 PID 框架：

```text
目标角度
  -> angle 位置式 PID
  -> 左右轮目标速度
  -> motorA/motorB 增量式 PID
  -> PWM 占空比
  -> 电机
  -> 编码器反馈速度
```

入口在 `code/pid.c` 的 `pid_control()`：

- 角度环：`angle.target = -20;`，当前角度使用 `yaw_Kalman`。
- 角度 PID 输出：`pid_cal(&angle)`。
- 电机目标速度：`motor_target_set(-angle.out, angle.out)`。
- 编码器反馈：`Encoder_count1`、`Encoder_count2` 在 EXTI 中断里累加，PID 周期内读完后清零。
- 电机 PID：`pid_cal(&motorA)`、`pid_cal(&motorB)`。
- 输出限幅：`pidout_limit()` 限制到 `0 ~ MAX_DUTY`。
- 最终输出：`motorA_duty(motorA.out)`、`motorB_duty(motorB.out)`。

注意：`pid_control()` 目前设计成由 TIM3 中断周期调用。要让它跑起来，需要在 `main()` 里恢复 TIM3 初始化，比如原来注释掉的 `tim_interrupt_ms_init(TIM_3, 10, 0);`。

## 姿态链路

姿态相关数据分两套路径：

1. `user/isr.c` 的 `EXTI9_5_IRQHandler()` 中，PB7 上升沿触发后：
   - 读取 `MPU6050_GetData()`
   - 读取 `HMC5883L_GetData()`
   - 用陀螺仪积分得到 `roll_gyro/pitch_gyro/yaw_gyro`
   - 用加速度计估算 `roll_acc/pitch_acc/yaw_acc`
   - 用磁力计估算 `yaw_hmc`
   - 用 `Kalman_Filter()` 得到 `roll_Kalman/pitch_Kalman/yaw_Kalman`

2. `user/attitude.c` 里还有一套手动更新函数：
   - `MPU6050_Calibrate()`：陀螺仪零偏校准
   - `Attitude_Update()`：互补滤波计算 roll/pitch
   - `HMC5883L_UpdateYaw()`：磁力计倾斜补偿并融合 yaw

当前 `main()` 中手动调用 `Attitude_Update()`、`HMC5883L_UpdateYaw()` 的代码被注释；实际姿态更新主要看 PB7 外部中断是否正常触发。

## 灰度循迹链路

`code/gray_track.c` 管 8 路数字灰度传感器：

- D1-D4：PB12、PB13、PB14、PB15
- D5-D8：PA8、PC13、PC14、PC15
- `digtal(channel)` 读取对应通道电平。
- `track()` 根据 D1-D8 的组合直接设置左右轮目标速度。

`track()` 已经写好，但在 `pid_control()` 里目前被注释掉。若要做循迹，需要决定它是直接接管速度目标，还是和角度环/姿态环组合。

## 关键引脚表

| 功能 | 引脚/外设 | 代码位置 | 说明 |
| --- | --- | --- | --- |
| 电机 A PWM | TIM2_CH1 / PA0 | `motor_init()` | 1000 Hz |
| 电机 B PWM | TIM2_CH2 / PA1 | `motor_init()` | 1000 Hz |
| 电机 A 方向 | PA6、PA7 | `motorA_duty()` | `motorA_dir` 控制正反转 |
| 电机 B 方向 | PB0、PB1 | `motorB_duty()` | `motorB_dir` 控制正反转 |
| 编码器 A | PA2 外部中断，PA3 判向 | `encoder_init()` / `EXTI2_IRQHandler()` | 下降沿计数 |
| 编码器 B | PA4 外部中断，PA5 判向 | `encoder_init()` / `EXTI4_IRQHandler()` | 下降沿计数 |
| 姿态触发 | PB7 外部中断 | `main()` / `EXTI9_5_IRQHandler()` | 上升沿读取并融合 IMU/磁力计 |
| 软件 I2C | PB10=SCL，PB11=SDA | `ml_i2c.h` | MPU6050/HMC5883L 使用 |
| OLED I2C | PB8=SCL，PB9=SDA | `ml_oled.h` | OLED 单独一套软件 I2C |
| 灰度 D1-D4 | PB12-PB15 | `gray_init()` | 上拉输入 |
| 灰度 D5-D8 | PA8、PC13-PC15 | `gray_init()` | 上拉输入 |
| 调试串口 | USART1 PA9/PA10 | `ml_uart.c` | 115200，printf 输出到 USART1 |

## 中断入口

- `TIM3_IRQHandler()`：计划中的 PID 周期控制入口，内部调用 `pid_control()`。
- `EXTI2_IRQHandler()`：编码器 A 计数。
- `EXTI4_IRQHandler()`：编码器 B 计数。
- `EXTI9_5_IRQHandler()`：
  - PB7：MPU6050 + HMC5883L 数据读取和卡尔曼滤波
  - 其他 EXTI5/6/8/9 分支目前为空
- USART1/2/3 中断入口已占位，但当前没有实际接收处理逻辑。

## 修改时最容易踩的点

- `main.c` 当前是测试模式。不要误以为完整闭环已经在跑。
- `pid_control()` 依赖定时器周期。如果没有启动 TIM3，它不会自动执行。
- `pidout_limit()` 把 PID 输出限制到非负数；方向由 `motor_target_set()` 里的 `motorA_dir/motorB_dir` 决定。
- `motor_target_set()` 会把负目标速度转成方向位 + 正目标值，所以后续读编码器反馈时也按方向恢复正负。
- 姿态更新有两套写法：PB7 中断里的卡尔曼融合，以及 `attitude.c` 里的互补融合。改控制算法前要先选定一套主路径，避免两边变量含义混用。
- OLED 和通用 I2C 使用不同引脚、不同软件 I2C 实现，不要把 PB8/PB9 和 PB10/PB11 搞混。
- 源码里部分中文注释可能因为编码问题显示乱码。建议新增文档使用 UTF-8；除非必要，不要批量重编码源码文件。

## 建议的下一步阅读顺序

1. `user/main.c`：确认当前到底初始化了什么、主循环在做什么。
2. `user/isr.c`：看中断是否真正驱动了控制和姿态更新。
3. `code/motor.c`：确认电机 PWM、方向、编码器计数。
4. `code/pid.c`：理解级联 PID 和当前目标值。
5. `code/gray_track.c`：理解灰度循迹如何给速度目标。
6. `ml_libs/ml_i2c.h`、`ml_libs/ml_oled.h`、`ml_libs/ml_pwm.h`：确认关键引脚和占空比范围。

## 如果要把车重新跑起来

优先按这个顺序排查：

1. 单独测试电机方向和 PWM：确认 `motorA_duty()`、`motorB_duty()` 与实际左右轮一致。
2. 单独测试编码器方向：手转轮子，看 `Encoder_count1/2` 正负是否符合预期。
3. 恢复 TIM3 控制周期：让 `pid_control()` 按固定周期运行。
4. 先只做速度闭环：暂时固定左右轮目标速度，调 `motorA/motorB` PID。
5. 再接入灰度循迹或姿态角度环，不要同时引入多个变量。
6. 最后再调 `angle` PID 和 `yaw_Kalman`/`yaw` 的选择。

