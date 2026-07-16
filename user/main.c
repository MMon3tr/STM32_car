#include "headfile.h"

#define B_PWM_PIN_TEST 0
#define B_PWM_TEST_LEVEL 0

/* === 安全约束 (Step 0 定义, 后续步骤使用) === */
#define MAX_SPEED_TARGET    100.0f   /* 速度目标上限 (脉冲/10ms)      */
#define MAX_DELTA_V          30.0f   /* 差速补偿上限 (脉冲/10ms)      */
#define MAX_TILT_ANGLE       45.0f   /* 倾角过大停车阈值 (度)         */
#define ANGLE_CTRL_PERIOD_MS 40      /* 角度环控制周期 (ms), 25Hz    */

/* 陀螺比例补偿: GYRO_CONFIG=0x18(±2000dps) 但实测积分量偏小3.72x
 * 系数 = 360°物理 / yaw_gyro实测Δ = 360/96.8 ≈ 3.72
 * 若后续发现根因, 改回1.0即可 */
#define GYRO_SCALE_COMPENSATION  (3.85f)

int main(void)
{
	OLED_Init();
	OLED_Clear();

	motor_init();

#if B_PWM_PIN_TEST
	/* Direct PA1 test: bypass TIM2 and the PID controller. */
	gpio_init(GPIO_A, Pin_1, OUT_PP);
	gpio_set(GPIO_A, Pin_1, B_PWM_TEST_LEVEL);
	gpio_set(GPIO_B, Pin_0, 0);
	gpio_set(GPIO_B, Pin_1, 1);

	OLED_ShowString(1, 1, "B PWM PIN TEST");
	OLED_ShowString(2, 1, "PA1 LEVEL:");
	OLED_ShowNum(2, 12, B_PWM_TEST_LEVEL, 1);
	OLED_ShowString(3, 1, "BIN1:0 BIN2:1");
	OLED_ShowString(4, 1, "EXPECT: STOP");

	while (1)
	{
	}
#else
	/* === Step 0: 传感器初始化 === */
	I2C_Init();                       /* PB10/PB11 配置为开漏输出           */
	MPU6050_Init();                   /* 唤醒芯片, ±2000dps, ±2g, 使能INT  */
	HMC5883L_Init();                  /* 连续测量模式, 75Hz                 */
	delay_ms(100);                    /* 等待传感器稳定                     */
	MPU6050_Calibrate();              /* 1秒静止校准陀螺零偏 (500样本)      */
	/* exti_init(EXTI_PB7, RISING, 0); -- 禁用: ISR与主循环共享I2C总线会冲突   */
	/* 传感器数据由主循环中Attitude_Update() + HMC5883L_UpdateYaw()采集 */

	encoder_init();
	uart_init(UART_1, 115200, 0);

	/* 回读MPU6050和HMC5883L关键寄存器, 验证初始化是否成功 */
	{
		uint8_t who_am_i   = MPU6050_Read(0x75);
		uint8_t gyro_cfg   = MPU6050_Read(0x1B);  /* GYRO_CONFIG     */
		uint8_t accel_cfg  = MPU6050_Read(0x1C);  /* ACCEL_CONFIG    */
		uint8_t pwr_mgmt   = MPU6050_Read(0x6B);  /* PWR_MGMT_1      */
		uint8_t hmc_cra    = HMC5883L_Read(0x00); /* Config Reg A    */
		uint8_t hmc_crb    = HMC5883L_Read(0x01); /* Config Reg B    */
		uint8_t hmc_mr     = HMC5883L_Read(0x02); /* Mode Register   */
		uint8_t hmc_status = HMC5883L_Read(0x09); /* Status Register */
		printf("\r\n=== Step 0: Sensors Initialized ===\r\n");
		printf("MPU6050: WHO_AM_I=0x%02X (exp 0x68) "
		       "GYRO=0x%02X (exp 0x18) ACCEL=0x%02X (exp 0x00) "
		       "PWR=0x%02X\r\n",
		       who_am_i, gyro_cfg, accel_cfg, pwr_mgmt);
		printf("HMC5883L: CRA=0x%02X (exp 0xF8) CRB=0x%02X (exp 0x20) "
		       "MR=0x%02X (exp 0x00) STATUS=0x%02X\r\n",
		       hmc_cra, hmc_crb, hmc_mr, hmc_status);
		printf("Gyro bias: gx=%.1f gy=%.1f gz=%.1f\r\n",
		       gx_bias, gy_bias, gz_bias);
	}

	pid_init(&motorA, DELTA_PID, 500, 10, 0);
	pid_init(&motorB, DELTA_PID, 500, 10, 0);

	/* Dual-wheel trim: A is 2.05 and B is 2.0 pulses per 10 ms. */
	motor_target_set(2.05f, 2.0f);
	tim_interrupt_ms_init(TIM_3, 10, 0);

	while (1)
	{
		static uint8_t print_count = 0;
		static uint8_t display_mode = 0;        /* 0=速度环, 1=姿态    */
		static uint8_t display_toggle_cnt = 0;

		/* --- 100Hz姿态更新 + yaw陀螺积分 (10次x10ms) --- */
		for (int i = 0; i < 10; i++)
		{
			Attitude_Update();  /* roll, pitch互补滤波; 更新gx,gy,gz */
			/* 同步积分yaw陀螺: gz已由Attitude_Update内的MPU6050_GetData更新 */
			yaw_gyro += (gz - gz_bias) / 16.4f * 0.01f * GYRO_SCALE_COMPENSATION;
			delay_ms(10);
		}

		/* --- 读取磁力计 + 计算yaw (每次外循环, ~10Hz) --- */
		HMC5883L_GetData();
		{
			float yaw_hmc_raw = atan2((float)hmc_y, (float)hmc_x) * 57.29578f;
			/* 互补融合: 95%陀螺 + 5%磁力计 */
			yaw = 0.95f * yaw_gyro + 0.05f * yaw_hmc_raw;
			/* 仅输出包裹到[-180,180], 不改yaw_gyro累加器 */
			while (yaw > 180.0f)  yaw -= 360.0f;
			while (yaw < -180.0f) yaw += 360.0f;
		}

		/* --- 每2秒切换OLED显示模式 --- */
		if (++display_toggle_cnt >= 20)
		{
			display_toggle_cnt = 0;
			display_mode = !display_mode;
			OLED_Clear();
		}

		if (display_mode == 0)
		{
			/* 屏幕0: 速度环数据 */
			OLED_ShowString(1, 1, "A T:");
			OLED_ShowNum(1, 5, pid_debug_a_target, 4);
			OLED_ShowString(1, 10, "N:");
			OLED_ShowSignedNum(1, 12, pid_debug_a_now, 3);

			OLED_ShowString(2, 1, "A D:");
			OLED_ShowNum(2, 5, motorA_dir, 1);
			OLED_ShowString(2, 7, "O:");
			OLED_ShowNum(2, 9, pid_debug_a_out, 5);

			OLED_ShowString(3, 1, "B T:");
			OLED_ShowNum(3, 5, pid_debug_b_target, 4);
			OLED_ShowString(3, 10, "N:");
			OLED_ShowSignedNum(3, 12, pid_debug_b_now, 3);

			OLED_ShowString(4, 1, "B C:");
			OLED_ShowNum(4, 5, pid_debug_b_ccr, 4);
			OLED_ShowString(4, 10, "O:");
			OLED_ShowNum(4, 12, pid_debug_b_out, 5);
		}
		else
		{
			/* 屏幕1: 姿态数据 */
			OLED_ShowString(1, 1, "Yaw:");
			OLED_ShowFloat(1, 5, yaw, 4, 1);

			OLED_ShowString(2, 1, "Rol:");
			OLED_ShowFloat(2, 5, roll, 4, 1);

			OLED_ShowString(3, 1, "Pit:");
			OLED_ShowFloat(3, 5, pitch, 4, 1);

			/* 第4行: 磁力计原始值 */
			OLED_ShowString(4, 1, "X:");
			OLED_ShowSignedNum(4, 3, hmc_x, 5);
			OLED_ShowString(4, 9, "Y:");
			OLED_ShowSignedNum(4, 11, hmc_y, 5);
		}

		/* --- 串口输出 (约每550ms一次, 含姿态+速度+磁力计原始值) --- */
		if (++print_count >= 5)
		{
			print_count = 0;
			printf("Y:%+07.1f R:%+06.1f P:%+06.1f | "
			       "AT:%04d AN:%+04d BT:%04d BN:%+04d "
			       "AO:%05d BO:%05d\r\n",
			       yaw, roll, pitch,
			       pid_debug_a_target, pid_debug_a_now,
			       pid_debug_b_target, pid_debug_b_now,
			       pid_debug_a_out, pid_debug_b_out);
			printf("yaw_gyro=%.1f yaw_hmc=%.1f  "
			       "HMC raw: X=%+05d Y=%+05d Z=%+05d  "
			       "gz=%+05d\r\n",
			       yaw_gyro,
			       atan2((float)hmc_y, (float)hmc_x) * 57.29578f,
			       hmc_x, hmc_y, hmc_z,
			       gz);
		}
	}
#endif
}
