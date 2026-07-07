#include "headfile.h"
// 全速50000
int main(void)
{
	OLED_Init();
	
	OLED_ShowString(2,1,"Ciallo");
	
	motor_init();
	encoder_init();	

	uart_init(UART_1,115200,0);
	
	pid_init(&motorA, DELTA_PID, 10, 10, 5);
	pid_init(&motorB, DELTA_PID, 10, 10, 5);
	pid_init(&angle, POSITION_PID, 2, 0, 0.5);
	
	gray_init();
	
	I2C_Init();
	MPU6050_Init();
	HMC5883L_Init();
	exti_init(EXTI_PB7,RISING,0);
	
//	tim_interrupt_ms_init(TIM_3,10,0);
	
	MPU6050_Calibrate();
	while (1)
	{
//		printf("ax:%d, ay:%d az:%d gx:%d gy:%d gz:%d\r\n", ax, ay, az, gx, gy, gz);
//		printf("yaw:%.2f  pitch:%.2f roll:%.2f\r\n", yaw_Kalman, pitch_Kalman, roll_Kalman);
//		MPU6050_GetData();
//		OLED_ShowFloat(1, 1, yaw_Kalman, 3, 2);
		motorA_duty(10000);
//		Attitude_Update();
//		HMC5883L_UpdateYaw();
//		
//		OLED_ShowFloat(1,1,roll,3,2);
//		OLED_ShowFloat(2,1,pitch,3,2);
//		OLED_ShowFloat(3,1,yaw,3,2);
//		
//		delay_ms(20);
	} 
}


