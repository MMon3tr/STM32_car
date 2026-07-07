#include "headfile.h"

int main(void)
{
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1,1,"PID CLOSED-LOOP");

	motor_init();
	encoder_init();
	uart_init(UART_1,115200,0);

	// PID 参数：P=1500, I=30, D=0（增量式，30ms周期）
	pid_init(&motorA, DELTA_PID, 1500, 30, 0);
	pid_init(&motorB, DELTA_PID, 1500, 30, 0);

	// 双电机闭环，目标 15 脉冲/30ms
	motor_target_set(15, 15);

	// 启动 30ms PID 定时中断（增大周期提高速度分辨率）
	tim_interrupt_ms_init(TIM_3, 30, 0);

	while (1)
	{
		// OLED 实时显示 A 电机的 target / now / out
		OLED_ShowString(2,1,"T:");
		OLED_ShowNum(2,3,(int)motorA.target,4);
		OLED_ShowString(3,1,"N:");
		OLED_ShowNum(3,3,(int)motorA.now,4);
		OLED_ShowString(4,1,"O:");
		OLED_ShowNum(4,3,(int)motorA.out,5);
		delay_ms(100);
	}
}


