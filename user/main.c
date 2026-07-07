#include "headfile.h"

int main(void)
{
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1,1,"PID CLOSED-LOOP");

	motor_init();
	encoder_init();
	uart_init(UART_1,115200,0);

	// 保守 PID 参数：P=50, I=1, D=0
	pid_init(&motorA, DELTA_PID, 50, 1, 0);
	pid_init(&motorB, DELTA_PID, 50, 1, 0);

	// 先只调电机 A，目标 5 脉冲/10ms（最大速度约 10）
	motor_target_set(5, 0);

	// 启动 10ms PID 定时中断
	tim_interrupt_ms_init(TIM_3, 10, 0);

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


