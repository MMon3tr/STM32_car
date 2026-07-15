#include "headfile.h"

#define B_PWM_PIN_TEST 0
#define B_PWM_TEST_LEVEL 0

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
	encoder_init();
	uart_init(UART_1, 115200, 0);

	pid_init(&motorA, DELTA_PID, 500, 10, 0);
	pid_init(&motorB, DELTA_PID, 500, 10, 0);

	/* Dual-wheel trim: A is 2.05 and B is 2.0 pulses per 10 ms. */
	motor_target_set(2.05f, 2.0f);
	tim_interrupt_ms_init(TIM_3, 10, 0);

	while (1)
	{
		static uint8_t print_count = 0;

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

		delay_ms(100);

		if (++print_count >= 5)
		{
			print_count = 0;
			printf("A tar:%d now:%d out:%d\r\n", pid_debug_a_target, pid_debug_a_now, pid_debug_a_out);
			printf("B tar:%d now:%d out:%d\r\n", pid_debug_b_target, pid_debug_b_now, pid_debug_b_out);
		}
	}
#endif
}
