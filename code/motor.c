#include "motor.h"

#define MOTOR_B_PWM_INVERTED 0

uint8_t motorA_dir = 1;
uint8_t motorB_dir = 1;

volatile int Encoder_count1 = 0;
volatile int Encoder_count2 = 0;

int speed_now;

void motor_init()
{
	pwm_init(TIM_2, TIM2_CH1, 1000);
	gpio_init(GPIO_A, Pin_6, OUT_PP);
	gpio_init(GPIO_A, Pin_7, OUT_PP);

	pwm_init(TIM_2, TIM2_CH2, 1000);
	gpio_init(GPIO_B, Pin_0, OUT_PP);
	gpio_init(GPIO_B, Pin_1, OUT_PP);
}

void motorA_duty(int duty)
{
	gpio_set(GPIO_A, Pin_6, motorA_dir);
	gpio_set(GPIO_A, Pin_7, !motorA_dir);
	pwm_update(TIM_2, TIM2_CH1, duty);
}

void motorB_duty(int duty)
{
	gpio_set(GPIO_B, Pin_0, !motorB_dir);
	gpio_set(GPIO_B, Pin_1, motorB_dir);

#if MOTOR_B_PWM_INVERTED
	pwm_update(TIM_2, TIM2_CH2, MAX_DUTY - duty);
#else
	pwm_update(TIM_2, TIM2_CH2, duty);
#endif
}

void motorB_stop(void)
{
#if MOTOR_B_PWM_INVERTED
	pwm_update(TIM_2, TIM2_CH2, MAX_DUTY);
#else
	pwm_update(TIM_2, TIM2_CH2, 0);
#endif
	gpio_set(GPIO_B, Pin_0, 0);
	gpio_set(GPIO_B, Pin_1, 0);
}

void encoder_init()
{
	exti_init(EXTI_PA2, FALLING, 0);
	gpio_init(GPIO_A, Pin_3, IU);

	exti_init(EXTI_PA4, FALLING, 0);
	gpio_init(GPIO_A, Pin_5, IU);
}
