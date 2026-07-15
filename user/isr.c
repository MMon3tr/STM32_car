#include "stm32f10x.h"
#include "headfile.h"

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & 1)
	{
		TIM2->SR &= ~1;
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM3->SR & 1)
	{
		pid_control();
		TIM3->SR &= ~1;
	}
}

void TIM4_IRQHandler(void)
{
	if (TIM4->SR & 1)
	{
		TIM4->SR &= ~1;
	}
}

void USART1_IRQHandler(void)
{
	if (USART1->SR & 0x20)
	{
		USART1->SR &= ~0x20;
	}
}

void USART2_IRQHandler(void)
{
	if (USART2->SR & 0x20)
	{
		USART2->SR &= ~0x20;
	}
}

void USART3_IRQHandler(void)
{
	if (USART3->SR & 0x20)
	{
		USART3->SR &= ~0x20;
	}
}

void EXTI0_IRQHandler(void)
{
	if (EXTI->PR & (1 << 0))
	{
		EXTI->PR = 1 << 0;
	}
}

void EXTI1_IRQHandler(void)
{
	if (EXTI->PR & (1 << 1))
	{
		EXTI->PR = 1 << 1;
	}
}

void EXTI2_IRQHandler(void)
{
	if (EXTI->PR & (1 << 2))
	{
		if (gpio_get(GPIO_A, Pin_3))
			Encoder_count1--;
		else
			Encoder_count1++;

		EXTI->PR = 1 << 2;
	}
}

void EXTI3_IRQHandler(void)
{
	if (EXTI->PR & (1 << 3))
	{
		EXTI->PR = 1 << 3;
	}
}

void EXTI4_IRQHandler(void)
{
	if (EXTI->PR & (1 << 4))
	{
		if (gpio_get(GPIO_A, Pin_5))
			Encoder_count2++;
		else
			Encoder_count2--;

		EXTI->PR = 1 << 4;
	}
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & (1 << 5))
	{
		EXTI->PR = 1 << 5;
	}

	if (EXTI->PR & (1 << 6))
	{
		EXTI->PR = 1 << 6;
	}

	if (EXTI->PR & (1 << 7))
	{
		MPU6050_GetData();
		HMC5883L_GetData();

		roll_gyro += (float)gx / 16.4 * 0.005;
		pitch_gyro += (float)gy / 16.4 * 0.005;
		yaw_gyro += (float)gz / 16.4 * 0.005;

		roll_acc = atan((float)ay / az) * 57.296;
		pitch_acc = -atan((float)ax / az) * 57.296;
		yaw_acc = atan((float)ay / ax) * 57.296;
		yaw_hmc = atan2((float)hmc_x, (float)hmc_y) * 57.296;

		roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4);
		pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4);
		yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_hmc, (float)gz / 16.4);

		EXTI->PR = 1 << 7;
	}

	if (EXTI->PR & (1 << 8))
	{
		EXTI->PR = 1 << 8;
	}

	if (EXTI->PR & (1 << 9))
	{
		EXTI->PR = 1 << 9;
	}
}