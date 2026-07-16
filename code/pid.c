#include "headfile.h"

#define PID_TEST_MAX_DUTY 30000

pid_t motorA;
pid_t motorB;
pid_t angle;

volatile int pid_debug_a_target;
volatile int pid_debug_a_now;
volatile int pid_debug_a_out;
volatile int pid_debug_b_target;
volatile int pid_debug_b_now;
volatile int pid_debug_b_out;
volatile int pid_debug_b_ccr;

void datavision_send()
{
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, (uint8_t)motorA.target);
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, 0x03);
}

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void motor_target_set(float spe1, float spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}

	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

void pid_control()
{
	if(motorA_dir)
		motorA.now = Encoder_count1;
	else
		motorA.now = -Encoder_count1;

	if(motorB_dir)
		motorB.now = Encoder_count2;
	else
		motorB.now = -Encoder_count2;

	Encoder_count1 = 0;
	Encoder_count2 = 0;

	pid_cal(&motorA);
	pidout_limit(&motorA);
	motorA_duty(motorA.out);

	if(motorB.target == 0)
	{
		motorB.out = 0;
		motorB.pout = 0;
		motorB.iout = 0;
		motorB.dout = 0;
		motorB.error[0] = 0;
		motorB.error[1] = 0;
		motorB.error[2] = 0;
		motorB_stop();
	}
	else
	{
		pid_cal(&motorB);
		pidout_limit(&motorB);
		motorB_duty(motorB.out);
	}

	pid_debug_a_target = (int)motorA.target;
	pid_debug_a_now = (int)motorA.now;
	pid_debug_a_out = (int)motorA.out;
	pid_debug_b_target = (int)motorB.target;
	pid_debug_b_now = (int)motorB.now;
	pid_debug_b_out = (int)motorB.out;
	pid_debug_b_ccr = (int)TIM2->CCR2;
}

void pid_cal(pid_t *pid)
{
	pid->error[0] = pid->target - pid->now;

	if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
	if(pid->out >= PID_TEST_MAX_DUTY)
		pid->out = PID_TEST_MAX_DUTY;
	if(pid->out <= 0)
		pid->out = 0;
}

/* 将角度误差规范到 [-180, 180] 度 */
float angle_error_normalize(float error)
{
	while (error > 180.0f)  error -= 360.0f;
	while (error < -180.0f) error += 360.0f;
	return error;
}
