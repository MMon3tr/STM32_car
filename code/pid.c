#include "headfile.h"

pid_t motorA;
pid_t motorB;
pid_t angle;

void datavision_send()  // ��λ�����η��ͺ���
{
	// ���ݰ�ͷ
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	// ��������
	uart_sendbyte(UART_1, (uint8_t)motorA.target);  
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
//	uart_sendbyte(UART_1, (uint8_t)motorB.target);  
//	uart_sendbyte(UART_1, (uint8_t)motorB.now);
	// ���ݰ�β
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

void motor_target_set(int spe1, int spe2)
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
	// ===== 速度环：读取编码器脉冲（本周期增量 = 速度）=====
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

	// ===== PID 计算 =====
	pid_cal(&motorA);
	pid_cal(&motorB);

	// ===== 输出限幅 =====
	pidout_limit(&motorA);
	pidout_limit(&motorB);

	// ===== 输出到电机 =====
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);

	// 每 500ms 打印一次调试信息（10ms周期 × 50次）
	static uint8_t print_cnt = 0;
	if(++print_cnt >= 50)
	{
		print_cnt = 0;
		printf("A tar:%.0f now:%.0f out:%.0f\r\n", motorA.target, motorA.now, motorA.out);
		printf("B tar:%.0f now:%.0f out:%.0f\r\n", motorB.target, motorB.now, motorB.out);
	}
}
void pid_cal(pid_t *pid)
{
	// ���㵱ǰƫ��
	pid->error[0] = pid->target - pid->now;

	// �������
	if(pid->pid_mode == DELTA_PID)  // ����ʽ
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // λ��ʽ
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// ��¼ǰ����ƫ��
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	// ����޷�
//	if(pid->out>=MAX_DUTY)	
//		pid->out=MAX_DUTY;
//	if(pid->out<=0)	
//		pid->out=0;
	
}

void pidout_limit(pid_t *pid)
{
	// ����޷�
	if(pid->out>=MAX_DUTY)	
		pid->out=MAX_DUTY;
	if(pid->out<=0)	
		pid->out=0;
}


