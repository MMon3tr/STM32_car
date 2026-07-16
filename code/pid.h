#ifndef __PID_h_
#define __PID_h_
#include "headfile.h"

enum
{
  POSITION_PID = 0,  // λ��ʽ
  DELTA_PID,         // ����ʽ
};

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

void pid_cal(pid_t *pid);
void pid_control(void);
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(float spe1, float spe2);
void pidout_limit(pid_t *pid);
float angle_error_normalize(float error);

extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
extern volatile int pid_debug_a_target;
extern volatile int pid_debug_a_now;
extern volatile int pid_debug_a_out;
extern volatile int pid_debug_b_target;
extern volatile int pid_debug_b_now;
extern volatile int pid_debug_b_out;
extern volatile int pid_debug_b_ccr;
#endif
