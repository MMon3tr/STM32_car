#ifndef _ATTITUDE_H
#define _ATTITUDE_H

#include "ml_mpu6050.h"
#include "ml_hmc5883l.h"
#include "filter.h"
#include "math.h"
#include "ml_delay.h"

// ================= 姿态角 =================
extern float roll;
extern float pitch;
extern float yaw;

// ================= 中间量 =================
//extern float roll_acc;
//extern float pitch_acc;

//extern float yaw_gyro;
//extern float yaw_hmc;

// ================= MPU零偏 =================
extern float gx_bias;
extern float gy_bias;
extern float gz_bias;

// ================= 时间步长 =================
extern float dt;

// ================= 函数 =================
void MPU6050_Calibrate(void);
void Attitude_Update(void);
void HMC5883L_UpdateYaw(void);

#endif
