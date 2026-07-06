#include "attitude.h"
#include "math.h"

// ================= 姿态角 =================
float roll = 0;
float pitch = 0;
float yaw = 0;

// ================= 中间量 =================
//float roll_acc = 0;
//float pitch_acc = 0;

//float yaw_gyro = 0;
//float yaw_hmc = 0;

// ================= MPU零偏 =================
float gx_bias = 0;
float gy_bias = 0;
float gz_bias = 0;

// ================= 时间步长 =================
float dt = 0.01f;

// ================= MPU6050校准 =================
void MPU6050_Calibrate(void)
{
    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;

    for(int i = 0; i < 500; i++)
    {
        MPU6050_GetData();

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay_ms(2);
    }

    gx_bias = gx_sum / 500.0f;
    gy_bias = gy_sum / 500.0f;
    gz_bias = gz_sum / 500.0f;
}

// ================= 姿态更新（roll/pitch） =================
void Attitude_Update(void)
{
    MPU6050_GetData();
	if(fabsf((float)gz) < 0.02f)
		gz = 0;

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float gx_dps = (gx - gx_bias) / 16.4f;
    float gy_dps = (gy - gy_bias) / 16.4f;

    // 加速度角
    roll_acc  = atan2(ay_g, az_g) * 57.29578f;
    pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.29578f;

    // 互补滤波
    roll  = 0.98f * (roll  + gx_dps * dt) + 0.02f * roll_acc;
    pitch = 0.98f * (pitch + gy_dps * dt) + 0.02f * pitch_acc;
}

// ================= yaw计算（磁力计） =================
void HMC5883L_UpdateYaw(void)
{
    HMC5883L_GetData();

    float roll_rad  = roll * 0.0174533f;
    float pitch_rad = pitch * 0.0174533f;

    float mx = (float)hmc_x;
    float my = (float)hmc_y;
    float mz = (float)hmc_z;

    // 倾斜补偿
    float mx2 = mx * cos(pitch_rad) + mz * sin(pitch_rad);
    float my2 = mx * sin(roll_rad) * sin(pitch_rad)
              + my * cos(roll_rad)
              - mz * sin(roll_rad) * cos(pitch_rad);

    yaw_hmc = atan2(-my2, mx2) * 57.29578f;

    // 陀螺积分
    float gz_dps = (gz - gz_bias) / 16.4f;
    yaw_gyro += gz_dps * dt;

    // 融合
    yaw = 0.95f * yaw_gyro + 0.05f * yaw_hmc;
}