#ifndef __ALG_QUATERNION_H__
#define __ALG_QUATERNION_H__
#include <math.h>
#include <stdint.h>

// 四元数结构
typedef struct {
    float w, x, y, z;
} quat_t;

// 三维向量
typedef struct {
    float x, y, z;
} vec3_t;

// 欧拉角（弧度）
typedef struct {
    float roll;   // X轴
    float pitch;  // Y轴
    float yaw;    // Z轴
} euler_t;

// IMU数据结构
typedef struct {
    vec3_t gyro;     // 陀螺仪 (rad/s)
    vec3_t acc;      // 加速度计 (m/s²)
    float temp;      // 温度 (℃)
    float dt;        // 采样时间 (s)

    float Kp;        //比例项
    float Ki;        //积分项
} imu_data_t;

euler_t quat_to_euler(quat_t q);
void mahony_update(quat_t *q, imu_data_t imu, float kp, float ki);
#endif /* __ALG_QUATERNION_H__ */