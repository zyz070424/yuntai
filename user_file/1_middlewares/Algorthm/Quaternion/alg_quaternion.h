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

// 欧拉角（为兼容现有视觉链路，这里实际使用角度制）
typedef struct {
    float roll;   // X轴
    float pitch;  // Y轴
    float yaw;    // Z轴
} euler_t;

// IMU数据结构
typedef struct {
    vec3_t gyro;     // 陀螺仪 (deg/s)
    vec3_t acc;      // 加速度计 (g)
    float temp;      // 温度 (℃)
    float dt;        // 采样时间 (s)

    float Kp;        //比例项
    float Ki;        //积分项
} imu_data_t;

// Mahony 调试状态（轻量观测量）
typedef struct {
    float acc_norm_raw;      // 本周期原始加速度模长（g）
    float acc_weight;        // 本周期加速度纠偏权重 [0, 1]
    float dt_used;           // 本周期实际用于积分的 dt（s）
    uint8_t gyro_only_mode;  // 1=仅陀螺积分 0=融合加速度
} mahony_debug_t;

euler_t quat_to_euler(quat_t q);
void mahony_update(quat_t *q, imu_data_t imu, float kp, float ki);
extern volatile mahony_debug_t g_mahony_debug;
#endif /* __ALG_QUATERNION_H__ */
