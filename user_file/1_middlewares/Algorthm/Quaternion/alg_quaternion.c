#include "alg_quaternion.h"

/**
 * 模块说明：
 * 1) 本文件实现基础四元数运算 + Mahony 姿态更新。
 * 2) 输入约定：gyro 单位为 rad/s，acc 为任意比例值（会在内部归一化）。
 * 3) 输出约定：quat_to_euler 返回角度制（deg）。
 */

/**
 * @brief 四元数乘法
 * @note 结果表示“先做 b 再做 a”的复合旋转（主动旋转约定下）。
 */
static inline quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

/**
 * @brief 四元数共轭
 * @param q 输入四元数
 * @return 共轭四元数
 */
static inline quat_t quat_conj(quat_t q)
{
    return (quat_t){q.w, -q.x, -q.y, -q.z};
}

/**
 * @brief 四元数归一化
 * @param q 待归一化的四元数
 * @return 归一化后的四元数
 * @note 防止数值积分误差导致四元数模长漂移。
 */
static inline quat_t quat_normalize(quat_t q)
{
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 0)
    {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    return q;
}

/**
 * @brief 用四元数旋转向量 (v' = q * v * q^-1)
 * @param q 四元数（旋转量）
 * @param v 3D 向量（待旋转向量）
 * @return 旋转后的向量
 */
static inline vec3_t quat_rotate_vector(quat_t q, vec3_t v)
{
    // 把 3D 向量嵌入成“纯四元数”
    quat_t p = {0, v.x, v.y, v.z};

    quat_t q_conj = quat_conj(q);
    quat_t temp = quat_mul(q, p);
    quat_t result = quat_mul(temp, q_conj);

    return (vec3_t){result.x, result.y, result.z};
}

/**
 * @brief 根据当前姿态估计机体系下重力方向
 * @note Mahony 里用它和 acc 归一化方向做叉乘得到误差。
 */
static inline vec3_t quat_to_gravity(quat_t q)
{
    vec3_t gravity_world = {0, 0, 1};
    return quat_rotate_vector(q, gravity_world);
}

/**
 * @brief 四元数积分：dq/dt = 0.5 * q * omega
 * @param gyro 角速度（rad/s）
 * @param dt   采样周期（s）
 */
static inline quat_t quat_integrate(quat_t q, vec3_t gyro, float dt)
{
    quat_t omega = {0, gyro.x, gyro.y, gyro.z};
    quat_t dq = quat_mul(q, omega);

    // 一阶积分（Euler）
    q.w += 0.5f * dt * dq.w;
    q.x += 0.5f * dt * dq.x;
    q.y += 0.5f * dt * dq.y;
    q.z += 0.5f * dt * dq.z;

    return quat_normalize(q);
}

/**
 * @brief 四元数转欧拉角
 * @return 角度制（deg）
 */
euler_t quat_to_euler(quat_t q)
{
    euler_t euler;

    // Roll (X)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (Y)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
    {
        // 防止 asin 超界，处理接近万向锁情况
        euler.pitch = copysignf(M_PI / 2.0f, sinp);
    }
    else
    {
        euler.pitch = asinf(sinp);
    }

    // Yaw (Z)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp);

    // rad -> deg
    euler.roll *= 180.0f / M_PI;
    euler.pitch *= 180.0f / M_PI;
    euler.yaw *= 180.0f / M_PI;

    return euler;
}

/**
 * @brief Mahony 姿态一步更新
 * @param q  当前四元数（输入/输出）
 * @param imu 输入传感器数据（gyro: rad/s，acc 会内部归一化）
 * @param kp 比例增益（越大收敛越快）
 * @param ki 积分增益（用于长期漂移补偿）
 */
void mahony_update(quat_t *q, imu_data_t imu, float kp, float ki)
{
    // 积分项必须跨周期保留
    static vec3_t integral_error = {0, 0, 0};

    // 仅做记录，不改变外部控制流
    imu.Ki = ki;
    imu.Kp = kp;

    // 1) 归一化加速度方向
    float acc_norm = sqrtf(imu.acc.x * imu.acc.x +
                           imu.acc.y * imu.acc.y +
                           imu.acc.z * imu.acc.z);
    if (acc_norm > 0)
    {
        imu.acc.x /= acc_norm;
        imu.acc.y /= acc_norm;
        imu.acc.z /= acc_norm;
    }

    // 2) 估计重力方向
    vec3_t gravity = quat_to_gravity(*q);

    // 3) 误差 = 测得重力方向 x 估计重力方向
    vec3_t error;
    error.x = imu.acc.y * gravity.z - imu.acc.z * gravity.y;
    error.y = imu.acc.z * gravity.x - imu.acc.x * gravity.z;
    error.z = imu.acc.x * gravity.y - imu.acc.y * gravity.x;

    // 4) 积分补偿
    integral_error.x += error.x * ki * imu.dt;
    integral_error.y += error.y * ki * imu.dt;
    integral_error.z += error.z * ki * imu.dt;

    // 5) 比例 + 积分 修正陀螺仪
    vec3_t gyro_corrected;
    gyro_corrected.x = imu.gyro.x + kp * error.x + integral_error.x;
    gyro_corrected.y = imu.gyro.y + kp * error.y + integral_error.y;
    gyro_corrected.z = imu.gyro.z + kp * error.z + integral_error.z;

    // 6) 用修正角速度积分更新姿态
    *q = quat_integrate(*q, gyro_corrected, imu.dt);
} 
