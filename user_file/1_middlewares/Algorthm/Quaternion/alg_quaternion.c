#include "alg_quaternion.h"

#define MAHONY_DT_DEFAULT_S            0.001f
#define MAHONY_DT_MIN_S                0.0002f
#define MAHONY_DT_MAX_S                0.0100f

// 动态加速度门控阈值（单位：g）
#define MAHONY_ACC_NORM_TARGET_G       1.0f
#define MAHONY_ACC_NORM_VALID_MIN_G    0.05f
#define MAHONY_ACC_DEV_FULL_TRUST_G    0.08f
#define MAHONY_ACC_DEV_GYRO_ONLY_G     0.35f
#define MAHONY_ACC_WEIGHT_EPS          1e-3f

volatile mahony_debug_t g_mahony_debug = {1.0f, 1.0f, MAHONY_DT_DEFAULT_S, 0u};

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
    if ((isfinite(norm) != 0) && (norm > 0.0f))
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
 * @brief 计算加速度纠偏权重（连续降权）
 * @param acc_norm_raw 原始加速度模长（g）
 * @return 权重 [0, 1]
 */
static float mahony_compute_acc_weight(float acc_norm_raw)
{
    float dev;
    float denom;

    if ((isfinite(acc_norm_raw) == 0) || (acc_norm_raw < MAHONY_ACC_NORM_VALID_MIN_G))
    {
        return 0.0f;
    }

    dev = fabsf(acc_norm_raw - MAHONY_ACC_NORM_TARGET_G);

    if (dev <= MAHONY_ACC_DEV_FULL_TRUST_G)
    {
        return 1.0f;
    }

    if (dev >= MAHONY_ACC_DEV_GYRO_ONLY_G)
    {
        return 0.0f;
    }

    denom = MAHONY_ACC_DEV_GYRO_ONLY_G - MAHONY_ACC_DEV_FULL_TRUST_G;
    if (denom <= 0.0f)
    {
        return 0.0f;
    }

    return (MAHONY_ACC_DEV_GYRO_ONLY_G - dev) / denom;
}

/**
 * @brief 四元数转欧拉角
 * @return 角度制（deg）
 * @note 这里本来是按照bmi088的手册解算的，但是ACE的解算结果和bmi088的解算结果有差异，所以这里用ACE的解算结果，也就是pitch和roll的顺序对调。
 *       我的解算的顺序是按bmi088的ZYX，但是ACE的Pitch和Roll都不会万向锁。
 */
euler_t quat_to_euler(quat_t q)
{
    euler_t euler;
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    // Roll (X),但是ACE的pitch
    sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.pitch = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (Y)，但是是ACE的roll
    sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
    {
        // 防止 asin 超界，处理接近万向锁情况
        euler.roll = copysignf(M_PI / 2.0f, sinp);
    }
    else
    {
        euler.roll = asinf(sinp);
    }

    // Yaw (Z)
    siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
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
    static vec3_t integral_error = {0.0f, 0.0f, 0.0f};
    float acc_norm_raw;
    float acc_weight;
    vec3_t gravity;
    vec3_t error = {0.0f, 0.0f, 0.0f};
    vec3_t gyro_corrected;
    uint8_t gyro_only_mode;

    if (q == NULL)
    {
        return;
    }

    if ((isfinite(q->w) == 0) || (isfinite(q->x) == 0) ||
        (isfinite(q->y) == 0) || (isfinite(q->z) == 0))
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        integral_error.x = 0.0f;
        integral_error.y = 0.0f;
        integral_error.z = 0.0f;
    }

    if ((isfinite(imu.dt) == 0) || (imu.dt <= 0.0f))
    {
        imu.dt = MAHONY_DT_DEFAULT_S;
    }
    else if (imu.dt < MAHONY_DT_MIN_S)
    {
        imu.dt = MAHONY_DT_MIN_S;
    }
    else if (imu.dt > MAHONY_DT_MAX_S)
    {
        imu.dt = MAHONY_DT_MAX_S;
    }

    if ((isfinite(kp) == 0) || (kp < 0.0f))
    {
        kp = 0.0f;
    }

    if ((isfinite(ki) == 0) || (ki < 0.0f))
    {
        ki = 0.0f;
    }

    if ((isfinite(imu.gyro.x) == 0) || (isfinite(imu.gyro.y) == 0) || (isfinite(imu.gyro.z) == 0))
    {
        imu.gyro.x = 0.0f;
        imu.gyro.y = 0.0f;
        imu.gyro.z = 0.0f;
    }

    // 1) 基于加速度模长做动态门控（单位近似为 g）
    acc_norm_raw = sqrtf(imu.acc.x * imu.acc.x +
                         imu.acc.y * imu.acc.y +
                         imu.acc.z * imu.acc.z);
    acc_weight = mahony_compute_acc_weight(acc_norm_raw);
    gyro_only_mode = (acc_weight <= MAHONY_ACC_WEIGHT_EPS) ? 1u : 0u;

    if (gyro_only_mode == 0u)
    {
        // 2) 归一化加速度方向（仅在可信时参与纠偏）
        imu.acc.x /= acc_norm_raw;
        imu.acc.y /= acc_norm_raw;
        imu.acc.z /= acc_norm_raw;

        // 3) 估计重力方向
        gravity = quat_to_gravity(*q);

        // 4) 误差 = 测得重力方向 x 估计重力方向
        error.x = imu.acc.y * gravity.z - imu.acc.z * gravity.y;
        error.y = imu.acc.z * gravity.x - imu.acc.x * gravity.z;
        error.z = imu.acc.x * gravity.y - imu.acc.y * gravity.x;
    }

    // 5) 积分补偿（按权重降权，动态加速度大时弱化/关闭）
    integral_error.x += error.x * ki * imu.dt * acc_weight;
    integral_error.y += error.y * ki * imu.dt * acc_weight;
    integral_error.z += error.z * ki * imu.dt * acc_weight;

    if ((isfinite(integral_error.x) == 0) || (isfinite(integral_error.y) == 0) || (isfinite(integral_error.z) == 0))
    {
        integral_error.x = 0.0f;
        integral_error.y = 0.0f;
        integral_error.z = 0.0f;
    }

    // 6) 比例 + 积分 修正陀螺仪（比例项同样按权重降权）
    gyro_corrected.x = imu.gyro.x + kp * acc_weight * error.x + integral_error.x;
    gyro_corrected.y = imu.gyro.y + kp * acc_weight * error.y + integral_error.y;
    gyro_corrected.z = imu.gyro.z + kp * acc_weight * error.z + integral_error.z;

    if ((isfinite(gyro_corrected.x) == 0) || (isfinite(gyro_corrected.y) == 0) || (isfinite(gyro_corrected.z) == 0))
    {
        gyro_corrected = imu.gyro;
    }

    // 7) 用修正角速度积分更新姿态
    *q = quat_integrate(*q, gyro_corrected, imu.dt);

    if ((isfinite(q->w) == 0) || (isfinite(q->x) == 0) ||
        (isfinite(q->y) == 0) || (isfinite(q->z) == 0))
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }

    // 轻量调试观测量
    g_mahony_debug.acc_norm_raw = (isfinite(acc_norm_raw) != 0) ? acc_norm_raw : 0.0f;
    g_mahony_debug.acc_weight = acc_weight;
    g_mahony_debug.dt_used = imu.dt;
    g_mahony_debug.gyro_only_mode = gyro_only_mode;
}
