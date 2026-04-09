#include "alg_quaternion.h"

#define MAHONY_DT_DEFAULT_S         0.001f
#define MAHONY_DT_MIN_S             0.0002f
#define MAHONY_DT_MAX_S             0.0100f
#define MAHONY_ACC_NORM_VALID_MIN_G 0.05f
#define DEG_TO_RAD                  (M_PI / 180.0f)

volatile mahony_debug_t g_mahony_debug = {1.0f, 1.0f, MAHONY_DT_DEFAULT_S, 0u};

// 积分项状态（跨周期保留）
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

static float mahony_inv_sqrt(float x)
{
    if ((isfinite(x) == 0) || (x <= 0.0f))
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static float mahony_sanitize_dt(float dt_s)
{
    if ((isfinite(dt_s) == 0) || (dt_s <= 0.0f))
    {
        return MAHONY_DT_DEFAULT_S;
    }
    if (dt_s < MAHONY_DT_MIN_S)
    {
        return MAHONY_DT_MIN_S;
    }
    if (dt_s > MAHONY_DT_MAX_S)
    {
        return MAHONY_DT_MAX_S;
    }
    return dt_s;
}

static void mahony_reset_quat(quat_t *q)
{
    if (q == NULL)
    {
        return;
    }
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

static void mahony_reset_integral(void)
{
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

/**
 * @brief 四元数转欧拉角（标准 ZYX：roll-x / pitch-y / yaw-z）
 * @return 角度制（deg）
 */
euler_t quat_to_euler(quat_t q)
{
    euler_t euler;
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    // Roll (X)
    sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (Y)
    sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
    {
        euler.pitch = copysignf(M_PI / 2.0f, sinp);
    }
    else
    {
        euler.pitch = asinf(sinp);
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
 * @brief Mahony 姿态一步更新（IMU版：gyro + accel）
 * @param q  当前四元数（输入/输出）
 * @param imu 输入传感器数据（gyro: deg/s，acc: g）
 * @param kp 比例增益
 * @param ki 积分增益
 */
void mahony_update(quat_t *q, imu_data_t imu, float kp, float ki)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;
    float gx, gy, gz;
    float dt_s;
    float acc_norm_raw;
    uint8_t gyro_only_mode = 1u;

    if (q == NULL)
    {
        return;
    }

    if ((isfinite(q->w) == 0) || (isfinite(q->x) == 0) ||
        (isfinite(q->y) == 0) || (isfinite(q->z) == 0))
    {
        mahony_reset_quat(q);
        mahony_reset_integral();
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

    dt_s = mahony_sanitize_dt(imu.dt);

    // 输入语义改为 deg/s，内部统一换算成 rad/s
    gx = imu.gyro.x * DEG_TO_RAD;
    gy = imu.gyro.y * DEG_TO_RAD;
    gz = imu.gyro.z * DEG_TO_RAD;

    // 加速度有效性判断 + 归一化（MahonyAHRS IMU 核心）
    acc_norm_raw = sqrtf(imu.acc.x * imu.acc.x + imu.acc.y * imu.acc.y + imu.acc.z * imu.acc.z);
    if ((isfinite(acc_norm_raw) != 0) && (acc_norm_raw > MAHONY_ACC_NORM_VALID_MIN_G))
    {
        recipNorm = mahony_inv_sqrt(imu.acc.x * imu.acc.x + imu.acc.y * imu.acc.y + imu.acc.z * imu.acc.z);
        if (recipNorm > 0.0f)
        {
            imu.acc.x *= recipNorm;
            imu.acc.y *= recipNorm;
            imu.acc.z *= recipNorm;

            // 由四元数估计重力方向
            halfvx = q->x * q->z - q->w * q->y;
            halfvy = q->w * q->x + q->y * q->z;
            halfvz = q->w * q->w - 0.5f + q->z * q->z;

            // 误差 = measured_gravity x estimated_gravity
            halfex = (imu.acc.y * halfvz - imu.acc.z * halfvy);
            halfey = (imu.acc.z * halfvx - imu.acc.x * halfvz);
            halfez = (imu.acc.x * halfvy - imu.acc.y * halfvx);

            // 积分反馈（可用于抑制陀螺零偏）
            if (ki > 0.0f)
            {
                integralFBx += 2.0f * ki * halfex * dt_s;
                integralFBy += 2.0f * ki * halfey * dt_s;
                integralFBz += 2.0f * ki * halfez * dt_s;
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }
            else
            {
                mahony_reset_integral();
            }

            // 比例反馈
            gx += 2.0f * kp * halfex;
            gy += 2.0f * kp * halfey;
            gz += 2.0f * kp * halfez;

            gyro_only_mode = 0u;
        }
    }

    // 四元数积分更新
    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;
    qa = q->w;
    qb = q->x;
    qc = q->y;
    q->w += (-qb * gx - qc * gy - q->z * gz);
    q->x += (qa * gx + qc * gz - q->z * gy);
    q->y += (qa * gy - qb * gz + q->z * gx);
    q->z += (qa * gz + qb * gy - qc * gx);

    recipNorm = mahony_inv_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (recipNorm > 0.0f)
    {
        q->w *= recipNorm;
        q->x *= recipNorm;
        q->y *= recipNorm;
        q->z *= recipNorm;
    }
    else
    {
        mahony_reset_quat(q);
    }

    // 轻量调试观测量
    g_mahony_debug.acc_norm_raw = (isfinite(acc_norm_raw) != 0) ? acc_norm_raw : 0.0f;
    g_mahony_debug.acc_weight = (gyro_only_mode == 0u) ? 1.0f : 0.0f;
    g_mahony_debug.dt_used = dt_s;
    g_mahony_debug.gyro_only_mode = gyro_only_mode;
}
