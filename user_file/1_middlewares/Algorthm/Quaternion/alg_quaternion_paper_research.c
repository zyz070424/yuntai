#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define QUAT_PAPER_RESEARCH_DT_DEFAULT_S         0.001f
#define QUAT_PAPER_RESEARCH_DT_MIN_S             0.0002f
#define QUAT_PAPER_RESEARCH_DT_MAX_S             0.0100f
#define QUAT_PAPER_RESEARCH_ACC_NORM_VALID_MIN_G 0.05f
#define QUAT_PAPER_RESEARCH_DEG_TO_RAD           (M_PI / 180.0f)
#define QUAT_PAPER_RESEARCH_KF_Q                 0.02f
#define QUAT_PAPER_RESEARCH_KF_R_BASE            2.0f
#define QUAT_PAPER_RESEARCH_KF_R_MIN             0.2f
#define QUAT_PAPER_RESEARCH_KF_R_MAX             40.0f
#define QUAT_PAPER_RESEARCH_STATIC_GYRO_THR_DPS  1.0f
#define QUAT_PAPER_RESEARCH_STATIC_ACC_DEV_THR_G 0.05f
#define QUAT_PAPER_RESEARCH_STATIC_COUNT_MIN     80u
#define QUAT_PAPER_RESEARCH_VIB_ACC_DEV_THR_G    0.12f
#define QUAT_PAPER_RESEARCH_VIB_INNOV_THR_DPS    4.0f
#define QUAT_PAPER_RESEARCH_VIB_COUNT_MIN        20u
#define QUAT_PAPER_RESEARCH_BIAS_DEADBAND_DPS    0.03f
#define QUAT_PAPER_RESEARCH_LS_MIN_SAMPLES       6u
#define QUAT_PAPER_RESEARCH_ACC_WEIGHT_VIB_SCALE 0.15f
#define QUAT_PAPER_RESEARCH_INNOV_LP_ALPHA       0.10f
#define QUAT_PAPER_RESEARCH_BIAS_LP_ALPHA        0.05f
#define QUAT_PAPER_RESEARCH_LS_DEN_EPS           1.0e-6f

/**
 * @file alg_quaternion_paper_research.c
 * @brief 独立保存论文版姿态实验代码
 * @note 本文件默认不加入 CMakeLists.txt，仅保留给后续科研/对比使用
 * @note 推荐直接使用 quat_paper_research_observe_reset() 和 quat_paper_research_observe_update()
 */

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_paper_research_quat_t;

typedef struct {
    float x;
    float y;
    float z;
} quat_paper_research_vec3_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} quat_paper_research_euler_t;

typedef struct {
    quat_paper_research_vec3_t gyro;
    quat_paper_research_vec3_t acc;
    float temp;
    float dt;
} quat_paper_research_imu_t;

typedef struct {
    float estimate;
    float p;
    float q;
    float r;
    float innovation_lp;
} quat_paper_research_kf_axis_t;

typedef struct {
    float sum_t;
    float sum_y;
    float sum_tt;
    float sum_ty;
    uint16_t sample_count;
    float slope;
    float intercept;
} quat_paper_research_ls_axis_t;

typedef struct {
    quat_paper_research_quat_t quat;
    quat_paper_research_kf_axis_t kf[3];
    quat_paper_research_ls_axis_t ls[3];
    quat_paper_research_vec3_t gyro_bias_dps;
    quat_paper_research_vec3_t gyro_filt_dps;
    quat_paper_research_vec3_t gyro_comp_dps;
    float runtime_s;
    uint16_t static_counter;
    uint16_t vibration_counter;
    uint8_t is_static;
    uint8_t is_vibrating;
    float integralFBx;
    float integralFBy;
    float integralFBz;
} quat_paper_research_state_t;

typedef struct {
    float acc_norm_raw;
    float gyro_norm_raw_dps;
    float gyro_norm_comp_dps;
    quat_paper_research_vec3_t bias_pred_dps;
    quat_paper_research_vec3_t innovation_dps;
    uint8_t static_state;
    uint8_t vibration_state;
    float acc_weight_scale;
    float dt_used;
} quat_paper_research_debug_t;

typedef struct {
    float zero_raw_deg;
    float last_rel_wrapped_deg;
    float continuous_deg;
    uint8_t inited;
} quat_paper_research_yaw_state_t;

typedef struct {
    quat_paper_research_state_t paper_state;
    quat_paper_research_debug_t paper_debug;
    quat_paper_research_euler_t paper_euler_raw;
    quat_paper_research_euler_t paper_euler;
    quat_paper_research_imu_t imu_input;
    quat_paper_research_imu_t imu_input_body;
    quat_paper_research_yaw_state_t yaw_state;
    float dt_s;
    uint32_t update_count;
} quat_paper_research_observe_t;

static float quat_paper_research_inv_sqrt(float x)
{
    if ((isfinite(x) == 0) || (x <= 0.0f))
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}

static float quat_paper_research_sanitize_dt(float dt_s)
{
    if ((isfinite(dt_s) == 0) || (dt_s <= 0.0f))
    {
        return QUAT_PAPER_RESEARCH_DT_DEFAULT_S;
    }
    if (dt_s < QUAT_PAPER_RESEARCH_DT_MIN_S)
    {
        return QUAT_PAPER_RESEARCH_DT_MIN_S;
    }
    if (dt_s > QUAT_PAPER_RESEARCH_DT_MAX_S)
    {
        return QUAT_PAPER_RESEARCH_DT_MAX_S;
    }

    return dt_s;
}

/**
 * @brief 复位研究版四元数
 * @param q: 四元数指针
 * @retval 无
 */
static void quat_paper_research_reset_quat(quat_paper_research_quat_t *q)
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

/**
 * @brief 研究版内部限幅工具
 * @param value: 输入值
 * @param min_value: 下限
 * @param max_value: 上限
 * @retval 限幅后的结果
 */
static float quat_paper_research_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 按轴读取三维向量分量
 * @param v: 向量指针
 * @param axis: 轴号，0=x，1=y，2=z
 * @retval 对应轴分量
 */
static float quat_paper_research_vec3_get(const quat_paper_research_vec3_t *v, uint8_t axis)
{
    if (v == NULL)
    {
        return 0.0f;
    }

    switch (axis)
    {
        case 0u:
            return v->x;
        case 1u:
            return v->y;
        default:
            return v->z;
    }
}

/**
 * @brief 按轴写入三维向量分量
 * @param v: 向量指针
 * @param axis: 轴号，0=x，1=y，2=z
 * @param value: 待写入的分量值
 * @retval 无
 */
static void quat_paper_research_vec3_set(quat_paper_research_vec3_t *v, uint8_t axis, float value)
{
    if (v == NULL)
    {
        return;
    }

    switch (axis)
    {
        case 0u:
            v->x = value;
            break;
        case 1u:
            v->y = value;
            break;
        default:
            v->z = value;
            break;
    }
}

/**
 * @brief 计算三维向量模长
 * @param v: 输入向量
 * @retval 向量模长，异常输入时返回 0
 */
static float quat_paper_research_vec3_norm(quat_paper_research_vec3_t v)
{
    float sum;

    sum = v.x * v.x + v.y * v.y + v.z * v.z;
    if ((isfinite(sum) == 0) || (sum <= 0.0f))
    {
        return 0.0f;
    }

    return sqrtf(sum);
}

/**
 * @brief 复位单轴卡尔曼状态
 * @param axis: 单轴卡尔曼状态指针
 * @retval 无
 */
static void quat_paper_research_reset_kf_axis(quat_paper_research_kf_axis_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    axis->estimate = 0.0f;
    axis->p = 1.0f;
    axis->q = QUAT_PAPER_RESEARCH_KF_Q;
    axis->r = QUAT_PAPER_RESEARCH_KF_R_BASE;
    axis->innovation_lp = 0.0f;
}

/**
 * @brief 复位单轴最小二乘拟合状态
 * @param axis: 单轴最小二乘状态指针
 * @retval 无
 */
static void quat_paper_research_reset_ls_axis(quat_paper_research_ls_axis_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    axis->sum_t = 0.0f;
    axis->sum_y = 0.0f;
    axis->sum_tt = 0.0f;
    axis->sum_ty = 0.0f;
    axis->sample_count = 0u;
    axis->slope = 0.0f;
    axis->intercept = 0.0f;
}

/**
 * @brief 单轴新息自适应卡尔曼更新
 * @param axis: 单轴卡尔曼状态指针
 * @param measurement: 当前轴原始角速度（deg/s）
 * @param innovation_abs_out: 创新绝对值输出指针，可为 NULL
 * @retval 当前轴滤波后的角速度（deg/s）
 */
static float quat_paper_research_kf_axis_update(quat_paper_research_kf_axis_t *axis,
                                                float measurement,
                                                float *innovation_abs_out)
{
    float innovation;
    float innovation_abs;
    float k;
    float denom;

    if (axis == NULL)
    {
        if (innovation_abs_out != NULL)
        {
            *innovation_abs_out = 0.0f;
        }
        return measurement;
    }

    if (isfinite(measurement) == 0)
    {
        measurement = 0.0f;
    }

    if ((isfinite(axis->estimate) == 0) || (isfinite(axis->p) == 0) || (axis->p < 0.0f) ||
        (isfinite(axis->q) == 0) || (axis->q <= 0.0f) ||
        (isfinite(axis->r) == 0) || (axis->r <= 0.0f))
    {
        quat_paper_research_reset_kf_axis(axis);
        axis->estimate = measurement;
    }

    axis->p += axis->q;
    innovation = measurement - axis->estimate;
    innovation_abs = fabsf(innovation);

    if (isfinite(axis->innovation_lp) == 0)
    {
        axis->innovation_lp = innovation_abs;
    }
    else
    {
        axis->innovation_lp += QUAT_PAPER_RESEARCH_INNOV_LP_ALPHA * (innovation_abs - axis->innovation_lp);
    }

    axis->r = quat_paper_research_clamp(QUAT_PAPER_RESEARCH_KF_R_BASE +
                                            axis->innovation_lp * axis->innovation_lp,
                                        QUAT_PAPER_RESEARCH_KF_R_MIN,
                                        QUAT_PAPER_RESEARCH_KF_R_MAX);
    denom = axis->p + axis->r;
    if ((isfinite(denom) == 0) || (denom <= 0.0f))
    {
        quat_paper_research_reset_kf_axis(axis);
        if (innovation_abs_out != NULL)
        {
            *innovation_abs_out = 0.0f;
        }
        return measurement;
    }

    k = axis->p / denom;
    axis->estimate += k * innovation;
    axis->p = (1.0f - k) * axis->p;

    if (innovation_abs_out != NULL)
    {
        *innovation_abs_out = innovation_abs;
    }

    return axis->estimate;
}

/**
 * @brief 向单轴零偏线性模型追加静止样本
 * @param axis: 单轴最小二乘状态指针
 * @param t_s: 当前运行时间（s）
 * @param y: 当前静止样本（deg/s）
 * @retval 无
 */
static void quat_paper_research_ls_axis_add_sample(quat_paper_research_ls_axis_t *axis, float t_s, float y)
{
    float n;
    float den;

    if (axis == NULL)
    {
        return;
    }

    if ((isfinite(t_s) == 0) || (isfinite(y) == 0))
    {
        return;
    }

    if (axis->sample_count == UINT16_MAX)
    {
        axis->sum_t *= 0.5f;
        axis->sum_y *= 0.5f;
        axis->sum_tt *= 0.5f;
        axis->sum_ty *= 0.5f;
        axis->sample_count = (uint16_t)(UINT16_MAX / 2u);
    }

    axis->sum_t += t_s;
    axis->sum_y += y;
    axis->sum_tt += t_s * t_s;
    axis->sum_ty += t_s * y;
    axis->sample_count++;

    n = (float)axis->sample_count;
    if (n <= 0.0f)
    {
        return;
    }

    den = n * axis->sum_tt - axis->sum_t * axis->sum_t;
    if (fabsf(den) > QUAT_PAPER_RESEARCH_LS_DEN_EPS)
    {
        axis->slope = (n * axis->sum_ty - axis->sum_t * axis->sum_y) / den;
        axis->intercept = (axis->sum_y - axis->slope * axis->sum_t) / n;
    }
    else
    {
        axis->slope = 0.0f;
        axis->intercept = axis->sum_y / n;
    }
}

/**
 * @brief 预测当前时刻的单轴零偏
 * @param axis: 单轴最小二乘状态指针
 * @param t_s: 当前运行时间（s）
 * @param fallback: 样本不足时的回退零偏（deg/s）
 * @retval 预测零偏（deg/s）
 */
static float quat_paper_research_ls_axis_predict(const quat_paper_research_ls_axis_t *axis,
                                                 float t_s,
                                                 float fallback)
{
    float pred;

    if (axis == NULL)
    {
        return fallback;
    }

    if (axis->sample_count < QUAT_PAPER_RESEARCH_LS_MIN_SAMPLES)
    {
        return fallback;
    }

    pred = axis->slope * t_s + axis->intercept;
    if (isfinite(pred) == 0)
    {
        return fallback;
    }

    return pred;
}

/**
 * @brief 计算四元数导数
 * @param q: 当前四元数
 * @param gx_rad_s: X 轴角速度（rad/s）
 * @param gy_rad_s: Y 轴角速度（rad/s）
 * @param gz_rad_s: Z 轴角速度（rad/s）
 * @param qdot: 四元数导数输出指针
 * @retval 无
 */
static void quat_paper_research_quat_derivative(const quat_paper_research_quat_t *q,
                                                float gx_rad_s,
                                                float gy_rad_s,
                                                float gz_rad_s,
                                                quat_paper_research_quat_t *qdot)
{
    if ((q == NULL) || (qdot == NULL))
    {
        return;
    }

    qdot->w = 0.5f * (-q->x * gx_rad_s - q->y * gy_rad_s - q->z * gz_rad_s);
    qdot->x = 0.5f * (q->w * gx_rad_s + q->y * gz_rad_s - q->z * gy_rad_s);
    qdot->y = 0.5f * (q->w * gy_rad_s - q->x * gz_rad_s + q->z * gx_rad_s);
    qdot->z = 0.5f * (q->w * gz_rad_s + q->x * gy_rad_s - q->y * gx_rad_s);
}

/**
 * @brief 使用 RK2 更新研究版四元数
 * @param q: 当前四元数（输入/输出）
 * @param gx_rad_s: X 轴角速度（rad/s）
 * @param gy_rad_s: Y 轴角速度（rad/s）
 * @param gz_rad_s: Z 轴角速度（rad/s）
 * @param dt_s: 积分时间（s）
 * @retval 无
 */
static void quat_paper_research_integrate_rk2(quat_paper_research_quat_t *q,
                                              float gx_rad_s,
                                              float gy_rad_s,
                                              float gz_rad_s,
                                              float dt_s)
{
    quat_paper_research_quat_t qdot0;
    quat_paper_research_quat_t qmid;
    quat_paper_research_quat_t qdot1;
    float recip_norm;

    if (q == NULL)
    {
        return;
    }

    quat_paper_research_quat_derivative(q, gx_rad_s, gy_rad_s, gz_rad_s, &qdot0);

    qmid.w = q->w + qdot0.w * dt_s * 0.5f;
    qmid.x = q->x + qdot0.x * dt_s * 0.5f;
    qmid.y = q->y + qdot0.y * dt_s * 0.5f;
    qmid.z = q->z + qdot0.z * dt_s * 0.5f;

    quat_paper_research_quat_derivative(&qmid, gx_rad_s, gy_rad_s, gz_rad_s, &qdot1);

    q->w += qdot1.w * dt_s;
    q->x += qdot1.x * dt_s;
    q->y += qdot1.y * dt_s;
    q->z += qdot1.z * dt_s;

    recip_norm = quat_paper_research_inv_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (recip_norm > 0.0f)
    {
        q->w *= recip_norm;
        q->x *= recip_norm;
        q->y *= recip_norm;
        q->z *= recip_norm;
    }
    else
    {
        quat_paper_research_reset_quat(q);
    }
}

/**
 * @brief 四元数转欧拉角（标准 ZYX：roll-x / pitch-y / yaw-z）
 * @param q: 输入四元数
 * @retval 欧拉角（deg）
 */
static quat_paper_research_euler_t quat_paper_research_quat_to_euler(quat_paper_research_quat_t q)
{
    quat_paper_research_euler_t euler;
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);

    sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
    {
        euler.pitch = copysignf((float)(M_PI / 2.0), sinp);
    }
    else
    {
        euler.pitch = asinf(sinp);
    }

    siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp);

    euler.roll *= 180.0f / (float)M_PI;
    euler.pitch *= 180.0f / (float)M_PI;
    euler.yaw *= 180.0f / (float)M_PI;

    return euler;
}

/**
 * @brief 初始化研究版内部状态
 * @param state: 研究版状态结构体指针
 * @retval 无
 */
void quat_paper_research_init(quat_paper_research_state_t *state);

/**
 * @brief 清空研究版内部状态
 * @param state: 研究版状态结构体指针
 * @retval 无
 */
void quat_paper_research_reset(quat_paper_research_state_t *state);

/**
 * @brief 研究版一步更新
 * @param state: 研究版状态结构体指针
 * @param imu: 本周期 IMU 输入
 * @param kp: 重力纠偏比例项
 * @param ki: 重力纠偏积分项
 * @param debug: 调试输出结构体指针，可为 NULL
 * @retval 本周期解算后的欧拉角（deg）
 */
quat_paper_research_euler_t quat_paper_research_update(quat_paper_research_state_t *state,
                                                       quat_paper_research_imu_t imu,
                                                       float kp,
                                                       float ki,
                                                       quat_paper_research_debug_t *debug);

/**
 * @brief 将传感器坐标映射到云台机体系
 * @param imu: IMU 数据指针
 * @retval 无
 * @note 映射关系与当前 Gimbal/BMI088 主链保持一致
 */
void quat_paper_research_map_sensor_to_body(quat_paper_research_imu_t *imu);

/**
 * @brief 清空连续 yaw 状态
 * @param yaw_state: 连续 yaw 状态指针
 * @retval 无
 */
void quat_paper_research_yaw_reset(quat_paper_research_yaw_state_t *yaw_state);

/**
 * @brief 将原始 yaw 展开为连续角度
 * @param yaw_state: 连续 yaw 状态指针
 * @param raw_yaw_deg: 原始 yaw（deg）
 * @retval 连续 yaw（deg）
 */
float quat_paper_research_yaw_to_continuous(quat_paper_research_yaw_state_t *yaw_state, float raw_yaw_deg);

/**
 * @brief 清空研究版观测结构体
 * @param observe: 观测结构体指针
 * @param default_dt_s: 默认 dt（s）
 * @retval 无
 */
void quat_paper_research_observe_reset(quat_paper_research_observe_t *observe, float default_dt_s);

/**
 * @brief 执行一次研究版观测更新
 * @param observe: 观测结构体指针
 * @param imu: 本周期原始 IMU 输入
 * @param kp: 重力纠偏比例项
 * @param ki: 重力纠偏积分项
 * @retval 无
 * @note 该接口内部会完成坐标映射、论文版更新和连续 yaw 展开
 */
void quat_paper_research_observe_update(quat_paper_research_observe_t *observe,
                                        quat_paper_research_imu_t imu,
                                        float kp,
                                        float ki);

void quat_paper_research_reset(quat_paper_research_state_t *state)
{
    uint8_t axis;

    if (state == NULL)
    {
        return;
    }

    quat_paper_research_reset_quat(&state->quat);
    for (axis = 0u; axis < 3u; axis++)
    {
        quat_paper_research_reset_kf_axis(&state->kf[axis]);
        quat_paper_research_reset_ls_axis(&state->ls[axis]);
    }

    state->gyro_bias_dps.x = 0.0f;
    state->gyro_bias_dps.y = 0.0f;
    state->gyro_bias_dps.z = 0.0f;
    state->gyro_filt_dps.x = 0.0f;
    state->gyro_filt_dps.y = 0.0f;
    state->gyro_filt_dps.z = 0.0f;
    state->gyro_comp_dps.x = 0.0f;
    state->gyro_comp_dps.y = 0.0f;
    state->gyro_comp_dps.z = 0.0f;
    state->runtime_s = 0.0f;
    state->static_counter = 0u;
    state->vibration_counter = 0u;
    state->is_static = 0u;
    state->is_vibrating = 0u;
    state->integralFBx = 0.0f;
    state->integralFBy = 0.0f;
    state->integralFBz = 0.0f;
}

void quat_paper_research_init(quat_paper_research_state_t *state)
{
    quat_paper_research_reset(state);
}

quat_paper_research_euler_t quat_paper_research_update(quat_paper_research_state_t *state,
                                                       quat_paper_research_imu_t imu,
                                                       float kp,
                                                       float ki,
                                                       quat_paper_research_debug_t *debug)
{
    quat_paper_research_euler_t euler = {0};
    quat_paper_research_vec3_t bias_pred_dps = {0.0f, 0.0f, 0.0f};
    quat_paper_research_vec3_t innovation_dps = {0.0f, 0.0f, 0.0f};
    float dt_s;
    float acc_norm_raw;
    float acc_dev_g;
    float gyro_norm_raw_dps;
    float gyro_norm_filt_dps;
    float gyro_norm_comp_dps;
    float innovation_norm_dps;
    float recip_norm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;
    float acc_weight_scale = 0.0f;
    float gx_rad_s;
    float gy_rad_s;
    float gz_rad_s;
    float innovation_axis;
    float sample_dps;
    float bias_avg_dps;
    uint8_t axis;
    uint8_t static_candidate;
    uint8_t vibration_candidate;
    uint8_t acc_valid = 0u;

    if (state == NULL)
    {
        return euler;
    }

    if ((state->quat.w == 0.0f) && (state->quat.x == 0.0f) &&
        (state->quat.y == 0.0f) && (state->quat.z == 0.0f))
    {
        quat_paper_research_reset(state);
    }

    if (debug != NULL)
    {
        memset(debug, 0, sizeof(*debug));
    }

    if ((isfinite(state->quat.w) == 0) || (isfinite(state->quat.x) == 0) ||
        (isfinite(state->quat.y) == 0) || (isfinite(state->quat.z) == 0))
    {
        quat_paper_research_reset_quat(&state->quat);
        state->integralFBx = 0.0f;
        state->integralFBy = 0.0f;
        state->integralFBz = 0.0f;
    }

    if ((isfinite(kp) == 0) || (kp < 0.0f))
    {
        kp = 0.0f;
    }
    if ((isfinite(ki) == 0) || (ki < 0.0f))
    {
        ki = 0.0f;
    }

    if (isfinite(imu.gyro.x) == 0) imu.gyro.x = 0.0f;
    if (isfinite(imu.gyro.y) == 0) imu.gyro.y = 0.0f;
    if (isfinite(imu.gyro.z) == 0) imu.gyro.z = 0.0f;
    if (isfinite(imu.acc.x) == 0) imu.acc.x = 0.0f;
    if (isfinite(imu.acc.y) == 0) imu.acc.y = 0.0f;
    if (isfinite(imu.acc.z) == 0) imu.acc.z = 0.0f;

    dt_s = quat_paper_research_sanitize_dt(imu.dt);
    state->runtime_s += dt_s;

    for (axis = 0u; axis < 3u; axis++)
    {
        sample_dps = quat_paper_research_kf_axis_update(&state->kf[axis],
                                                        quat_paper_research_vec3_get(&imu.gyro, axis),
                                                        &innovation_axis);
        quat_paper_research_vec3_set(&state->gyro_filt_dps, axis, sample_dps);
        quat_paper_research_vec3_set(&innovation_dps, axis, innovation_axis);
    }

    gyro_norm_raw_dps = quat_paper_research_vec3_norm(imu.gyro);
    acc_norm_raw = quat_paper_research_vec3_norm(imu.acc);
    acc_dev_g = fabsf(acc_norm_raw - 1.0f);
    gyro_norm_filt_dps = quat_paper_research_vec3_norm(state->gyro_filt_dps);
    innovation_norm_dps = quat_paper_research_vec3_norm(innovation_dps);

    static_candidate = ((gyro_norm_filt_dps <= QUAT_PAPER_RESEARCH_STATIC_GYRO_THR_DPS) &&
                        (acc_dev_g <= QUAT_PAPER_RESEARCH_STATIC_ACC_DEV_THR_G)) ? 1u : 0u;
    if (static_candidate != 0u)
    {
        if (state->static_counter < UINT16_MAX)
        {
            state->static_counter++;
        }
    }
    else
    {
        state->static_counter = 0u;
    }
    state->is_static = (state->static_counter >= QUAT_PAPER_RESEARCH_STATIC_COUNT_MIN) ? 1u : 0u;

    vibration_candidate = ((acc_dev_g >= QUAT_PAPER_RESEARCH_VIB_ACC_DEV_THR_G) ||
                           (innovation_norm_dps >= QUAT_PAPER_RESEARCH_VIB_INNOV_THR_DPS)) ? 1u : 0u;
    if (vibration_candidate != 0u)
    {
        if (state->vibration_counter < UINT16_MAX)
        {
            state->vibration_counter++;
        }
    }
    else
    {
        state->vibration_counter = 0u;
    }
    state->is_vibrating = (state->vibration_counter >= QUAT_PAPER_RESEARCH_VIB_COUNT_MIN) ? 1u : 0u;

    if ((state->is_static != 0u) && (state->is_vibrating == 0u))
    {
        for (axis = 0u; axis < 3u; axis++)
        {
            sample_dps = quat_paper_research_vec3_get(&state->gyro_filt_dps, axis);
            quat_paper_research_ls_axis_add_sample(&state->ls[axis], state->runtime_s, sample_dps);

            bias_avg_dps = quat_paper_research_vec3_get(&state->gyro_bias_dps, axis);
            if (isfinite(bias_avg_dps) == 0)
            {
                bias_avg_dps = 0.0f;
            }
            bias_avg_dps += QUAT_PAPER_RESEARCH_BIAS_LP_ALPHA * (sample_dps - bias_avg_dps);
            quat_paper_research_vec3_set(&state->gyro_bias_dps, axis, bias_avg_dps);
        }
    }

    for (axis = 0u; axis < 3u; axis++)
    {
        sample_dps = quat_paper_research_ls_axis_predict(&state->ls[axis],
                                                         state->runtime_s,
                                                         quat_paper_research_vec3_get(&state->gyro_bias_dps, axis));
        quat_paper_research_vec3_set(&bias_pred_dps, axis, sample_dps);

        sample_dps = quat_paper_research_vec3_get(&state->gyro_filt_dps, axis) - sample_dps;
        if (fabsf(sample_dps) < QUAT_PAPER_RESEARCH_BIAS_DEADBAND_DPS)
        {
            sample_dps = 0.0f;
        }
        if (isfinite(sample_dps) == 0)
        {
            sample_dps = 0.0f;
        }
        quat_paper_research_vec3_set(&state->gyro_comp_dps, axis, sample_dps);
    }

    gyro_norm_comp_dps = quat_paper_research_vec3_norm(state->gyro_comp_dps);

    if ((isfinite(acc_norm_raw) != 0) && (acc_norm_raw > QUAT_PAPER_RESEARCH_ACC_NORM_VALID_MIN_G))
    {
        recip_norm = quat_paper_research_inv_sqrt(imu.acc.x * imu.acc.x + imu.acc.y * imu.acc.y + imu.acc.z * imu.acc.z);
        if (recip_norm > 0.0f)
        {
            imu.acc.x *= recip_norm;
            imu.acc.y *= recip_norm;
            imu.acc.z *= recip_norm;
            acc_valid = 1u;
        }
    }

    gx_rad_s = state->gyro_comp_dps.x * QUAT_PAPER_RESEARCH_DEG_TO_RAD;
    gy_rad_s = state->gyro_comp_dps.y * QUAT_PAPER_RESEARCH_DEG_TO_RAD;
    gz_rad_s = state->gyro_comp_dps.z * QUAT_PAPER_RESEARCH_DEG_TO_RAD;

    if (acc_valid != 0u)
    {
        halfvx = state->quat.x * state->quat.z - state->quat.w * state->quat.y;
        halfvy = state->quat.w * state->quat.x + state->quat.y * state->quat.z;
        halfvz = state->quat.w * state->quat.w - 0.5f + state->quat.z * state->quat.z;

        halfex = (imu.acc.y * halfvz - imu.acc.z * halfvy);
        halfey = (imu.acc.z * halfvx - imu.acc.x * halfvz);
        halfez = (imu.acc.x * halfvy - imu.acc.y * halfvx);

        acc_weight_scale = (state->is_vibrating != 0u) ? QUAT_PAPER_RESEARCH_ACC_WEIGHT_VIB_SCALE : 1.0f;

        if (ki > 0.0f)
        {
            state->integralFBx += 2.0f * ki * halfex * dt_s * acc_weight_scale;
            state->integralFBy += 2.0f * ki * halfey * dt_s * acc_weight_scale;
            state->integralFBz += 2.0f * ki * halfez * dt_s * acc_weight_scale;
            gx_rad_s += state->integralFBx;
            gy_rad_s += state->integralFBy;
            gz_rad_s += state->integralFBz;
        }
        else
        {
            state->integralFBx = 0.0f;
            state->integralFBy = 0.0f;
            state->integralFBz = 0.0f;
        }

        gx_rad_s += 2.0f * kp * halfex * acc_weight_scale;
        gy_rad_s += 2.0f * kp * halfey * acc_weight_scale;
        gz_rad_s += 2.0f * kp * halfez * acc_weight_scale;
    }

    quat_paper_research_integrate_rk2(&state->quat, gx_rad_s, gy_rad_s, gz_rad_s, dt_s);
    euler = quat_paper_research_quat_to_euler(state->quat);

    if (debug != NULL)
    {
        debug->acc_norm_raw = (isfinite(acc_norm_raw) != 0) ? acc_norm_raw : 0.0f;
        debug->gyro_norm_raw_dps = gyro_norm_raw_dps;
        debug->gyro_norm_comp_dps = gyro_norm_comp_dps;
        debug->bias_pred_dps = bias_pred_dps;
        debug->innovation_dps = innovation_dps;
        debug->static_state = state->is_static;
        debug->vibration_state = state->is_vibrating;
        debug->acc_weight_scale = acc_weight_scale;
        debug->dt_used = dt_s;
    }

    return euler;
}

void quat_paper_research_map_sensor_to_body(quat_paper_research_imu_t *imu)
{
    float sx;
    float sy;

    if (imu == NULL)
    {
        return;
    }

    sx = imu->gyro.x;
    sy = imu->gyro.y;
    imu->gyro.x = sy;
    imu->gyro.y = -sx;

    sx = imu->acc.x;
    sy = imu->acc.y;
    imu->acc.x = sy;
    imu->acc.y = -sx;
}

static float quat_paper_research_wrap180(float angle_deg)
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

void quat_paper_research_yaw_reset(quat_paper_research_yaw_state_t *yaw_state)
{
    if (yaw_state == NULL)
    {
        return;
    }

    memset(yaw_state, 0, sizeof(*yaw_state));
}

float quat_paper_research_yaw_to_continuous(quat_paper_research_yaw_state_t *yaw_state, float raw_yaw_deg)
{
    float yaw_rel_wrapped_deg;
    float dyaw_deg;

    if (yaw_state == NULL)
    {
        return 0.0f;
    }

    if (isfinite(raw_yaw_deg) == 0)
    {
        return yaw_state->continuous_deg;
    }

    if (yaw_state->inited == 0u)
    {
        yaw_state->zero_raw_deg = raw_yaw_deg;
        yaw_state->last_rel_wrapped_deg = 0.0f;
        yaw_state->continuous_deg = 0.0f;
        yaw_state->inited = 1u;
        return 0.0f;
    }

    yaw_rel_wrapped_deg = quat_paper_research_wrap180(raw_yaw_deg - yaw_state->zero_raw_deg);
    dyaw_deg = yaw_rel_wrapped_deg - yaw_state->last_rel_wrapped_deg;

    if (dyaw_deg > 180.0f)
    {
        dyaw_deg -= 360.0f;
    }
    else if (dyaw_deg < -180.0f)
    {
        dyaw_deg += 360.0f;
    }

    yaw_state->continuous_deg += dyaw_deg;
    yaw_state->last_rel_wrapped_deg = yaw_rel_wrapped_deg;

    return yaw_state->continuous_deg;
}

void quat_paper_research_observe_reset(quat_paper_research_observe_t *observe, float default_dt_s)
{
    if (observe == NULL)
    {
        return;
    }

    memset(observe, 0, sizeof(*observe));
    quat_paper_research_init(&observe->paper_state);
    quat_paper_research_yaw_reset(&observe->yaw_state);
    observe->dt_s = quat_paper_research_sanitize_dt(default_dt_s);
}

void quat_paper_research_observe_update(quat_paper_research_observe_t *observe,
                                        quat_paper_research_imu_t imu,
                                        float kp,
                                        float ki)
{
    if (observe == NULL)
    {
        return;
    }

    observe->imu_input = imu;
    observe->imu_input_body = imu;
    quat_paper_research_map_sensor_to_body(&observe->imu_input_body);
    observe->dt_s = quat_paper_research_sanitize_dt(imu.dt);
    observe->paper_euler_raw = quat_paper_research_update(&observe->paper_state,
                                                          observe->imu_input_body,
                                                          kp,
                                                          ki,
                                                          &observe->paper_debug);
    observe->paper_euler = observe->paper_euler_raw;
    observe->paper_euler.yaw = quat_paper_research_yaw_to_continuous(&observe->yaw_state,
                                                                     observe->paper_euler_raw.yaw);
    observe->update_count++;
}
