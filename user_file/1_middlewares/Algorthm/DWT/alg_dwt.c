#include "alg_dwt.h"
#include <math.h>

#define ALG_DWT_FALLBACK_DEFAULT_DT_S 0.001f
/*
 * @brief   消除无效时间间隔并进行限幅
 * @param   dt: 输入时间间隔（秒）
 * @param   default_dt_s: 默认时间间隔（秒）
 * @param   dt_min_s: 最小时间间隔（秒）
 * @param   dt_max_s: 最大时间间隔（秒）
 * @retval  限幅后的时间间隔（秒）
 */
static float ALG_DWT_Sanitize_And_Clamp_Dt(float dt,
                                           float default_dt_s,
                                           float dt_min_s,
                                           float dt_max_s)
{
    if ((isfinite(default_dt_s) == 0) || (default_dt_s <= 0.0f))
    {
        default_dt_s = ALG_DWT_FALLBACK_DEFAULT_DT_S;
    }

    if ((isfinite(dt_min_s) == 0) || (dt_min_s <= 0.0f))
    {
        dt_min_s = default_dt_s;
    }

    if ((isfinite(dt_max_s) == 0) || (dt_max_s <= 0.0f))
    {
        dt_max_s = default_dt_s;
    }

    if (dt_min_s > dt_max_s)
    {
        float temp = dt_min_s;
        dt_min_s = dt_max_s;
        dt_max_s = temp;
    }

    if ((isfinite(dt) == 0) || (dt <= 0.0f))
    {
        dt = default_dt_s;
    }

    if (dt < dt_min_s)
    {
        dt = dt_min_s;
    }
    else if (dt > dt_max_s)
    {
        dt = dt_max_s;
    }

    return dt;
}
/**
 * @brief   初始化DWT时间基准
 * @param   timebase: DWT时间基准对象指针
 * @param   default_dt_s: 默认时间间隔（秒）
 * @retval  无
 */
void ALG_DWT_Timebase_Init(alg_dwt_timebase_t *timebase, float default_dt_s)
{
    if (timebase == NULL)
    {
        return;
    }

    default_dt_s = ALG_DWT_Sanitize_And_Clamp_Dt(default_dt_s, default_dt_s, default_dt_s, default_dt_s);

    timebase->dt_ready = 0u;
    timebase->dwt_ready = 0u;
    timebase->last_cycle = 0u;
    timebase->last_tick_ms = HAL_GetTick();
    timebase->last_dt_s = default_dt_s;
    timebase->last_dt_from_dwt = 0u;

#if defined(CoreDebug) && defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    timebase->dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u) ? 1u : 0u;
    timebase->last_cycle = DWT->CYCCNT;
#endif
}
/**
 * @brief   获取时间间隔（秒）
 * @param   timebase: DWT时间基准对象指针
 * @param   default_dt_s: 默认时间间隔（秒）
 * @param   dt_min_s: 最小时间间隔（秒）
 * @param   dt_max_s: 最大时间间隔（秒）
 * @retval  获取到的时间间隔（秒）
 */
float ALG_DWT_Timebase_GetDtS(alg_dwt_timebase_t *timebase,
                              float default_dt_s,
                              float dt_min_s,
                              float dt_max_s)
{
    float dt;
    uint32_t now_tick_ms;

    if (timebase == NULL)
    {
        return ALG_DWT_Sanitize_And_Clamp_Dt(default_dt_s, default_dt_s, dt_min_s, dt_max_s);
    }

    dt = default_dt_s;
    now_tick_ms = HAL_GetTick();

#if defined(CoreDebug) && defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if ((timebase->dwt_ready != 0u) && (SystemCoreClock > 0u) &&
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u))
    {
        uint32_t now_cycle = DWT->CYCCNT;
        if (timebase->dt_ready != 0u)
        {
            uint32_t delta_cycle = now_cycle - timebase->last_cycle;
            dt = (float)delta_cycle / (float)SystemCoreClock;
            timebase->last_dt_from_dwt = 1u;
        }
        timebase->last_cycle = now_cycle;
    }
    else
#endif
    {
        if (timebase->dt_ready != 0u)
        {
            uint32_t delta_tick_ms = now_tick_ms - timebase->last_tick_ms;
            if (delta_tick_ms > 0u)
            {
                dt = (float)delta_tick_ms * 0.001f;
            }
            timebase->last_dt_from_dwt = 0u;
        }
    }

    timebase->last_tick_ms = now_tick_ms;
    timebase->dt_ready = 1u;
    dt = ALG_DWT_Sanitize_And_Clamp_Dt(dt, default_dt_s, dt_min_s, dt_max_s);
    timebase->last_dt_s = dt;

    return dt;
}
