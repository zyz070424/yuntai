#ifndef __ALG_DWT_H__
#define __ALG_DWT_H__

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint8_t dt_ready;
    uint8_t dwt_ready;
    uint32_t last_cycle;
    uint32_t last_tick_ms;
    float last_dt_s;
    uint8_t last_dt_from_dwt;
} alg_dwt_timebase_t;

void ALG_DWT_Timebase_Init(alg_dwt_timebase_t *timebase, float default_dt_s);
float ALG_DWT_Timebase_GetDtS(alg_dwt_timebase_t *timebase,
                              float default_dt_s,
                              float dt_min_s,
                              float dt_max_s);

#endif /* __ALG_DWT_H__ */
