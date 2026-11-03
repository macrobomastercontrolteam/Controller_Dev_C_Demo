#ifndef BSP_RNG_H
#define BSP_RNG_H

#include "global_inc.h"

uint32_t RNG_get_random_num(void);
int32_t RNG_get_random_range_int32(int min, int max);
fp32 RNG_get_random_range_fp32(fp32 min, fp32 max);

#endif
