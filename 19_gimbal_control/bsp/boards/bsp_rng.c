#include "bsp_rng.h"

static uint32_t rng_state = 0x6d2b79f5U;

uint32_t RNG_get_random_num(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

int32_t RNG_get_random_range_int32(int min, int max)
{
    return (int32_t)(RNG_get_random_num() % (uint32_t)(max - min + 1)) + min;
}

fp32 RNG_get_random_range_fp32(fp32 min, fp32 max)
{
    return ((fp32)RNG_get_random_num() / (fp32)UINT32_MAX) * (max - min) + min;
}
