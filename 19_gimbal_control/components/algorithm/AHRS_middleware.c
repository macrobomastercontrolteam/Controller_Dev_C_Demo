/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      Attitude-estimation middleware providing supporting functions.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Completed.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"
/**
 * @brief          Get the current altitude.
 * @author         RM
 * @param[in]      Altitude pointer, fp32
 * @retval         None
 */

void AHRS_get_height(fp32* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          Get the current latitude.
 * @author         RM
 * @param[in]      Latitude pointer, fp32
 * @retval         None
 */

void AHRS_get_latitude(fp32* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          Fast inverse-square-root function.
 * @author         RM
 * @param[in]      Input floating-point value, fp32
 * @retval         Reciprocal square root, 1/sqrt(input)
 */

fp32 AHRS_invSqrt(fp32 num)
{
    return 1/sqrtf(num);

//    fp32 halfnum = 0.5f * num;
//    fp32 y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(fp32*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          Sine function.
 * @author         RM
 * @param[in]      Angle, in radians
 * @retval         Sine of the angle
 */

fp32 AHRS_sinf(fp32 angle)
{
    return arm_sin_f32(angle);
}
/**
 * @brief          Cosine function.
 * @author         RM
 * @param[in]      Angle, in radians
 * @retval         Cosine of the angle
 */

fp32 AHRS_cosf(fp32 angle)
{
    return arm_cos_f32(angle);
}

/**
 * @brief          Tangent function.
 * @author         RM
 * @param[in]      Angle, in radians
 * @retval         Tangent of the angle
 */

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
 * @brief          Single-precision inverse sine (asin).
 * @author         RM
 * @param[in]      Sine value, range [-1.0f, 1.0f]
 * @retval         Angle, in radians
 */

fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}

/**
 * @brief          Inverse cosine (acos).
 * @author         RM
 * @param[in]      Cosine value, range [-1.0f, 1.0f]
 * @retval         Corresponding angle, in radians
 */

fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}

/**
 * @brief          Inverse tangent function.
 * @author         RM
 * @param[in]      y component, ranging from negative to positive infinity
 * @param[in]      x component, ranging from negative to positive infinity
 * @retval         Corresponding angle, in radians
 */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}
