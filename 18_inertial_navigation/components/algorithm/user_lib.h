#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"

typedef __PACKED_STRUCT
{
    fp32 input;        //Input data.
    fp32 out;          //Output data.
    fp32 min_value;    //Minimum limit.
    fp32 max_value;    //Maximum limit.
    fp32 frame_period; //Time interval.
} ramp_function_source_t;

typedef __PACKED_STRUCT
{
    fp32 input;        //Input data.
    fp32 out;          //Filtered output data.
    fp32 num[1];       //Filter coefficient.
    fp32 frame_period; //Filter time interval, in seconds.
} first_order_filter_type_t;
//Fast inverse square root.
extern fp32 invSqrt(fp32 num);

//Initialize the ramp generator.
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//Calculate the ramp output.
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//Initialize the first-order filter.
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//Calculate the first-order filter output.
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//Limit absolute magnitude.
extern void abs_limit(fp32 *num, fp32 Limit);
//Determine the sign.
extern fp32 sign(fp32 value);
//Floating-point deadband.
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int16 deadband.
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//Clamp to the limits.
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//Clamp to the limits.
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//Wrap within the limits.
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//Limit the angle to [-180, 180] degrees.
extern fp32 theta_format(fp32 Ang);

//Wrap radians to [-PI, PI].
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
