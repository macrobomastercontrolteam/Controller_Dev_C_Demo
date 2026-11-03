#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"

/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          Display an RGB color.
  * @param[in]      aRGB: 0xaaRRGGBB; aa = alpha, RR = red, GG = green, BB = blue
  * @retval         none
  */
extern void aRGB_led_show(uint32_t aRGB);


#endif
