#include "bsp_led.h"
#include "main.h"
extern TIM_HandleTypeDef htim5;

volatile uint16_t dbg_red_ccr = 0;
volatile uint16_t dbg_green_ccr = 0;
volatile uint16_t dbg_blue_ccr = 0;

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
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
    dbg_red_ccr = red;
    dbg_green_ccr = green;
    dbg_blue_ccr = blue;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
