  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled.c/h
  * @brief      0.96 inch oled use SSD1306 driver. the file includes oled initialization function,
  *             and some OLED setting function, GRAM operate function, oled show num ,char and string function,
  *             show RoboMaster LOGO function.
  *             The 0.96-inch OLED uses an SSD1306; this file provides initialization, display configuration, GRAM operations, and number, character, and string output,
  *             as well as a function to display the RoboMaster logo.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "OLED.h" 
#include "oledfont.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c2;

static uint8_t OLED_GRAM[128][8];



/**
  * @brief          write data/command to OLED, if you use spi, please rewrite the function
  * @param[in]      dat: the data ready to write
  * @param[in]      cmd: OLED_CMD means command; OLED_DATA means data
  * @retval         none
  */
/**
  * @brief          Write data or a command to the OLED; replace this function when using SPI.
  * @param[in]      dat: byte to write
  * @param[in]      cmd: OLED_CMD selects a command; OLED_DATA selects data
  * @retval         none
  */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, cmd_data, 2, 10);
}


/**
  * @brief          initialize the oled device
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Initialize the OLED module.
  * @param[in]      none
  * @retval         none
  */
void OLED_init(void)
{
    oled_write_byte(0xAE, OLED_CMD);    //display off
    oled_write_byte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
    oled_write_byte(0x10, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0xb0, OLED_CMD);    //Set Page Start Address for Page Addressing Mode,0-7
    oled_write_byte(0xc8, OLED_CMD);    //Set COM Output Scan Direction
    oled_write_byte(0x00, OLED_CMD);    //---set low column address
    oled_write_byte(0x10, OLED_CMD);    //---set high column address
    oled_write_byte(0x40, OLED_CMD);    //--set start line address
    oled_write_byte(0x81, OLED_CMD);    //--set contrast control register
    oled_write_byte(0xff, OLED_CMD);    //brightness 0x00~0xff
    oled_write_byte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127
    oled_write_byte(0xa6, OLED_CMD);    //--set normal display
    oled_write_byte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
    oled_write_byte(0x3F, OLED_CMD);    //
    oled_write_byte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xd3, OLED_CMD);    //-set display offset
    oled_write_byte(0x00, OLED_CMD);    //-not offset
    oled_write_byte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xf0, OLED_CMD);    //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD);    //
    oled_write_byte(0xda, OLED_CMD);    //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD);    //--set vcomh
    oled_write_byte(0x20, OLED_CMD);    //0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD);    //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD);    //
    oled_write_byte(0xaf, OLED_CMD);    //--turn on oled panel
}

/**
  * @brief          turn on OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Turn on the OLED display.
  * @param[in]      none
  * @retval         none
  */
void OLED_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
  * @brief          turn off OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Turn off the OLED display.
  * @param[in]      none
  * @retval         none
  */
void OLED_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          Operate on the GRAM buffer (128 * 8 characters).
  * @param[in]      pen: operation type
                    PEN_CLEAR: set to 0x00
                    PEN_WRITE: set to 0xff
                    PEN_INVERSION: invert each bit
  * @retval         none
  */
void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                OLED_GRAM[n][i] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                OLED_GRAM[n][i] = 0x00;
            }
            else
            {
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
            }
        }
    }
}

/**
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          Set the cursor position to (x, y).
  * @param[in]      x: x coordinate, from 0 to 127
  * @param[in]      y: y coordinate, from 0 to 7
  * @retval         none
  */
void OLED_set_pos(uint8_t x, uint8_t y)
{
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y
    oled_write_byte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
    oled_write_byte((x&0x0f), OLED_CMD);                //set column low address
}


/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          Modify one GRAM bit, corresponding to one display pixel.
  * @param[in]      x: x coordinate, [0, X_WIDTH - 1]
  * @param[in]      y: y coordinate, [0, Y_WIDTH - 1]
  * @param[in]      pen: operation type
                        PEN_CLEAR: set pixel (x, y) to 0
                        PEN_WRITE: set pixel (x, y) to 1
                        PEN_INVERSION: invert pixel (x, y)
  * @retval         none
  */
void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        OLED_GRAM[x][page] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        OLED_GRAM[x][page] ^= 1 << row;
    }
    else
    {
        OLED_GRAM[x][page] &= ~(1 << row);
    }
}




/**
  * @brief          draw a line from (x1, y1) to (x2, y2)
  * @param[in]      x1: the start point of line
  * @param[in]      y1: the start point of line
  * @param[in]      x2: the end point of line
  * @param[in]      y2: the end point of line
  * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
/**
  * @brief          Draw a straight line from (x1, y1) to (x2, y2).
  * @param[in]      x1: starting x coordinate
  * @param[in]      y1: starting y coordinate
  * @param[in]      x2: ending x coordinate
  * @param[in]      y2: ending y coordinate
  * @param[in]      pen: operation type: PEN_CLEAR, PEN_WRITE, or PEN_INVERSION
  * @retval         none
  */
  
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}


/**
  * @brief          show a character
  * @param[in]      row: start row of character
  * @param[in]      col: start column of character
  * @param[in]      chr: the character ready to show
  * @retval         none
  */
/**
  * @brief          Display one character.
  * @param[in]      row: starting row of the character
  * @param[in]      col: starting column of the character
  * @param[in]      chr: character
  * @retval         none
  */
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}


/**
  * @brief          show a character string
  * @param[in]      row: row of character string begin
  * @param[in]      col: column of character string begin
  * @param[in]      chr: the pointer to character string
  * @retval         none
  */
/**
  * @brief          Display a string.
  * @param[in]      row: starting row of the string
  * @param[in]      col: starting column of the string
  * @param[in]      chr: string
  * @retval         none
  */
void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}


/**
  * @brief          formatted output in oled 128*64
  * @param[in]      row: row of character string begin, 0 <= row <= 4;
  * @param[in]      col: column of character string begin, 0 <= col <= 20;
  * @param          *fmt: the pointer to format character string
  * @note           if the character length is more than one row at a time, the extra characters will be truncated
  * @retval         none
  */
/**
  * @brief          Format and display output.
  * @param[in]      row: starting row, 0 <= row <= 4
  * @param[in]      col: starting column, 0 <= col <= 20
  * @param[in]      *fmt: format string
  * @note           Strings longer than one line wrap onto the next line.
  * @retval         none
  */
void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
    static uint8_t LCD_BUF[128] = {0};
    static va_list ap;
    uint8_t remain_size = 0;

    if ((row > 4) || (col > 20) )
    {
        return;
    }
    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    OLED_show_string(row, col, LCD_BUF);
}

/**
  * @brief          send the data of gram to oled sreen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Send data to the OLED GRAM.
  * @param[in]      none
  * @retval         none
  */
void OLED_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        OLED_set_pos(0, i);
        for (n = 0; n < 128; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}



/**
  * @brief          show the logo of RoboMaster
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Display the RM logo.
  * @param[in]      none
  * @retval         none
  */
void OLED_LOGO(void)
{
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    OLED_operate_gram(PEN_CLEAR);


    for(; y < 64; y += 8)
    {
        for(x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x][y/8];
            for(i = 0; i < 8; i++)
            {
                if(temp_char & 0x80)
                {
                    OLED_draw_point(x, y + i,PEN_WRITE);
                }
                else
                {
                    OLED_draw_point(x,y + i,PEN_CLEAR);
                }
                temp_char <<= 1;
            }
        }
    }
    OLED_refresh_gram();
}









