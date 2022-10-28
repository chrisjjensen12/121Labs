/*********************************************************************
*
* Copyright (C) 2022 David C. Harrison. All right reserved.
*
* You may not use, distribute, publish, or modify this code without
* the express written permission of the copyright holder.
*
*********************************************************************/
/*********************************************************************
* DO NOT MODIFY THIS FILE
*********************************************************************/
#include <stdbool.h>
#include <inttypes.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
enum color { White, Red, Green, Blue };
enum direction { Left, Right };
enum mode { Off, On };
/*!
 *  @brief Device I2C Arress
 */
#define LCD_ADDRESS     (0x7c>>1)
#define RGB_ADDRESS     (0xc0>>1)


/*!
 *  @brief color define
 */
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04        // pwm2
#define REG_GREEN       0x03        // pwm1
#define REG_BLUE        0x02        // pwm0

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

/*!
 *  @brief commands
 */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/*!
 *  @brief flags for display entry mode
 */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/*!
 *  @brief flags for display on/off control
 */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/*!
 *  @brief flags for display/cursor shift
 */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/*!
 *  @brief flags for function set
 */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Connects PSoC 6 I2C MASTER with associated CONTEXT to a connected DRF0554
// 16x2 backlight RGB LCD. Text direction will be fixed left-to-right.
// Initial settings:
// Display On
// Color White
// Blink Off
// Cursor Off
// Autoscroll Off
// Returns FALSE on error, TRUE otherwise.
bool LCD_Start();

// Turns the display on if MODE is On, turns display off otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Display(enum mode);

// Sets the display to blink if MODE is On, non-blinking otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Blink(enum mode);

// Shows the cursor if MODE is On, hides it otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Cursor(enum mode);

// Scrolls display on char left if DIRECTION is Left, one char right otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Scroll(enum direction);

// Sets display color if COLOR is White, Red, Green, or Blue, no-op otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetColor(enum color);

// Sets display color according to RED, GREEN, BLUE.
// Example: LCD_SetRGB(0, 255, 255) for cyan.
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetRGB(uint8_t red, uint8_t green, uint8_t blue);

// Positions the cursor at COL and ROW where 0<=COL<=15 and 0<=ROW<=1
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetCursor(uint8_t col, uint8_t row);

// Clears the display.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Clear();

// Prints all characters from zero terminated string STR.
// If strlen(str) > 16, call LCD_Autoscroll(On) first.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Print(char *str);


//auto scrolls display
bool LCD_Autoscroll(enum mode);


