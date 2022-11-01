/*********************************************************************
*
* Copyright (C) 2022 David C. Harrison. All right reserved.
*
* You may not use, distribute, publish, or modify this code without
* the express written permission of the copyright holder.
*
*********************************************************************/
#include "DRF0554.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include <stdbool.h>
#include "cy_retarget_io.h"
#define I2C_HW SCB6
void LCD_begin(uint8_t cols, uint8_t lines);
static void i2c_send_byteS(uint8_t *data, uint32_t addr, uint32_t len);
static void LCD_setReg(unsigned char reg, unsigned char val);
void LCD_command(uint8_t value);
void LCD_setColorWhite(void);
void blinkLED(void);
void LCD_write(uint8_t value);
cy_en_scb_i2c_status_t status;
uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _initialized;
uint8_t _numlines,_currline;
cy_stc_scb_i2c_context_t i2cContext;
const uint8_t color_define[4][3] =
{
    {255, 255, 255},            // white
    {255, 0, 0},                // red
    {0, 255, 0},                // green
    {0, 0, 255},                // blue
};
int counter;

///////////// DRF0554 functions //////////////

//initializes the LCD
bool LCD_Start(){
	counter = 0;
	//initialize I2C
	Cy_SCB_I2C_Init(I2C_HW, &I2C_config, &i2cContext);
	Cy_SCB_I2C_Enable(I2C_HW);

	printf("\nScanning I2C 7-Bit Addresses…\r\n");
	uint32_t rval;

	// Iterate through the address starting at 0x00
	for(uint32_t address=0; address<0x7F; address++)
	{
		rval = Cy_SCB_I2C_MasterSendStart(I2C_HW, address, CY_SCB_I2C_WRITE_XFER, 100, &i2cContext);
		if(rval == CY_SCB_I2C_SUCCESS)
		{
			printf("Found: 0x%02X\r\n", (unsigned int)address);
		}
		Cy_SCB_I2C_MasterSendStop(I2C_HW, 0, &i2cContext);
	}

	printf("Scan complete\r\n\n");

	//initialize LCD
	_displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	LCD_begin(16, 2);
	return true;
}

// Turns the display on if MODE is On, turns display off otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Display(enum mode displayMode){

	if(displayMode == 1){
//		printf("Display On!\r\n");
		_displaycontrol |= LCD_DISPLAYON;
		LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}else if(displayMode == 0){
//		printf("Display Off!\r\n");
		_displaycontrol &= ~LCD_DISPLAYON;
		LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	CyDelay(500);
	return true;
}

// Sets the display to blink if MODE is On, non-blinking otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Blink(enum mode blinkMode){

	if(blinkMode == 1){
//		printf("LCD Blink On\r\n");
		_displaycontrol |= LCD_BLINKON;
		LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}else if(blinkMode == 0){
//		printf("LCD Blink Off\r\n");
		_displaycontrol &= ~LCD_BLINKON;
		LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	CyDelay(500);
	return true;
}

// Shows the cursor if MODE is On, hides it otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Cursor(enum mode cursorMode){

	if(cursorMode == 1){
//		printf("LCD Cursor On\r\n");
		_displaycontrol |= LCD_CURSORON;
	    LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}else if(cursorMode == 0){
//		printf("LCD Cursor Off\r\n");
		_displaycontrol &= ~LCD_CURSORON;
	    LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	CyDelay(500);
	return true;
}


// Scrolls display on char left if DIRECTION is Left, one char right otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Scroll(enum direction scrollDirection){
	if(scrollDirection == 1){ //right
//		printf("Scroll Right\r\n");
		LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
	}else if(scrollDirection == 0){ //left
//		printf("Scroll Left\r\n");
		LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
	}

	CyDelay(500);
	return true;
}

// Sets display color if COLOR is White, Red, Green, or Blue, no-op otherwise.
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetColor(enum color displayColor){
	if(displayColor > 3){
		return false;
	}

//	printf("Setting Color\r\n");
	LCD_SetRGB(color_define[displayColor][0], color_define[displayColor][1], color_define[displayColor][2]);

	CyDelay(500);
	return true;
}

// Sets display color according to RED, GREEN, BLUE.
// Example: LCD_SetRGB(0, 255, 255) for cyan.
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetRGB(uint8_t red, uint8_t green, uint8_t blue){

//	printf("Setting RGB x3\r\n");
	LCD_setReg(REG_RED, red);
	LCD_setReg(REG_GREEN, green);
	LCD_setReg(REG_BLUE, blue);

	CyDelay(500);
	return true;
}

// Positions the cursor at COL and ROW where 0<=COL<=15 and 0<=ROW<=1
// Returns FALSE on error, TRUE otherwise.
bool LCD_SetCursor(uint8_t col, uint8_t row){

    col = (row == 0 ? col|0x80 : col|0xc0);
    uint8_t data[3] = {0x80, col};

    i2c_send_byteS(data, LCD_ADDRESS, 2);

    CyDelay(500);
	return true;
}

// Clears the display.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Clear(){
//	printf("LCD Clear\r\n");
	LCD_command(LCD_CLEARDISPLAY);        // clear display, set cursor position to zero
	CyDelay(2000); // this command takes a long time!
	return true;
}


// Prints all characters from zero terminated string STR.
// If strlen(str) > 16, call LCD_Autoscroll(On) first.
// Returns FALSE on error, TRUE otherwise.
bool LCD_Print(char *str){
	uint8_t size = strlen(str);

	for(int i = 0; i < size; i++){
		LCD_write(str[i]);
	}

//	CyDelay(500);
	return true;
}

//auto scrolls display
bool LCD_Autoscroll(enum mode autoScrollMode){

	if(autoScrollMode == 1){
//		printf("LCD Auto scroll on\r\n");
		_displaymode |= LCD_ENTRYSHIFTINCREMENT;
		LCD_command(LCD_ENTRYMODESET | _displaymode);
	}else if(autoScrollMode == 0){
//		printf("LCD Auto scroll off\r\n");
	    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
		LCD_command(LCD_ENTRYMODESET | _displaymode);
	}

	CyDelay(500);
	return true;
}

///////////// Helper functions //////////////

static void i2c_send_byteS(uint8_t *data, uint32_t addr, uint32_t len)
{
	counter++;
	uint32_t device_address_sent_ack;
//	uint32_t internal_register_address_sent_ack;
	uint32_t data_byte;
	uint32_t transmission_end;
//	printf("Counter = %d\r\n", counter);
	//send start bits and device slave address
	device_address_sent_ack = Cy_SCB_I2C_MasterSendStart(I2C_HW, addr, CY_SCB_I2C_WRITE_XFER, 100, &i2cContext);
	if(device_address_sent_ack == CY_SCB_I2C_SUCCESS)
	{
//		printf("Success sending device address!\r\n");
	}else{
//		printf("Error sending device address\r\n");
	}

	//send actual data
	for(int i=0; i<len; i++) {
		data_byte = Cy_SCB_I2C_MasterWriteByte(I2C_HW, data[i], 100, &i2cContext);
		if(data_byte == CY_SCB_I2C_SUCCESS){
//			printf("Data byte written\r\n");
		}else{
//			printf("Error writing data byte\r\n");
		}
		CyDelay(5);
	}

	transmission_end = Cy_SCB_I2C_MasterSendStop(I2C_HW, 100, &i2cContext);
	if(transmission_end == CY_SCB_I2C_SUCCESS){
//		printf("Transmission ended\r\n");
	}else{
//		printf("Error ending transmission\r\n");
	}
//	printf("\r\n");
}



void LCD_command(uint8_t value){
    unsigned char dta[3] = {0x80, value};
    i2c_send_byteS(dta, LCD_ADDRESS, 2);
}

static void LCD_setReg(unsigned char reg, unsigned char val){
	uint8_t data[2] = {reg, val};
	i2c_send_byteS(data, RGB_ADDRESS, 2);
}

void LCD_write(uint8_t value){
    unsigned char dta[2] = {0x40, value};
    i2c_send_byteS(dta, LCD_ADDRESS, 2);
}

void LCD_begin(uint8_t cols, uint8_t lines){

	if (lines > 1) {
		_displayfunction |=  LCD_2LINE;
	}

	_numlines = lines;
    _currline = 0;

	CyDelay(50);	// 50-msec delay

	// Send function set command sequence
	LCD_command(LCD_FUNCTIONSET | _displayfunction);
	CyDelay(5);  // wait more than 4.1ms

	// second try
	LCD_command(LCD_FUNCTIONSET | _displayfunction);
	CyDelay(5);

	// third go
	LCD_command(LCD_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	LCD_Display(On);

	// clear it off
	LCD_Clear();

	// Initialize to default text direction (for romance languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	LCD_command(LCD_ENTRYMODESET | _displaymode);

	// backlight init
	LCD_setReg(REG_MODE1, 0);
	LCD_setReg(REG_OUTPUT, 0xFF);
	LCD_setReg(REG_MODE2, 0x20);     // all led control by pwm

	LCD_setColorWhite();

}



void LCD_setColorWhite(void){
	LCD_SetRGB(255, 255, 255);
}

void blinkLED(void){
	LCD_setReg(0x07, 0x17);  // blink every second
	LCD_setReg(0x06, 0x7f);  // half on, half off
}
