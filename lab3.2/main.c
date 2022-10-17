/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "DRF0554.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
void showUsage(void);
void getUserInput(void);
void handleCharacterInput(char c);
//flags
int displayOnOffFlag;
int blinkingOnOffFlag;
int cursorOnOffFlag;


void showUsage(void){
	printf("\r\n");
	printf("Usage: \r\n'o': Switch Display On/Off\r\n'b': Switch Blinking On/Off\r\n");
	printf("'c': Switch Cursor On/Off\r\n'm': Move Cursor\r\n'l': Scroll Left\r\n");
	printf("'r': Scroll Right\r\n'W': Set Color White\r\n'R': Set Color Red\r\n'Y': Set Color Yellow\r\n");
	printf("'G': Set Color Green\r\n'B': Set Color Blue\r\n'P': Set Custom Color Pink\r\n'O': Set Custom Color Orange\r\n");
	printf("'h' Show Usage\r\n");
	printf("\r\n");
}

void getUserInput(void){

	 char c;

	 setvbuf(stdin, NULL, _IONBF, 0);

	 c = getchar();

	 printf("You Entered: %c\r\n", c);

	 handleCharacterInput(c);

}

void handleCharacterInput(char c){

	switch(c){

		case 'o':
			if(displayOnOffFlag == 1){
				printf("Turning display off...\r\n");
				LCD_Display(Off);
				displayOnOffFlag = 0;
			}else{
				printf("Turning display on...\r\n");
				LCD_Display(On);
				displayOnOffFlag = 1;
			}
			break;

		case 'b':
			if(blinkingOnOffFlag == 1){
				printf("Stop Blinking...\r\n");
				LCD_Blink(Off);
				blinkingOnOffFlag = 0;
			}else{
				printf("Start Blinking...\r\n");
				LCD_Blink(On);
				blinkingOnOffFlag = 1;
			}
			break;

		case 'c':
			if(cursorOnOffFlag == 1){
				printf("Stop Cursor...\r\n");
				LCD_Cursor(Off);
				cursorOnOffFlag = 0;
			}else{
				printf("Start Cursor...\r\n");
				LCD_Cursor(On);
				cursorOnOffFlag = 1;
			}
			break;

		case 'm':
			if(cursorOnOffFlag == 1){
				printf("Cursor Move...\r\n");
				LCD_SetCursor(0, 1);
			}else{
				printf("Cursor not on (press c), doing nothing...\r\n");
			}
			break;

		case 'd':
			printf("Cursor Move Right...\r\n");
			LCD_Scroll(Right);
			break;

		case 'l':
			printf("Scroll Left...\r\n");
			LCD_Scroll(Left);
			break;

		case 'r':
			printf("Scroll Right...\r\n");
			LCD_Scroll(Right);
			break;

		case 'h':
			showUsage();
			break;

		case 'W':
			printf("Setting Color White...\r\n");
			LCD_SetColor(White);
			break;
		case 'R':
			printf("Setting Color Red...\r\n");
			LCD_SetColor(Red);
			break;
		case 'G':
			printf("Setting Color Green...\r\n");
			LCD_SetColor(Green);
			break;
		case 'B':
			printf("Setting Color Blue...\r\n");
			LCD_SetColor(Blue);
			break;
		case 'P':
			printf("Setting Color Pink...\r\n");
			LCD_SetRGB(255, 0, 255);
			break;
		case 'O':
			printf("Setting Color Orange...\r\n");
			LCD_SetRGB(255, 87, 51);
			break;
		case 'Y':
			printf("Setting Color Yellow...\r\n");
			LCD_SetRGB(250, 255, 51);
			break;

	}



}


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
	 result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
								  CY_RETARGET_IO_BAUDRATE);

	 /* retarget-io init failed. Stop program execution */
	 if (result != CY_RSLT_SUCCESS)
	 {
		 CY_ASSERT(0);
	 }

	printf("\x1b[2J\x1b[;H");

	printf("Lab 3.2\r\n\n");

    __enable_irq();


////////////  appendix 2 stuff inside of main, comment back in if demo person wants it /////////////////
//	printf("CSE121 Lab 3.2 DFR0554 Library\r\n");
//	LCD_Start();
//	LCD_SetColor(Blue);
//	LCD_Print("Hello CSE121");
//	LCD_SetRGB(52, 152, 219);
//	char buf[16];
//	uint16_t cnt = 0;
//
//    for (;;)
//    {
//        LCD_SetCursor(0, 1);
//        sprintf(buf, "%u", cnt++);
//        LCD_Print(buf);
//        CyDelay(1000);
//    }
//////////////////////////////////////////////////////////


    LCD_Start();
    LCD_SetColor(Blue);
    LCD_Print("CSE121 Lab 3.2");
    showUsage();

    //set initial flags:
    displayOnOffFlag = 1;
    blinkingOnOffFlag = 0;
    cursorOnOffFlag = 0;

    for (;;)
	{
    	getUserInput();
	}


}

/* [] END OF FILE */
