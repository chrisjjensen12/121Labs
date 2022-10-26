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
#define BUTTON_PORT GPIO_PRT0
#define BUTTON_PORT_NUM 0U
#define BUTTON_NUM 4U
#define ADC_HW SAR
void button_on_handler(void);
void updateRGBvalues(int16_t result);
void getPotentiometerValues(void);
int16_t fixADCresult(int16_t ADCresult);

int RGBflag;
char Rbuf[16];
char Gbuf[16];
char Bbuf[16];
uint16_t red;
uint16_t green;
uint16_t blue;


const cy_stc_sysint_t intrCfg =
{
	.intrSrc = ioss_interrupts_gpio_0_IRQn,
	.intrPriority = 0UL
};

void button_on_handler(void)
{
	if(RGBflag == 1){
		printf("Switching to GREEN\r\n");
		RGBflag = 2;
	}else if(RGBflag == 2){
		printf("Switching to BLUE\r\n");
		RGBflag = 3;
	}else if(RGBflag == 3){
		printf("Switching to RED\r\n");
		RGBflag = 1;
	}

	Cy_GPIO_ClearInterrupt(BUTTON_PORT, BUTTON_NUM);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);

}

void getPotentiometerValues(void){

	uint16_t chan = 0UL;
	int16_t ADCresult;
	int16_t result;

	CyDelay(100);
	ADCresult = Cy_SAR_GetResult16(SAR, chan);
	result = fixADCresult(ADCresult);
//	printf("%d\r\n", result);

	//update the RGB values
	updateRGBvalues(result);
}

int16_t fixADCresult(int16_t ADCresult){
	int16_t result = 0;
	ADCresult = ADCresult/8; //set divider so that max pot value is 255


	if(ADCresult > 255){
		result = 255;
	}else if(ADCresult < 0){
		result = 0;
	}else{
		result = ADCresult;
	}

	return result;
}


void updateRGBvalues(int16_t result){

	//convert int to string
	char mystr[100];

	if(result < 100){
		sprintf(mystr, "0%u", result);
	}else{
		sprintf(mystr, "%u", result);
	}

	if(RGBflag == 1){
		LCD_SetCursor(0, 1);
		LCD_Print(mystr);
		red = result;
		printf("Setting RGB to (%d, %d, %d)\r\n", red, green, blue);
		LCD_SetRGB(red, green, blue);
	}else if(RGBflag == 2){
		LCD_SetCursor(5, 1);
		LCD_Print(mystr);
		green = result;
		printf("Setting RGB to (%d, %d, %d)\r\n", red, green, blue);
		LCD_SetRGB(red, green, blue);
	}else if(RGBflag == 3){
		LCD_SetCursor(12, 1);
		LCD_Print(mystr);
		blue = result;
		printf("Setting RGB to (%d, %d, %d)\r\n", red, green, blue);
		LCD_SetRGB(red, green, blue);
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

	printf("Lab 3.3\r\n\n");

    __enable_irq();


    //initialize ADC
    Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);

    Cy_SysAnalog_Enable();

    cy_en_sar_status_t SARstatus;
	SARstatus = Cy_SAR_Init(SAR, &ADC_config);
	if (CY_SAR_SUCCESS == SARstatus)
	{
		/* Turn on the SAR hardware. */
		Cy_SAR_Enable(SAR);
		/* Begin continuous conversions. */
		Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
	}

    /* Initialize the interrupt with vector at Interrupt_Handler*/
	Cy_SysInt_Init(&intrCfg, &button_on_handler);

	Cy_GPIO_SetInterruptMask(BUTTON_PORT, BUTTON_PORT_NUM, 1UL);
	//clear pending IRQ
	NVIC_ClearPendingIRQ(intrCfg.intrSrc);
	/* Enable the interrupt */
	NVIC_EnableIRQ(intrCfg.intrSrc);


    //init LCD
    LCD_Start();

    //init RGBflag to 1
    RGBflag = 1;

    LCD_Print("RED  GREEN  BLUE");

    //init all RGB to 000
    red = 0;
	green = 0;
	blue = 0;

	LCD_SetCursor(0, 1);
	LCD_Print("000");

	LCD_SetCursor(5, 1);
	LCD_Print("000");

	LCD_SetCursor(12, 1);
	LCD_Print("000");

    for (;;)
	{
    	getPotentiometerValues();
	}


}

/* [] END OF FILE */
