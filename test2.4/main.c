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


#define tcpwm_0_cnt_5_HW TCPWM0
#define tcpwm_0_cnt_5_NUM 5UL
#define tcpwm_0_cnt_5_MASK (1UL << 5)
#define ADC_HW SAR
void setPeriod(int newPeriod);

void getUserInput(){

	 char c;

	 //get current period value
	 uint32_t period = Cy_TCPWM_PWM_GetPeriod0(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM);

	 setvbuf(stdin, NULL, _IONBF, 0);

	 c = getchar();

	 if(c == 'u'){
		 setPeriod(period+50);
		 printf("Increasing brightness. Period: %ld\r\n", Cy_TCPWM_PWM_GetPeriod0(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM));
	 }
	 if(c == 'd'){
		 setPeriod(period-50);
		 printf("Decreasing brightness. Period: %ld\r\n", Cy_TCPWM_PWM_GetPeriod0(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM));
	 }

}

void setPeriod(int newPeriod){

	Cy_TCPWM_TriggerStopOrKill_Single(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM);
	while ( Cy_TCPWM_PWM_GetStatus(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM) & CY_TCPWM_PWM_STATUS_COUNTER_RUNNING );
	Cy_TCPWM_PWM_SetCounter(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM, 0);
	Cy_TCPWM_PWM_SetPeriod0(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM, newPeriod);
	Cy_TCPWM_TriggerReloadOrIndex_Single(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM);

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

    __enable_irq();

    Cy_TCPWM_PWM_Init(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM, &tcpwm_0_cnt_5_config);
    Cy_TCPWM_PWM_Enable(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM);
    Cy_TCPWM_TriggerStart(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_MASK);

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

	printf("\x1b[2J\x1b[;H");

	printf("Lab 2.2 test\r\n\n");

	setPeriod(400);

	uint16_t chan = 0UL;
	int16_t ADCresult;

	for (;;)
	{
		CyDelay(10);
//		getUserInput();
		ADCresult = Cy_SAR_GetResult16(SAR, chan);
		setPeriod(ADCresult);
//		printf("%d\r\n", ADCresult);
	}

}

/* [] END OF FILE */

