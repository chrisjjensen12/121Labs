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
#include "cy_ble.h"
#include "cy_ble_hal_pvt.h"
#include "cycfg_ble.h"
#include "ble_findme.h"
#define TRIGGER_PORT GPIO_PRT9
#define TRIGGER_NUM 2U
#define ECHO_PORT GPIO_PRT9
#define ECHO_NUM 3U
#define TIMER_HW TCPWM0
#define TIMER_NUM 0UL
#define TIMER_MASK (1UL << 0)
#define CY_BLE_CUSTOM_SERVICE_DISTANCE_CHARACTERISTIC_CHAR_HANDLE   (0x0009u)

cy_stc_ble_conn_handle_t app_conn_handle1;


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

	Cy_GPIO_Pin_Init(TRIGGER_PORT, TRIGGER_NUM, &TRIGGER_config);
	Cy_GPIO_Pin_Init(ECHO_PORT, ECHO_NUM, &ECHO_config);

	Cy_TCPWM_Counter_Init(TIMER_HW, TIMER_NUM, &TIMER_config);
	/* Enable the initialized counter */
	Cy_TCPWM_Counter_Enable(TIMER_HW, TIMER_NUM);

	printf("\x1b[2J\x1b[;H");

	printf("Lab 4.4\r\n\n");

    __enable_irq();

    ble_findme_init();


    for (;;)
    {

    	Cy_BLE_ProcessEvents();

    }
}

/* [] END OF FILE */
