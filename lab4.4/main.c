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
#define BUTTON_PORT GPIO_PRT0
#define BUTTON_NUM 4U
void button_on_handler(void);
#define CY_BLE_CUSTOM_SERVICE_DISTANCE_CHARACTERISTIC_CHAR_HANDLE   (0x0009u)

cy_stc_ble_conn_handle_t app_conn_handle1;

const cy_stc_sysint_t intrCfg =
{
	.intrSrc = ioss_interrupts_gpio_0_IRQn,
	.intrPriority = 0UL
};

//cy_stc_ble_gatt_handle_value_pair_t val_pair = {
//    .value = {NULL, 0, 0},
//    .attrHandle = CY_BLE_CUSTOM_SERVICE_DISTANCE_CHARACTERISTIC_CHAR_HANDLE
//};


void button_on_handler(void)
{
	printf("button handler!\r\n");


	if(CY_BLE_CONN_STATE_CONNECTED == Cy_BLE_GetConnectionState(app_conn_handle1)){
		printf("connected to device\r\n");
	}else{
		printf("not connected to device\r\n");
	}

    /*Structure used to load values to GATT database*/
    cy_stc_ble_gatt_handle_value_pair_t	handleValuePair;

    handleValuePair.attrHandle = CY_BLE_CUSTOM_SERVICE_DISTANCE_CHARACTERISTIC_CHAR_HANDLE;
    handleValuePair.value.val = (uint8*)15;
    handleValuePair.value.len = sizeof(uint8_t);
    Cy_BLE_GATTS_WriteAttributeValueLocal(&handleValuePair);

	Cy_GPIO_ClearInterrupt(BUTTON_PORT, BUTTON_NUM);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);
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

	/* Initialize the interrupt with vector at Interrupt_Handler*/
	Cy_SysInt_Init(&intrCfg, &button_on_handler);

	Cy_GPIO_SetInterruptMask(BUTTON_PORT, BUTTON_PORT_NUM, 1UL);
	//clear pending IRQ
	NVIC_ClearPendingIRQ(intrCfg.intrSrc);
	/* Enable the interrupt */
	NVIC_EnableIRQ(intrCfg.intrSrc);


	printf("\x1b[2J\x1b[;H");

	printf("Lab 4.4\r\n\n");

    __enable_irq();

    ble_findme_init();
    /*Structure used to load values to GATT database*/
    cy_stc_ble_gatt_handle_value_pair_t	handleValuePair;

    uint8_t var = 67;

    handleValuePair.attrHandle = CY_BLE_CUSTOM_SERVICE_DISTANCE_CHARACTERISTIC_CHAR_HANDLE;
    handleValuePair.value.len = sizeof(uint8_t);


    for (;;)
    {

    	Cy_BLE_ProcessEvents();

    	handleValuePair.value.val = (uint8_t*)&var;

    	Cy_BLE_GATTS_WriteAttributeValueLocal(&handleValuePair);

    }
}

/* [] END OF FILE */
