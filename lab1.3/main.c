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
#include "FreeRTOS.h"
#include "task.h"

#define RED_PORT GPIO_PRT6
#define RED_NUM 3U
#define GREEN_PORT GPIO_PRT7
#define GREEN_NUM 1U


void red_task( void * pvParameters )
{

	int pinState = 0;

    for( ;; )
    {
    	Cy_GPIO_Write(RED_PORT, RED_NUM, pinState); // x = 0 for on, 1 for off
    	CyDelay(1000);
    	pinState = 1 - pinState;
    }


}

void green_task( void * pvParameters )
{

	int pinState = 0;

    for( ;; )
    {
    	Cy_GPIO_Write(GREEN_PORT, GREEN_NUM, pinState); // x = 0 for on, 1 for off
    	CyDelay(200);
    	pinState = 1 - pinState;
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

    __enable_irq();

    xTaskCreate(red_task, "RED_TASK", 400, NULL, 2, 0);

    xTaskCreate(green_task, "GREEN_TASK", 400, NULL, 2, 0);

    vTaskStartScheduler();


    for (;;)
    {


    }
}

/* [] END OF FILE */
