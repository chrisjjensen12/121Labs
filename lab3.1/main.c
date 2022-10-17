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
#define I2C_HW SCB6
void scanForI2CAdresses();
/* Allocate context for I2C operation */
cy_stc_scb_i2c_context_t i2cContext;


//Credit to "https://iotexpert.com/i2c-detect-with-psoc-6/" for some aspects of scanning function
void scanForI2CAdresses()
{
	printf("\nScanning I2C 7-Bit Addresses…\r\n");
	uint32_t rval;

	// Iterate through the address starting at 0x00
	for(uint32_t address=0; address<0x7F; address++)
	{
		rval = Cy_SCB_I2C_MasterSendStart(I2C_HW, address, CY_SCB_I2C_WRITE_XFER, 10, &i2cContext);
		if(rval == CY_SCB_I2C_SUCCESS)
		{
			printf("Found: 0x%02X\r\n", (unsigned int)address);
		}
		Cy_SCB_I2C_MasterSendStop(I2C_HW, 0, &i2cContext);
	}

	printf("Scan complete\r\n\n");

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

	printf("Lab 3.1\r\n\n");

    __enable_irq();

    //initialize I2C
    Cy_SCB_I2C_Init(I2C_HW, &I2C_config, &i2cContext);
    Cy_SCB_I2C_Enable(I2C_HW);

    //turn off stdin buffer
    setvbuf(stdin, NULL, _IONBF, 0);

    for (;;)
    {
    	printf("Input s to search I2C bus\r\n");
    	char c = getchar();

    	if(c == 's'){
    		scanForI2CAdresses();
    	}

    }
}

/* [] END OF FILE */
