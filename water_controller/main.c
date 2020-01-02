/*
 * water_controller.c
 *
 * Created: 12/30/2019 2:56:08 PM
 * Author : shnx
 */ 


#include "sam.h"
#include "uart_extern.h"


int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	
	/* Initialize the UART */
	uart_init();

    /* Replace with your application code */
    while (1) 
    {
    }
}
