/*
 * water_controller.c
 *
 * Created: 12/30/2019 2:56:08 PM
 * Author : shankaranarayana barre
 */ 


#include "sam.h"
#include "uart_extern.h"
#include "PGA460.h"


int main(void)
{
	volatile char var;
    /* Initialize the SAM system */
    SystemInit();
	
	/* Initialize the PGA460 */
	PGA460_Init();

	/* UART test code. */
	uart_putchar(0,0xAA);
	var = uart_getchar(0);
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

