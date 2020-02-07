/*
* uart.c
*
* For use on a SAM D20 
*
* Created: 2/1/2020 16:17:00
* Author : Shankaranarayana Barre
*
* */

#include "sam.h"
#include "stdbool.h"

#include "uart_vars.h"

// BAUD calculation
#define CLOCKRATE 8000000
#define BAUDRATE 115200
//#define BAUD 65536UL - ((uint64_t)65536 * 16 * BAUDRATE) / CLOCKRATE

#define REC_ERR 2

extern uint8_t sercom3_errors, sercom3_char_received, sercom3_data;

void uart_init(uint8_t sercom_idx, uint32_t baud_rate)
{
	SercomUsart *sercom_usart_ins = SERCOM0 + ( sercom_idx * 0x400U);
    // PORT Setup

	// SERCOM3 PAD[2] PIN_PA24 is TXD, SERCOM3 PAD[3] PIN_PA25 is RXD
	// Set PAD[2] PIN_PA24 TXD to output
	//REG_PORT_DIRSET0 = PORT_PA24;

	// Set PMUXEN bit for both PINCFG24 PIN_PA24 TXD and PINCFG25 PIN_PA25 RXD
	// 24 8 bit registers offset to PINCFG24, pointer is 16 bit to handle int value, (32 - 24 * 2) = 12 16 bit pointer offset
	// Uses 4 instructions, 7 clock cycles, 12 bytes including pointer address
	// #define REG_PORT_PINCFG0           (*(RwReg  *)0x41004440UL) /**< \brief (PORT) Pin Configuration 0 */
	// Faster and shorter than
	// PORT->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
	// PORT->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
	//*((uint16_t *)&REG_PORT_PINCFG0 + 12) = (PORT_PINCFG_PMUXEN) | (PORT_PINCFG_PMUXEN << 8);

	// Set PMUX12 to peripheral function C for PIN_PA24 TXD and PIN_PA25 RXD
	// 12 8 (4+4) bit registers offset to PMUX12
	// Uses 3 instructions, 5 clock cycles, 10 bytes including pointer address
	// Same result as PORT->Group[0].PMUX[12].reg = PORT_PMUX_PMUXO_C | PORT_PMUX_PMUXE_C;
	//*((uint8_t *)&REG_PORT_PMUX0 + 12) = PORT_PMUX_PMUXO_C | PORT_PMUX_PMUXE_C;

	// Clock Setup

	// Set up GCLK - Generic Clock Controller
	// GCLK_CLKCTRL - GEN Generic Clock Generator is GCLKGEN0
	// GCLK_CLKCTRL - ID Generic Clock Selection ID is GCLK_SERCOM3_CORE
	// GCLK_CLKCTRL - CLKEN Clock Enable is enabled
	REG_GCLK_CLKCTRL |= GCLK_CLKCTRL_GEN_GCLK0 | ((GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + sercom_idx) << GCLK_CLKCTRL_ID_Pos) | GCLK_CLKCTRL_CLKEN;
	while(REG_GCLK_STATUS & GCLK_STATUS_SYNCBUSY);

	// Power Management

	// Enable peripheral clock for SERCOM3
	// APBC Mask SERCOM3 SERCOM3 APB Clock Enable is enabled
	REG_PM_APBCMASK |= (0x01U << (PM_APBCMASK_SERCOM0_Pos + sercom_idx));

	// SERCOM3 Setup

	// SERCOM3 USART CTRLA register setup
	// Disables SERCOM3 registers Enable-Protection so we can write to all the SERCOM3 registers
	// SERCOM3_USART_CTRLA - Mode = USART with internal clock
	// SERCOM3_USART_CTRLA - RUNSTDBY = Generic clock is enabled in all sleep modes. Any interrupt can wake up the device.
	// SERCOM3_USART_CTRLA - TXPO assigned to PAD2
	// SERCOM3_USART_CTRLA - RXPO assigned to PAD3
	// SERCOM3_USART_CTRLA - DORD = LSB is transmitted first
	sercom_usart_ins->CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK | SERCOM_USART_CTRLA_RUNSTDBY | SERCOM_USART_CTRLA_TXPO_PAD2
							  | SERCOM_USART_CTRLA_RXPO_PAD3 | SERCOM_USART_CTRLA_DORD;
	while(sercom_usart_ins->STATUS.reg & SERCOM_USART_STATUS_SYNCBUSY);

	// SERCOM3 USART BAUD register setup
	sercom_usart_ins->BAUD.reg = 65536UL - ((uint64_t)65536 * 16 * baud_rate) / SystemCoreClock;

	// SERCOM3 USART CTRLB register setup
	// TXEN is enabled
	// RXEN is enabled
	// DORD = LSB is transmitted first
	sercom_usart_ins->CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
	while(sercom_usart_ins->STATUS.reg & SERCOM_USART_STATUS_SYNCBUSY);

	// Enable SERCOM3 interrupts
	// RXC Receive Complete Interrupt Enabled
	//sercom_usart_ins->INTENSET.reg = SERCOM_USART_INTENSET_RXC;

	// Enable SERCOM3 NVIC Interrupt
	//NVIC_EnableIRQ(SERCOM3_IRQn);

	// Enable SERCOM3
	sercom_usart_ins->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	while(sercom_usart_ins->STATUS.reg & SERCOM_USART_STATUS_SYNCBUSY);
	
}

// Using Interrupt for receive only, Sercom3putchar for transmit
void SERCOM3_Handler()
{
	if(REG_SERCOM3_USART_INTFLAG & SERCOM_USART_INTFLAG_RXC)	//character received?
	{
		//sercom3_char_received = true;
		//sercom3_data = REG_SERCOM3_USART_DATA;	//fetch received character

		if(REG_SERCOM3_USART_STATUS & (SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF))	//receive errors happened?
		{
			REG_SERCOM3_USART_STATUS = SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF;	//reset errors (framing and buffer overflow error)
			//sercom3_errors |= REC_ERR;	//set global rec error flag in my global error variable not to use the char
		}

	}

	if (REG_SERCOM3_USART_INTFLAG & SERCOM_USART_INTFLAG_TXC)
	{
		while ((REG_SERCOM3_USART_INTFLAG & SERCOM_USART_INTFLAG_DRE) == 0);
		REG_SERCOM3_USART_INTFLAG |= SERCOM_USART_INTFLAG_TXC;
	}
}

void uart_putchar(uint8_t sercom_idx, char ch)
{
	SercomUsart *sercom_usart_ins = SERCOM0 + ( sercom_idx * 0x400U);
	while ((sercom_usart_ins->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == 0);
	sercom_usart_ins->DATA.reg = ch;
}

char uart_getchar(uint8_t sercom_idx)
{
	SercomUsart *sercom_usart_ins = SERCOM0 + ( sercom_idx * 0x400U);
	while ((sercom_usart_ins->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) == 0);
	return sercom_usart_ins->DATA.reg;
}

