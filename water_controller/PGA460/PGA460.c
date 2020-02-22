
#include "sam.h"
#include "uart_extern.h"
#include "PGA460.h"

#define PGA460_TX_PORT		0u
#define PGA460_TX_PIN		4u
#define PGA460_RX_PORT		0u
#define PGA460_RX_PIN		5u

#define SYNC_BYTE			0x55u
#define MAX_REG_ACC_SIZE	43u
#define FRAME_OVERHEAD		3u

uint8_t gl_data_buff[MAX_REG_ACC_SIZE + FRAME_OVERHEAD] = 
							{
								SYNC_BYTE
							};
							

void PGA460_Init (void)
{
	/* Pin Init.*/
	/* Set TX pin direction to output.*/
	PORT->Group[PGA460_TX_PORT].DIRSET.reg = (0x01u << PGA460_TX_PIN);
	/* Select the Sercom option in the pin mux. */
	PORT->Group[PGA460_TX_PORT].PMUX[PGA460_TX_PIN/2].reg = 0x33u;
	PORT->Group[PGA460_TX_PORT].PINCFG[PGA460_TX_PIN].reg = PORT_PINCFG_PMUXEN;
	
	/* RX Pin Configuration. */
	PORT->Group[PGA460_RX_PORT].DIRCLR.reg = (0x01u << PGA460_RX_PIN);
	PORT->Group[PGA460_RX_PORT].PINCFG[PGA460_RX_PIN].reg = PORT_PINCFG_PMUXEN;
	
	/* Initialize the UART */
	uart_init(0, 115200);

}

void PGA460_read_register(void)
{
	uint8_t read_buff[4];
	gl_data_buff[1] = 0x09u;
	gl_data_buff[2] = 0x1Bu;
	gl_data_buff[3] = PGA460_calc_checksum(&gl_data_buff[1], 2);
	uart_write(0, gl_data_buff, 0x04u);
	uart_read(0, read_buff, sizeof(read_buff));
	
}
uint8_t PGA460_calc_checksum (uint8_t * buff, uint32_t len)
{
	uint16_t checksum = 0;
	for (uint32_t i =0; i < len; i++)
	{
		checksum += buff[i];
		checksum = ((checksum + (checksum >> 0x08u)) & 0xFFu);
	}
	return(0xFFu - (checksum & 0xFFu));
}