#include <atmel_start.h>

#include "pga460.h"

#define SYNC_BYTE			0x55u
#define MAX_REG_ACC_SIZE	43u
#define FRAME_OVERHEAD		3u

uint8_t gl_data_buff[MAX_REG_ACC_SIZE + FRAME_OVERHEAD] = 
							{
								SYNC_BYTE
							};

uint8_t PGA460_calc_checksum (uint8_t * buff, uint32_t len);
							
//void PGA460_read_register(void)
//{
	//struct io_descriptor *io;
	//uint8_t read_buff[4] = {0};
	//gl_data_buff[1] = 0x09u;
	//gl_data_buff[2] = 0x1Bu;
	//gl_data_buff[3] = PGA460_calc_checksum(&gl_data_buff[1], 2);
	//
	//usart_sync_get_io_descriptor(&USART_0, &io);
	//usart_sync_enable(&USART_0);
//
	//io_write(io, gl_data_buff, 4);
	//io_read(io, read_buff, 3);
	//
//}
//uint8_t PGA460_calc_checksum (uint8_t * buff, uint32_t len)
//{
	//uint16_t checksum = 0;
	//for (uint32_t i =0; i < len; i++)
	//{
		//checksum += buff[i];
		//checksum = ((checksum + (checksum >> 0x08u)) & 0xFFu);
	//}
	//return(0xFFu - (checksum & 0xFFu));
//}
int main(void)
 {
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	//PGA460_read_register();
	PGA460_run();

	/* Replace with your application code */
	while (1) {
	}
}
