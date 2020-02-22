#ifndef _UART_EXTERN_H_
#define _UART_EXTERN_H_

void uart_init(uint8_t sercom_idx, uint32_t baud_rate);
void uart_putchar(uint8_t sercom_idx, char ch);
char uart_getchar(uint8_t sercom_idx);
void uart_write(uint8_t sercom_idx, uint8_t * buff, uint32_t len);
void uart_read(uint8_t sercom_idx, uint8_t * buff, uint32_t len);


#endif /* _UART_EXTERN_H_ */
