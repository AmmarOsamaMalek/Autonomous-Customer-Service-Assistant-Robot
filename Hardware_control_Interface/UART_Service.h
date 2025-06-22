


#ifndef UART_SERVICE_H_
#define UART_SERVICE_H_


void UART_SendString(u8*str);


void UART_ReceiveString(u8*str);


void UART_SendNumber(u32 num);


u32 UART_ReceiveNumber(void);


u8 endian_check(void);


u32 endian_convert32(u32 num);


u16 endian_convert16(u16 num);


void UART_SendStringCheckSum(u8*str);


u8 UART_ReceiveStringCheckSum(u8*str);


void UART_SendStringAsynch(u8*str);
void UART_ReceiveStringAsynch(u8*str);


void UART_BufferInit(void);
u8 UART_BufferGetter(u8*str);

#endif /* UART_SERVICE_H_ */