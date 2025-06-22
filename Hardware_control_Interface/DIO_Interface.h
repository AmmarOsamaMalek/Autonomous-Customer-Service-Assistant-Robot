/*
 * DIO_Interface.h
 *
 * Created: 3/24/2023 3:56:15 PM
 *  Author: Ammar
 */ 


#ifndef DIO_INTERFACE_H_
#define DIO_INTERFACE_H_

typedef enum{
	PA,
	PB,
	PC,
	PD	
	}Port_type;


typedef enum{
	
	OUTPUT,
	INFREE,
	INPULL
	}Dio_PinStatus_type;

typedef enum{
	PINA0=0,
	PINA1=1,
	PINA2,
	PINA3,
	PINA4,
	PINA5,
	PINA6,
	PINA7,
	PINB0,
	PINB1,
	PINB2,
	PINB3,
	PINB4,
	PINB5,
	PINB6,
	PINB7,
	PINC0,
	PINC1,
	PINC2,
	PINC3,
	PINC4,
	PINC5,
	PINC6,
	PINC7,
	PIND0,
	PIND1,
	PIND2,
	PIND3,
	PIND4,
	PIND5,
	PIND6,
	PIND7,
	TOTAL_PINS
}DIO_Pin_type;


typedef enum{
	LOW = 0,
	HIGH
	}Dio_PinVoltage_type;
void DIO_Init(void);
//void DIO_InitPin2(DIO_Pin_type pin,Dio_PinStatus_type status);

void Dio_WritePin(DIO_Pin_type pin,Dio_PinVoltage_type volt);
//void DIO_WritePin2(DIO_Pin_type pin,Dio_PinVoltage_type volt);

//Dio_PinVoltage_type DIO_ReadPin2(DIO_Pin_type pin);
Dio_PinVoltage_type Dio_ReadPin(DIO_Pin_type pin);

void DIO_TogglePin(DIO_Pin_type pin);

void DIO_WritePort(Port_type port,u8 value);

u8 DIO_ReadPort(Port_type port);

#endif /* DIO_INTERFACE_H_ */