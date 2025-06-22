
#include "StdTypes.h"
#include "MemMap.h"
#include "Utlis.h"
#include "DIO_Interface.h"
#include "DIO_Private.h"



void DIO_Init(void)
{
	DIO_Pin_type i;
	for (i=PINA0;i<TOTAL_PINS;i++)
	{
		Dio_InitPin(i,PinsStatusArray[i]);
	}
}




static void Dio_InitPin(DIO_Pin_type pin,Dio_PinStatus_type status)
{
	
	Port_type Port =pin/8;
	u8 pin_num = pin%8;
	switch(status)
	{
		case OUTPUT:
		switch(Port)
	{	
		case PA:
		SET_BIT(DDRA,pin_num);
		CLEAR_BIT(PORTA,pin_num);
		break;
		case PB:
		SET_BIT(DDRB,pin_num);
		CLEAR_BIT(PORTB,pin_num);
		break;
		case PC:
		SET_BIT(DDRC,pin_num);
		CLEAR_BIT(PORTC,pin_num);
		break;
		case PD:
		SET_BIT(DDRD,pin_num);
		CLEAR_BIT(PORTD,pin_num);
		break;
	}
	break;
		case INFREE:
		switch(Port)
	{	
		case PA:
		CLEAR_BIT(DDRA,pin_num);
		CLEAR_BIT(PORTA,pin_num);
		break;
		case PB:
		CLEAR_BIT(DDRB,pin_num);
		CLEAR_BIT(PORTB,pin_num);
		break;
		case PC:
		CLEAR_BIT(DDRC,pin_num);
		CLEAR_BIT(PORTC,pin_num);
		break;
		case PD:
		CLEAR_BIT(DDRD,pin_num);
		CLEAR_BIT(PORTD,pin_num);
		break;
	}	
	break;
		case INPULL:
        switch(Port)
   {
	         case PA:
	         CLEAR_BIT(DDRA,pin_num);
	         SET_BIT(PORTA,pin_num);
	         break;
	         case PB:
	         CLEAR_BIT(DDRB,pin_num);
	         SET_BIT(PORTB,pin_num);
	         break;
	         case PC:
	         CLEAR_BIT(DDRC,pin_num);
	         SET_BIT(PORTC,pin_num);
	         break;
	         case PD:
	         CLEAR_BIT(DDRD,pin_num);
	         SET_BIT(PORTD,pin_num);
	         break;
    }		
	}
}

void Dio_WritePin(DIO_Pin_type pin,Dio_PinVoltage_type volt)
{
	Port_type port =pin/8;
	u8 pin_num = pin%8;
	if(volt == HIGH)
	{
	 switch(port)
	 {
		 case PA:
		 SET_BIT(PORTA,pin_num);
		 break;
		 case PB:
		 SET_BIT(PORTB,pin_num);
		 break;
		 case PC:
		 SET_BIT(PORTC,pin_num);
		 break;
		 case PD:
		 SET_BIT(PORTD,pin_num);
		 break;
	 }	
	}
	else if(volt == LOW)
	{
		switch(port)
		{
			case PA:
			CLEAR_BIT(PORTA,pin_num);
			break;
			case PB:
			CLEAR_BIT(PORTB,pin_num);
			break;
			case PC:
			CLEAR_BIT(PORTC,pin_num);
			break;
			case PD:
			CLEAR_BIT(PORTD,pin_num);
			break;
		}
	}
}

Dio_PinVoltage_type Dio_ReadPin(DIO_Pin_type pin)
{
	Port_type port =pin/8;
	u8 pin_num = pin%8;
	Dio_PinVoltage_type volt = 0;
	switch(port)
	{
		case PA:
	volt = READ_BIT(PINA,pin_num);
		break;
		case PB:
	volt = READ_BIT(PINB,pin_num);
		break;
		case PC:
	volt = READ_BIT(PINC,pin_num);
		break;
		case PD:
	volt = READ_BIT(PIND,pin_num);
		break;
	}
	
	return volt;
}
/*
Error_t Dio_ReadPin(Port_type port,u8 pin_num,Dio_PinVoltage_type*Pvolt)
{
	if(Pvolt == NULLPTR)
		return NULL_PTR_ERROR;
		
	else if((port>PD)||(pin_num>7))
	return OUT_OF_RANGE;
	
	else
	{
	switch(Port)
	{
		case PA:
		*Pvolt = READ_BIT(PINA,pin_num);
		break;
		case PB:
		*Pvolt = READ_BIT(PINB,pin_num);
		break;
		case PC:
		*Pvolt = READ_BIT(PINC,pin_num);
		break;
		case PD:
		*Pvolt = READ_BIT(PIND,pin_num);
		break;
	}
	return OK;	
	}
}


void DIO_InitPin2(DIO_Pin_type pin_num,Dio_PinStatus_type status)
{
	switch(status)
	{
		case OUTPUT:
	if(pin_num<8)
	{
			SET_BIT(DDRA,pin_num);
			CLEAR_BIT(PORTA,pin_num);
			return;
	}
	else if(pin_num<16)
	 {
			SET_BIT(DDRB,pin_num-8);
			CLEAR_BIT(PORTB,pin_num-8);
			return;
    }
	else if(pin_num<24)
	{
			SET_BIT(DDRC,pin_num-16);
			CLEAR_BIT(PORTC,pin_num-16);
			return;
	}
	else if(pin_num<32)
	{
			SET_BIT(DDRD,pin_num-24);
			CLEAR_BIT(PORTD,pin_num-24);
			return;
	}
		
		break;
		case INFREE:
		if(pin_num<8)
		{
			CLEAR_BIT(DDRA,pin_num);
			CLEAR_BIT(PORTA,pin_num);
			return;
		}
		else if(pin_num<16)
		{
			CLEAR_BIT(DDRB,pin_num-8);
			CLEAR_BIT(PORTB,pin_num-8);
			return;
		}
		else if(pin_num<24)
		{
			CLEAR_BIT(DDRC,pin_num-16);
			CLEAR_BIT(PORTC,pin_num-16);
			return;
		}
		else if(pin_num<32)
		{
			CLEAR_BIT(DDRD,pin_num-24);
			CLEAR_BIT(PORTD,pin_num-24);
			return;
		}

		break;
		case INPULL:
		if(pin_num<8)
		{
			CLEAR_BIT(DDRA,pin_num);
			SET_BIT(PORTA,pin_num);
			return;
		}
		else if(pin_num<16)
		{
			CLEAR_BIT(DDRB,pin_num-8);
			SET_BIT(PORTB,pin_num-8);
			return;
		}
		else if(pin_num<24)
		{
			CLEAR_BIT(DDRC,pin_num-16);
			SET_BIT(PORTC,pin_num-16);
			return;
		}
		else if(pin_num<32)
		{
			CLEAR_BIT(DDRD,pin_num-24);
			SET_BIT(PORTD,pin_num-24);
			return;
		}
	}
}
	
void DIO_WritePin2(DIO_Pin_type pin_num,Dio_PinVoltage_type volt)
{
	if(volt == HIGH)
	{
		if(pin_num<8)
		{
			SET_BIT(PORTA,pin_num);
			return;
		}
		else if(pin_num<16)
		{
			SET_BIT(PORTB,pin_num-8);
			return;
		}
		else if(pin_num<24)
		{
			SET_BIT(PORTC,pin_num-16);
			return;
		}
		else if(pin_num<32)
		{
			SET_BIT(PORTD,pin_num-24);
			return;
		}
	}
	else if(volt == LOW)
	{
		if(pin_num<8)
		{
			CLEAR_BIT(PORTA,pin_num);
			return;
		}
		else if(pin_num<16)
		{
			CLEAR_BIT(PORTB,pin_num-8);
			return;
		}
		else if(pin_num<24)
		{
			CLEAR_BIT(PORTC,pin_num-16);
			return;
		}
		else if(pin_num<32)
		{
			CLEAR_BIT(PORTD,pin_num-24);
			return;
		}
	}
	
}

Dio_PinVoltage_type DIO_ReadPin2(DIO_Pin_type pin_num)
{
	Dio_PinVoltage_type volt = 0;
	
	if(pin_num<8)
			volt = READ_BIT(PINA,pin_num);
			
	else if(pin_num<16)
		   volt = READ_BIT(PINB,pin_num-8);
		   
	else if(pin_num<24)
		   volt = READ_BIT(PINC,pin_num-16);
		   
    else if(pin_num<32)
		   volt = READ_BIT(PIND,pin_num-24);
		   
	return volt;
}
*/
void DIO_TogglePin(DIO_Pin_type pin_num)
{
	if(pin_num<8)
	TOGGLE_BIT(PORTA,pin_num);
	
	else if(pin_num<16)
	TOGGLE_BIT(PORTB,(pin_num)-8);
	
	else if(pin_num<24)
	TOGGLE_BIT(PORTC,(pin_num)-16);
	
	else if(pin_num<32)
	TOGGLE_BIT(PORTD,pin_num-24);
}


void DIO_WritePort(Port_type port,u8 value)
{
	switch(port)
	{
		case PA:
		PORTA = value;
		break;
		case PB:
		PORTB = value;
		break;
		case PC:
		PORTC = value;
		break;
		case PD:
		PORTD = value;
		break;
	}
}

u8 DIO_ReadPort(Port_type port)
{
	u8 value = 0;
	switch(port)
	{
		case PA:
		value = PORTA;
		break;
		case PB:
		value = PORTB;
		break;
		case PC:
		value = PORTC;
		break;
		case PD:
	    value = PORTD;
		break;
	}
	return value;
}