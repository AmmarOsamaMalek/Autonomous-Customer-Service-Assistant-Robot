

#include "StdTypes.h"
#include "Utlis.h"
#include "DIO_Interface.h"
#define  F_CPU  8000000
#include <util/delay.h>
#include "LCD_Interface.h"
#include "LCD_Cfg.h"

#if LCD_MODE==_8_BIT
static void WriteIns(u8 Ins)
{
	Dio_WritePin(RS,LOW);
	DIO_WritePort(LCD_PORT,Ins);
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
}

static void WriteData(u8 data)
{
	Dio_WritePin(RS,HIGH);
	DIO_WritePort(LCD_PORT,data);
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
}

void LCD_Init(void)
{
	_delay_ms(50);
	WriteIns(0x38);
	WriteIns(0x0c);
	WriteIns(0x01);
	_delay_ms(1);
	WriteIns(0x06);
}
#elif LCD_MODE==_4_BIT
static void WriteIns(u8 Ins)
{
	Dio_WritePin(RS,LOW);
	Dio_WritePin(D7,READ_BIT(Ins,7));
	Dio_WritePin(D6,READ_BIT(Ins,6));
	Dio_WritePin(D5,READ_BIT(Ins,5));
	Dio_WritePin(D4,READ_BIT(Ins,4));
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
	
	Dio_WritePin(D7,READ_BIT(Ins,3));
	Dio_WritePin(D6,READ_BIT(Ins,2));
	Dio_WritePin(D5,READ_BIT(Ins,1));
	Dio_WritePin(D4,READ_BIT(Ins,0));
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
}

static void WriteData(u8 data)
{
	
	Dio_WritePin(RS,HIGH);
	Dio_WritePin(D7,READ_BIT(data,7));
	Dio_WritePin(D6,READ_BIT(data,6));
	Dio_WritePin(D5,READ_BIT(data,5));
	Dio_WritePin(D4,READ_BIT(data,4));
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
	
	Dio_WritePin(D7,READ_BIT(data,3));
	Dio_WritePin(D6,READ_BIT(data,2));
	Dio_WritePin(D5,READ_BIT(data,1));
	Dio_WritePin(D4,READ_BIT(data,0));
	Dio_WritePin(EN,HIGH);
	_delay_ms(1);
	Dio_WritePin(EN,LOW);
	_delay_ms(1);
}

void LCD_Init(void)
{
	_delay_ms(50);
	WriteIns(0x02);
	WriteIns(0x28);
	WriteIns(0x0c);
	WriteIns(0x01);
	_delay_ms(1);
	WriteIns(0x06);
}

#endif
void LCD_WriteChar(u8 ch)
{
	WriteData(ch);
}


void LCD_WriteString(u8*str)
{
	u8 i;
	for(i = 0;str[i];i++)
	{
		LCD_WriteChar(str[i]);
	}
}


void LCD_SetCursor(u8 line,u8 cell)
{
	
	if(line == 0)
	{
		WriteIns(0x80|cell);
	}
	else if(line == 1)
	{
		WriteIns(0x80|0x40|cell);
	}
	
}

void LCD_Clear(void)
{
	WriteIns(0x01);
	_delay_ms(1);
}

void LCD_WriteNumber(s32 num)
 	{
	 	u8 str[10],i=0,j;
	 	if (num==0)
	 	{
		 	LCD_WriteChar('0');
		 	return;
	 	}
	 	if (num<0)
	 	{
		 	num=num*(-1);
		 	LCD_WriteChar('-');
	 	}
	 	
	 	while(num)
	 	{
		 	str[i]=num%10 +'0';
		 	i++;
		 	num=num/10;
	 	}
	 	for (j=i;j>0;j--)
	 	{
		 	LCD_WriteChar(str[j-1]);
	 	}

 	}
void LCD_WriteBinary(u8 num)
 {
	 s8 i;
	 for (i=7;i>=0;i--)
	 {
		 LCD_WriteChar(READ_BIT(num,i)+'0');
	 }
	 
 }


void LCD_ClearLoc(u8 line ,u8 cell,u8 num)
 {
	 u8 i;
	 LCD_SetCursor(line,cell);
	 for (i=0;i<num;i++)
	 {
		 LCD_WriteChar(' ');
	 }
	 
 }
 
 void LCD_WriteNumber_3D(u16 num)
 {
	 //LCD_WriteChar(((num%100000)/10000)+'0');
	 LCD_WriteChar(((num%10000)/1000)+'0');
	 LCD_WriteChar(((num%1000)/100)+'0');
	 LCD_WriteChar(((num%100)/10)+'0');
	 LCD_WriteChar(((num%10)/1)+'0');
 }
 
 
 void LCD_CustomChar(u8 loc,u8*pattern)
 {
	 WriteIns(0x40+loc*8);
	 for(u8 i=0;i<8;i++)
	 {
		 WriteData(pattern[i]);
	 }
	 WriteIns(0x80);
 }
 
 
 void LCD_WriteHex(u8 num)
 {
	 u8 high,low,i,hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	 for(i = 0;i<4;i++)
	 {
		 high = (high*2) + READ_BIT(num,7-i);
		 low  = (low*2) + READ_BIT(num,3-i);
	 }
	 LCD_WriteChar(hex[high]);
	 LCD_WriteChar(hex[low]);
 }