/*
 * DIO_Private.h
 *
 * Created: 3/24/2023 3:55:25 PM
 *  Author: Ammar
 */ 


#ifndef DIO_PRIVATE_H_
#define DIO_PRIVATE_H_


static void Dio_InitPin(DIO_Pin_type pin,Dio_PinStatus_type status);

extern const Dio_PinStatus_type  PinsStatusArray[TOTAL_PINS];


#endif /* DIO_PRIVATE_H_ */