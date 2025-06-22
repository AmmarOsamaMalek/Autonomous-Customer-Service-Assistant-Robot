/*
 * Utlis.h
 *
 * Created: 3/24/2023 10:46:20 AM
 *  Author: Ammar
 */ 


#ifndef UTLIS_H_
#define UTLIS_H_


#define WRITE_BIT(x,y,z)        x = ((x & ~(1<<y)) | (z<<y))
#define READ_BIT(reg,bit)       ((reg>>bit)&1)
#define SET_BIT(reg,bit)        reg = (reg|(1<<bit))
#define CLEAR_BIT(reg,bit)      reg = (reg&~(1<<bit))
#define TOGGLE_BIT(reg,bit)     reg = (reg^(1<<bit))


#endif /* UTLIS_H_ */