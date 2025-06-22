
#ifndef EXI_H_
#define EXI_H_

#include "StdTypes.h"
#include "MemMap.h"
#include "Utlis.h"

typedef enum{
	LOW_LEVEL=0,
	ANY_LOGIC_CHANGE,
	FALLING_EDGE,
	RISING_EDGE,
}TriggerEdge_type;

typedef enum{
	EX_INT0=0,
	EX_INT1,
	EX_INT2
}ExInterruptSource_type;

void EXI_Init(void);


void EXI_Enable(ExInterruptSource_type Interrupt);
void EXI_Disable(ExInterruptSource_type Interrupt);
void EXI_TriggerEdge(ExInterruptSource_type Interrupt,TriggerEdge_type Edge);

void EXI_SetCallBack(ExInterruptSource_type Interrupt,void(*LocalPtr)(void));




#endif /* EXI_H_ */