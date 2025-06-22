

#ifndef LCD_CFG_H_
#define LCD_CFG_H_

#define _4_BIT   0
#define _8_BIT   1

/* _4_BIT OR _8_BIT*/
#define  LCD_MODE  _4_BIT



#define  RS   PINA1
#define  EN   PINA2

#define  D7   PINA6
#define  D6   PINA5
#define  D5   PINA4
#define  D4   PINA3

#define LCD_PORT  PD
#endif /* LCD_CFG_H_ */