#ifndef __DELAY_H_
#define __DELAY_H_

#include "board.h"

void delay_init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);


#endif