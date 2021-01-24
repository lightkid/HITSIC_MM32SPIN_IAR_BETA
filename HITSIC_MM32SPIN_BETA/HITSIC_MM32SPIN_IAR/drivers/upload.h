#ifndef __UPLOAD_H_
#define __UPLOAD_H_

#include "board.h"

#define BYTE0(Temp)       (*(char *)(&Temp))     
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))

void Send_Begin(void);
void Send_Variable(void);

#endif