#ifndef _ENCODER_H
#define _ENCODER_H
#include "sys.h"

void Encoder_Init(void);
void Encoder_TIM_Init(u32 arr,u32 psc);
u32 read_encoder(void);

#endif
