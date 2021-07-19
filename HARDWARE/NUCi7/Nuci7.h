#ifndef __NUCI7_H
#define __NUCI7_H

typedef struct{
	float data[3];
} Bikemsg;

typedef struct{
	float phi;
	int speed;
} Command;

extern int itflag;

void Nuci7Write(void);
void Nuci7_callback(void);
void Nuci7Reset(void);
void Bikedrive(void);


#endif
