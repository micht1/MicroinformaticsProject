#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <chprintf.h>

#include <communications.h>

#define COMPUTERMESSAGESIZE 8

static PCMessage_t *computerMessagePointer=NULL;
/*
*	Sends floats numbers to the computer
*/
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size) 
{	
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

void setMessage (PCMessage_t *messageToSend)
{
	if(messageToSend!=NULL)
	{
		computerMessagePointer=messageToSend;
	}
}
void messageReady(bool isReady)
{
	computerMessagePointer->updated=isReady;
	sendEventDataToComputer();
}

uint8_t sendEventDataToComputer(void)
{
	if(computerMessagePointer!=NULL)
	{
		if(computerMessagePointer->updated==true)
		{
			float messageBuffer[COMPUTERMESSAGESIZE]={0};
			messageBuffer[0]=computerMessagePointer->xPosition;
			messageBuffer[1]=computerMessagePointer->yPosition;
			messageBuffer[2]=cos(computerMessagePointer->directionOfSound);
			messageBuffer[3]=sin(computerMessagePointer->directionOfSound);
			messageBuffer[4]=cos(computerMessagePointer->obstacleDirection);
			messageBuffer[5]=sin(computerMessagePointer->obstacleDirection);
			messageBuffer[6]=cos(computerMessagePointer->avoidanceDirection);
			messageBuffer[7]=sin(computerMessagePointer->avoidanceDirection);
			SendFloatToComputer((BaseSequentialStream *) &SD3,messageBuffer,COMPUTERMESSAGESIZE);
			//chprintf((BaseSequentialStream *) &SD3,"X: %f,Y: %f,Sound: %f,Desired: %f,obstacle: %f\n\r",computerMessagePointer->xPosition, computerMessagePointer->yPosition, computerMessagePointer->directionOfSound, computerMessagePointer->avoidanceDirection, computerMessagePointer->obstacleDirection);
			return 1;
		}
	}
	return 0;
}
