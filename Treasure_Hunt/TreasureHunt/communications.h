#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);
typedef struct{
	float xPosition;
	float yPosition;
	float directionOfSound;
	float avoidanceDirection;
	float obstacleDirection;
	bool updated;

}PCMessage_t;

void messageReady(bool isReady);
void setMessage (PCMessage_t *messageToSend);
uint8_t sendEventDataToComputer(void);

#endif /* COMMUNICATIONS_H */
