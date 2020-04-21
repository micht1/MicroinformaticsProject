#ifndef IRSENSOREADING_H
#define IRSENSOREADING_H
#include "msgbus/messagebus.h"

typedef struct {
	bool obstaclePresent;
	bool ignoringObstacle;
	int16_t directionOfObstacle;	//direcetion in degrees

}IRData;

extern messagebus_t bus;

void IRProcessingStart(void);
bool isObstaclePresent(void);

void ignoreObstacle(bool doIgnoreObstacle);

float getObjectDirection(void);

#endif

