#ifndef IRSENSOREADING_H
#define IRSENSOREADING_H
#include "msgbus/messagebus.h"

typedef struct {
	bool obstaclePresent;
	bool ignoringObstacle;
	int16_t directionOfObstacle;

}IRData;

extern messagebus_t bus;

void IRProcessingStart(void);
bool isObstaclePresent(void);
int16_t getObstacleDirection(void);
void ignoreObstacle(bool doIgnoreObstacle);

#endif

