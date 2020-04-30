#include <main.h>
#include <math.h>
#include <driveMotors.h>

static int32_t xPosition=0;
static int32_t yPosition=0;
static float desiredBearingPhi=0;
static float bearingPhi=0;



void setDesiredBearing(float desiredBearing)
{
	desiredBearingPhi=desiredBearing;
}



float getXPosition(void)
{
	return xPosition;
}

float getYPosition(void)
{
	return yPosition;
}


float getBearing(void)
{
	return bearingPhi;
}
