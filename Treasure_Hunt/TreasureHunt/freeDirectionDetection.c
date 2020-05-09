#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "math.h"
#include "driveMotors.h"
#include <arm_math.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "freeDirectionDetection.h"


static bool doScan=false;				//variable which starts the scanning
static scanningStep_t scanStatus=0;		//variable which tracks the scanning status
static float startingBearing=0;			//variable in which the starting point of the scan is saved
static float endingBearing=0;			//variable in which the end point of the scan is saved

//a bearing which is situated between start and endpoint, it is nessecary,
//because if the scanning range is bigger than pi the robot would rotate the wrong way if only the end bearing is used
static float intermediaryBearing=0;
static float bearingFC=0;				//internal variables used to save the bearing where the distance changes from far to close
static float bearingCF=0;				//internal variables used to save the bearing where the distance changes from close to far



static THD_WORKING_AREA(freeDirection_thd_was, 512);
static THD_FUNCTION(freeDirection_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	uint16_t currentDistance=0;
	uint16_t oldDistance=0;
	int16_t minDelta=0;
	float minBearing=0;
	int16_t maxDelta=0;
	float maxBearing=0;
	uint8_t halfOfScan=0;
	while(1)
	{
		switch(scanStatus)
		{
		case IDLE:
			if(doScan==true)	// start scan, reset everything to starting values
			{
				setDesiredBearing(startingBearing);
				scanStatus=ROTATINGTOSTART;
				maxDelta=MINDETECTIONDELTA;
				minDelta=-MINDETECTIONDELTA;
				bearingCF=0;
				bearingFC=0;
				oldDistance=0;
				currentDistance=0;
				halfOfScan=0;
			}
			break;
		case ROTATINGTOSTART:			//rotate to the starting position
			if(fabs(getBearing()-startingBearing)<STARTINGTOLERANCE && doScan==true)
			{
				scanStatus=SCANNINGROTATION;
				limitWheelSpeed(SCANWHEELSPEED);
				setDesiredBearing(intermediaryBearing);
			}
			else if(halfOfScan>0 && doScan==true)		//if intermediate position is reached set end position
			{
				scanStatus=SCANNINGROTATION;
				setDesiredBearing(endingBearing);
				limitWheelSpeed(SCANWHEELSPEED);
			}
			else if(doScan==false)		// stop scan if told so
			{
				scanStatus=IDLE;
			}
			break;
		case SCANNINGROTATION:						//scanning
			currentDistance=VL53L0X_get_dist_mm();
			if(currentDistance>MAXDISTANCE)			//if measurement is impossible use old Value
			{
				currentDistance=oldDistance;
			}
			// the detection is based on the assumtion, that the obstacle has 2  ends. These can be detected by the change strong change of the measured distance
			// one end is when the distance changes from far to short, the other is when the distance changes from short to far
			if((oldDistance -currentDistance)>maxDelta && oldDistance!=0)	//if difference between old distance and new distance is big enough an end of the obstacle is detected
			{
				maxDelta=oldDistance - currentDistance;
				maxBearing=getBearing();
			}
			if((oldDistance -currentDistance)<minDelta && oldDistance!=0)	//if the difference between old distance and new distance is negativ enough the other end is detected
			{
				minDelta=oldDistance - currentDistance;
				minBearing=getBearing();
			}
			oldDistance=currentDistance;

			if((fabs(getBearing()-intermediaryBearing)<ENDINGTOLERANCE)&& halfOfScan==0)	// one half of the scan is done. do the other half
			{
				halfOfScan++;
				scanStatus=ROTATINGTOSTART;
			}
			else if((fabs(getBearing()-endingBearing)<ENDINGTOLERANCE) && halfOfScan>0)		// if end is reached save the directions in the correct variables
			{

				scanStatus=FINISHEDSCANNING;
				setDesiredBearing(getBearing());
				bearingFC=maxBearing;
				bearingCF=minBearing;
				if(minDelta==-MINDETECTIONDELTA)
				{
					bearingCF=3*M_PI;
				}
				if(maxDelta==MINDETECTIONDELTA)
				{
					bearingFC=3*M_PI;
				}
				limitWheelSpeed(MAXWHEELSPEED);
			}
			break;
		case FINISHEDSCANNING:
			limitWheelSpeed(MAXWHEELSPEED);
			doScan=false;
			break;

		}
		chThdSleep(MS2ST(50));
	}


}
void doScanning(bool shouldScan)
{
	doScan=shouldScan;
	if(shouldScan==true)
	{
		scanStatus=IDLE;
	}
}
scanningStep_t getScanStatus(void)
{
	return scanStatus;
}
void setScanningRange(float beginningBearing, float endBearing)
{
	if(doScan==false)
	{
		//calculated the intermediary  bearing. It is always bigger than the starting bearing to achieve ccw rotation
		intermediaryBearing = (beginningBearing +endBearing)/2;
		intermediaryBearing = (beginningBearing>intermediaryBearing) ? intermediaryBearing+M_PI :intermediaryBearing;
		//wrap the angles
		beginningBearing=wrapAngle(beginningBearing);
		intermediaryBearing=wrapAngle(intermediaryBearing);
		endBearing=wrapAngle(endBearing);
		startingBearing=beginningBearing;
		endingBearing=endBearing;
		//scanStatus=IDLE;
	}
}
void getFreeBearing(float *freeBearings,uint8_t bearingSize)
{
	if(bearingSize==NBOFBEARINGS)
	{
		freeBearings[0]=bearingFC;
		freeBearings[1]=bearingCF;
	}
	else
	{
		freeBearings=NULL;
	}
}
void startDirectionDetectionThread(void)
{
	VL53L0X_start();
	chThdCreateStatic(freeDirection_thd_was, sizeof(freeDirection_thd_was), NORMALPRIO, freeDirection_thd, NULL);
}
