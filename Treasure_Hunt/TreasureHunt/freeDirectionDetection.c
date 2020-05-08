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


static bool doScan=false;
static scanningStep_t scanStatus=0;
static float startingBearing=0;
static float endingBearing=0;
static float intermediaryBearing=0;
static float bearingFC=0;
static float bearingCF=0;



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
			if(doScan==true)
			{
				setDesiredBearing(startingBearing);
				scanStatus=ROTATINGTOSTART;
				maxDelta=MINDETECTIONDELTA;
				minDelta=-MINDETECTIONDELTA;
				bearingCF=0;
				bearingFC=0;
				oldDistance=0;
				currentDistance=0;
				chprintf((BaseSequentialStream *) &SD3,"inter: %f start: %f, end: %f\n\r",intermediaryBearing/M_PI*180,startingBearing/M_PI*180,endingBearing/M_PI*180);
				halfOfScan=0;
			}
			break;
		case ROTATINGTOSTART:
			if(fabs(getBearing()-startingBearing)<STARTINGTOLERANCE && doScan==true)
			{
				scanStatus=SCANNINGROTATION;
				limitWheelSpeed(SCANWHEELSPEED);
				setDesiredBearing(intermediaryBearing);
				//chprintf((BaseSequentialStream *) &SD3,"endb: %f\n\r",endingBearing);
			}
			else if(halfOfScan>0 && doScan==true)
			{
				scanStatus=SCANNINGROTATION;
				setDesiredBearing(endingBearing);
				limitWheelSpeed(SCANWHEELSPEED);
			}
			else if(doScan==false)
			{
				scanStatus=IDLE;
			}
			break;
		case SCANNINGROTATION:
			currentDistance=VL53L0X_get_dist_mm();
			if(currentDistance>MAXDISTANCE)
			{
				currentDistance=oldDistance;
			}
			if((oldDistance -currentDistance)>maxDelta && oldDistance!=0)
			{
				maxDelta=oldDistance - currentDistance;
				maxBearing=getBearing();
			}
			if((oldDistance -currentDistance)<minDelta && oldDistance!=0)
			{
				minDelta=oldDistance - currentDistance;
				minBearing=getBearing();
			}
			//chprintf((BaseSequentialStream *) &SD3,"max: %d, min %d\n\r",maxDelta,minDelta);
			//chprintf((BaseSequentialStream *) &SD3,"bearing %f, maxB:\n\r",getBearing());
			oldDistance=currentDistance;
			if((fabs(getBearing()-intermediaryBearing)<ENDINGTOLERANCE)&& halfOfScan==0)
			{
				halfOfScan++;
				scanStatus=ROTATINGTOSTART;
			}
			else if((fabs(getBearing()-endingBearing)<ENDINGTOLERANCE) && halfOfScan>0)
			{

				scanStatus=FINISHEDSCANNING;
				setDesiredBearing(getBearing());
				bearingFC=maxBearing;
				bearingCF=minBearing;
				if(minDelta==-MINDETECTIONDELTA)
				{
					bearingCF=3*M_PI;
					//chprintf((BaseSequentialStream *) &SD3,"Nan1:%d\n\r",scanStatus);
				}
				if(maxDelta==MINDETECTIONDELTA)
				{
					bearingFC=3*M_PI;
					//chprintf((BaseSequentialStream *) &SD3,"Nan2\n\r");
				}
				//chprintf((BaseSequentialStream *) &SD3,"SpeedLimited\n\r");
				limitWheelSpeed(MAXWHEELSPEED);
			}
			break;
		case FINISHEDSCANNING:
			limitWheelSpeed(MAXWHEELSPEED);
			doScan=false;
			break;

		}
		//chprintf((BaseSequentialStream *) &SD3,"Scanning Status: %d\n\r",scanStatus);
		//chprintf((BaseSequentialStream *) &SD3,"status: %u\n\r",scanStatus);
		//chprintf((BaseSequentialStream *) &SD3,"status :%u, bearing: %f, currentdistance %u\n\r",scanStatus, getBearing(),VL53L0X_get_dist_mm());
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
		intermediaryBearing = (beginningBearing +endBearing)/2;
		intermediaryBearing = (beginningBearing>intermediaryBearing) ? intermediaryBearing+M_PI :intermediaryBearing;

		chprintf((BaseSequentialStream *) &SD3,"beginning: %f, end: %fn\r",beginningBearing/M_PI*180,endBearing/M_PI*180);
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
		chprintf((BaseSequentialStream *) &SD3,"fc: %f,cf %f\n\r",bearingFC,bearingCF);
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
