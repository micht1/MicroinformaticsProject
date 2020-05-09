#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "leds.h"
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include "driveMotors.h"
#include <audio/microphone.h>
#include "freeDirectionDetection.h"
#include "selector.h"

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <audio_processing.h>
#include <IRSensorReading.h>

// defines
#define ANGLEFILTER 0.1f
#define ANGLETHRESHOLD 0.30f
#define STABILITYTHRESHOLD 10

#define TRAVELSPEED 15
#define ATTREASURELEVEL 80000
#define MAINTHREADPERIOD 99
#define OVERSHOOTTIME 1000

#define ATSIDE 90
#define BACKAREA 120
#define FRONTAREA 30
#define BLINKINGCOUNTERMAXIMUM 100

#define NUMBEROFLED 4

#define ROTATINGLEDSIZE 6
const uint8_t rotatingLed[ROTATINGLEDSIZE][NUMBEROFLED] = {{0,0,0,0},{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1},{1,0,0,0}};
#define VICTOYDANCESIZE 2
const uint8_t victoryDance[VICTOYDANCESIZE][NUMBEROFLED] ={{1,0,1,0},{0,1,0,1}};
// function prototypes
bool obstacleAvoidance(float intendedTravelBearing,float *avoidanceBearing,float *obstacleDirection);
void rotateLed(bool doRotate,const uint8_t ledPattern[][NUMBEROFLED],uint8_t patternSize);
//inline functions and macros
 inline float scanRange(float x)
{
	return M_PI*(5+x)/9;
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
//static complex_float unoptimizedBuffer[FFT_SIZE];
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();


    messagebus_init(&bus,&bus_lock,&bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //starts the motor and the odometrie
    startMotors();
    //starts threat which detects from which direction the sound comes
    mic_start(&processAudioData);
    //starts the scanning Thread
    startDirectionDetectionThread();
    //starts obstacle detection
    IRProcessingStart();



    robotStatus_t currentStatus=ROBOTSTOP;
    float desiredTravelBearing=0;
    float currentSoundBearing=0;
    float filteredBearing=0;
    int oldSwitch=get_selector();
    bool soundTrigger=false;
    uint8_t stabilityCounter=0;
    bool firstSoundLock=false;
    PCMessage_t computerMessage={0};
    setMessage (&computerMessage);
    uint8_t blinkingCounter=0;
    /* Infinite loop. */
    while (1) {

    	switch(currentStatus)
    	{
    	case ROBOTSTOP:
    		if(oldSwitch!=get_selector())
    		{
    			currentStatus=ROBOTIDLE;
    		}
    		oldSwitch=get_selector();
    		break;
    	case ROBOTIDLE:

    		if(blinkingCounter%2==0)
    		{
    			rotateLed(true,rotatingLed,ROTATINGLEDSIZE);
    		}
    		if(getSoundLevel()>MINSOUNDLEVEL)
    		{
    			desiredTravelBearing=0;
    			currentSoundBearing=0;
    			filteredBearing=0;
    			firstSoundLock=false;
    			stabilityCounter=0;
    			set_body_led(0);
    			currentStatus=FOLLOWINGSOUND;
    			rotateLed(false,rotatingLed,ROTATINGLEDSIZE);
    		}
    		if(oldSwitch!=get_selector())
			{
				currentStatus=ROBOTSTOP;
				rotateLed(false,rotatingLed,ROTATINGLEDSIZE);
			}
    		oldSwitch=get_selector();
    		break;
    	case FOLLOWINGSOUND:
    		set_body_led(0);
    		if(isRotating()==false)
    		{
    			currentSoundBearing = getRelativeAngle()+getBearing();
    			filteredBearing = (isnanf(currentSoundBearing) || fabs(currentSoundBearing)>2*M_PI ) ? filteredBearing : (currentSoundBearing)*ANGLEFILTER+(1-ANGLEFILTER)*filteredBearing;
    			if(fabs(filteredBearing-currentSoundBearing)<ANGLETHRESHOLD)
				{
					stabilityCounter++;
				}
				else if(fabs(filteredBearing-currentSoundBearing)>ANGLETHRESHOLD && stabilityCounter!=0)
				{
					stabilityCounter--;
				}

    			if(stabilityCounter>STABILITYTHRESHOLD && fabs(filteredBearing-desiredTravelBearing)>ANGLETHRESHOLD)
				{
					firstSoundLock=true;
					desiredTravelBearing=wrapAngle(filteredBearing);
					stabilityCounter=0;
				}
    		}
    		if(firstSoundLock)
			{
				setDesiredBearing(desiredTravelBearing);
				setDesiredSpeed(TRAVELSPEED);
				if(presenceOfObstacle()==NOOBSTACLE)
				{
					isAllowedToDrive(true);
				}
			}
    		if(getSoundLevel()>ATTREASURELEVEL)
    		{
    			soundTrigger=true;
    		}
    		else if(getSoundLevel()<MINSOUNDLEVEL)
    		{
    			currentStatus=ROBOTIDLE;
    		}
    		if(presenceOfObstacle()>NOOBSTACLE && soundTrigger==false)
    		{
    			computerMessage.directionOfSound=desiredTravelBearing;
    			computerMessage.xPosition=getXPosition();
    			computerMessage.yPosition=getYPosition();
    			currentStatus=AVOIDINGOBSTACLE;
    		}
    		else if(presenceOfObstacle()>NOOBSTACLE && soundTrigger==true)
    		{
    			currentStatus=REACHEDTREASURE;
    		}
    		break;
    	case AVOIDINGOBSTACLE:

    		 if(obstacleAvoidance(desiredTravelBearing,&computerMessage.avoidanceDirection,&computerMessage.obstacleDirection)==true)
			 {

    			 currentStatus=ROBOTIDLE;
			 }
    		break;
    	case REACHEDTREASURE:
    		set_body_led(1);
    		setDesiredSpeed(0);
    		if(blinkingCounter%5==0)
    		{
    			rotateLed(true,victoryDance,VICTOYDANCESIZE);
    		}
    		if(oldSwitch!=get_selector())
			{
    			set_body_led(0);
    			rotateLed(false,victoryDance,VICTOYDANCESIZE);
				currentStatus=ROBOTSTOP;
			}
			oldSwitch=get_selector();
    		break;
    	case TESTING:
       		chprintf((BaseSequentialStream *) &SD3,"This is the TestState, You have no business being here\n\r");
    		break;
    	}
    	blinkingCounter = (blinkingCounter>BLINKINGCOUNTERMAXIMUM) ? 0 : (blinkingCounter+1) ;
		chThdSleep(MS2ST(MAINTHREADPERIOD));
    }
}
bool obstacleAvoidance(float intendedTravelBearing,float *avoidanceBearing,float *obstacleDirection)
{
	static float freeBearings[NBOFBEARINGS]={0};
	static float directionOfObstacle=0;
    static float avoidanceTravelDirection=0;
    static uint8_t obstacleAvoidanceStep=0;
    static uint8_t noObstacleAnymore=0;
   // bool doSkip=true;

	switch(obstacleAvoidanceStep)
	{
	case 0:
		if(!isRotating())
		{
			directionOfObstacle=wrapAngle(getObstacleDirection()/180*M_PI+getBearing());
			*obstacleDirection=directionOfObstacle;
			noObstacleAnymore=0;
			setScanningRange(directionOfObstacle-M_PI/9*5,directionOfObstacle+M_PI/9*5);
			doScanning(true);
			obstacleAvoidanceStep=1;
		}
		break;
	case 1:
		if(getScanStatus()==FINISHEDSCANNING)
		{
			getFreeBearing(freeBearings,sizeof(freeBearings)/sizeof(freeBearings[0]));
			if((freeBearings[0])>2*M_PI && (freeBearings[1])>2*M_PI)
			{
				chprintf((BaseSequentialStream *) &SD3,"No Pathfound\n\r");
				set_body_led(1);
			}
			else
			{
				obstacleAvoidanceStep=2;
				if((freeBearings[0])>2*M_PI || (freeBearings[1])>2*M_PI)
				{
					//toDo: Handeling of situation where no free bearings are found
				}
				float closestFreeBearing = (fabs(intendedTravelBearing-freeBearings[0])<fabs(intendedTravelBearing-freeBearings[1])) ? freeBearings[0]: freeBearings[1];
				float possibleTravelDirection1=(directionOfObstacle+M_PI/2);
				float possibleTravelDirection2=(directionOfObstacle-M_PI/2);
				avoidanceTravelDirection=(fabs(possibleTravelDirection1-closestFreeBearing)<fabs(possibleTravelDirection2-closestFreeBearing)) ? possibleTravelDirection1: possibleTravelDirection2;
				setDesiredBearing(avoidanceTravelDirection);

				*avoidanceBearing=avoidanceTravelDirection;
				messageReady(true);

			}
		}
		break;
	case 2:

		if(!isRotating())
		{
			ignoreObstacle(true);
			isAllowedToDrive(true);
			setDesiredSpeed(TRAVELSPEED);
			obstacleAvoidanceStep=3;
		}
		break;
	case 3:
		if(presenceOfObstacle()==DANGERCLOSE && fabs(getObstacleDirection())<BACKAREA)
		{
			obstacleAvoidanceStep=4;

			setDesiredBearing((float)getObstacleDirection()/180*M_PI+getBearing());
			setDesiredSpeed(-TRAVELSPEED);
		}
		else if(presenceOfObstacle()==NOOBSTACLE)
		{
			noObstacleAnymore++;
		}
		if(presenceOfObstacle()>NOOBSTACLE)
		{
			noObstacleAnymore = (noObstacleAnymore>0) ? (noObstacleAnymore-1) : 0;
		}
		if(noObstacleAnymore>OVERSHOOTTIME/MAINTHREADPERIOD)
		{
			obstacleAvoidanceStep=0;
			setDesiredSpeed(0);
			ignoreObstacle(false);
			return true;
		}
		break;
	case 4:
		if(presenceOfObstacle()<=OBSTACLEDETECTED)
		{
			setDesiredBearing(avoidanceTravelDirection);
			obstacleAvoidanceStep=3;
			setDesiredSpeed(TRAVELSPEED);
		}
		if(fabs(getObstacleDirection())>BACKAREA)
		{
			obstacleAvoidanceStep=5;
		}
		break;
	case 5:
		setDesiredSpeed(0);
		ignoreObstacle(false);
		obstacleAvoidanceStep=0;
		setDesiredBearing(intendedTravelBearing);
		break;
	}
	return false;
}
void rotateLed(bool doRotate,const uint8_t ledPattern[][NUMBEROFLED],uint8_t patternSize)
{
	static uint8_t ledPatternCounter=0;

	ledPattern[ledPatternCounter][0] ? set_led(LED1,1) : set_led(LED1,0);
	ledPattern[ledPatternCounter][1] ? set_led(LED3,1) : set_led(LED3,0);
	ledPattern[ledPatternCounter][2] ? set_led(LED5,1) : set_led(LED5,0);
	ledPattern[ledPatternCounter][3] ? set_led(LED7,1) : set_led(LED7,0);
	if(doRotate==false)
	{
		set_led(LED1,0);
		set_led(LED3,0);
		set_led(LED5,0);
		set_led(LED7,0);
	}
	else
	{
		ledPatternCounter=(ledPatternCounter>=patternSize-1) ? 0 : ledPatternCounter+1;
	}

}
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
