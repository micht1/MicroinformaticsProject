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
#define ANGLEFILTER 0.1f			// filtecoeffizient used in the simple filter of the angle measurement
#define ANGLETHRESHOLD 0.30f		// angle under which the newly measured angle is close enough to the filtered angle to be considered stable
#define STABILITYTHRESHOLD 10		// defines how many new angles need to be considered stable before the travel direction is changed

#define TRAVELSPEED 15				// speed at which the robot normally should travel
#define ATTREASURELEVEL 90000		// sound level which indicates that the robot is near the goal
#define MAINTHREADPERIOD 100
#define OVERSHOOTTIME 1000			// amount of time the robot sould travel on the avoidance path after no obstacle was detected anymore

#define ATSIDE 90					// side is at +- 90°
#define BACKAREA 120				// behind robot is at+-120§
#define FRONTAREA 30				//in front of the robot is at +-30°
#define BLINKINGCOUNTERMAXIMUM 99	// maximum to where the counter counts

#define NUMBEROFLED 4				// number of red led

// 2 blinking patterns
#define ROTATINGLEDSIZE 6
const uint8_t rotatingLed[ROTATINGLEDSIZE][NUMBEROFLED] = {{0,0,0,0},{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1},{1,0,0,0}};
#define VICTOYDANCESIZE 2
const uint8_t victoryDance[VICTOYDANCESIZE][NUMBEROFLED] ={{1,0,1,0},{0,1,0,1}};
// function prototypes
bool obstacleAvoidance(float intendedTravelBearing,float *avoidanceBearing,float *obstacleDirection);
void rotateLed(bool doRotate,const uint8_t ledPattern[][NUMBEROFLED],uint8_t patternSize);


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
int main(void)
{
	//initialise peripherie and start threads
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



    robotStatus_t currentStatus=ROBOTSTOP;		//	variable which tracks status of the robot
    float desiredTravelBearing=0;				// 	variable in which the travel direction is saved
    float currentSoundBearing=0;				//	variable in which the newest measurement of the sound direction is saved
    float filteredBearing=0;					// 	variable in which the filtered sound direction is saved
    int oldSwitch=get_selector();				//variable used to catch a change in the selector
    bool soundTrigger=false;					//variable which is used to indicate if the robot has encountered the sound level assosiated with being near goal
    uint8_t stabilityCounter=0;					// counter used to count how many of the sound direction measurments in a row are near the filtered direction
    bool firstSoundLock=false;					// variable to prevent random movement before the first proper direction is determined
    PCMessage_t computerMessage={0};			//buffer for the computer message
    setMessage (&computerMessage);				// set the buffer as the buffer to use
    uint8_t blinkingCounter=0;					//counter used to generate blinking
    /* Infinite loop. */
    while (1) {

    	switch(currentStatus)
    	{
    	case ROBOTSTOP:							// do nothing
    		if(oldSwitch!=get_selector())
    		{
    			currentStatus=ROBOTIDLE;
    		}
    		oldSwitch=get_selector();
    		break;
    	case ROBOTIDLE:						// wait unitl loud enough sound is detected

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
    			soundTrigger=false;
    			rotateLed(false,rotatingLed,ROTATINGLEDSIZE);
    		}
    		if(oldSwitch!=get_selector())
			{
				currentStatus=ROBOTSTOP;
				rotateLed(false,rotatingLed,ROTATINGLEDSIZE);
			}
    		oldSwitch=get_selector();
    		break;
    	case FOLLOWINGSOUND:			// calculate sound direction and follow it. if and obstacle is detected go into avoidance mode
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
    	case AVOIDINGOBSTACLE:		//avoid obstacle

    		 if(obstacleAvoidance(desiredTravelBearing,&computerMessage.avoidanceDirection,&computerMessage.obstacleDirection)==true)
			 {

    			 currentStatus=ROBOTIDLE;
			 }
    		break;
    	case REACHEDTREASURE:	//make a victory dance!(means blink leds)
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
    	case TESTING:		//if needed test programms can be inserted here.
       		chprintf((BaseSequentialStream *) &SD3,"This is the TestState, You have no business being here\n\r");
       		break;
    	}
    	blinkingCounter = (blinkingCounter>BLINKINGCOUNTERMAXIMUM) ? 0 : (blinkingCounter+1) ;
		chThdSleep(MS2ST(MAINTHREADPERIOD));
    }
}
/**
 * @brief: function which steers the robot so that he avoid the obstacle and in best cease closer to the sound source
 *
 * The obstacle avoidance will first search for both edges of the obstacle.
 * It will then determine to which one it should drive in order to drive closer to the sound source
 * Since the direction in which the obstacle is, can not be determined with a great amound of resolution, it often wont drive parallel to the obstacle even though it wants to.
 * So when the robot detects, that the obstacle came too close it drives so that he distances himself a bit again
 *
 * @param[in] intendedTravelBearing: direction in the robot wanted to travel, which also means the direction of the sound
 * @param[out] avoidanceBearing: pointer where the direction the robot wants to travel to avoid the obstacle should be saved
 * @param[out] obstacleDirection: pointer where the direction of the Obstacle should be saved
 * @return: true if function thinks it has successfully avoided the obstacle, false if it is still in progress
 */
bool obstacleAvoidance(float intendedTravelBearing,float *avoidanceBearing,float *obstacleDirection)
{
	static float freeBearings[NBOFBEARINGS]={0};
	static float directionOfObstacle=0;
    static float avoidanceTravelDirection=0;
    static uint8_t obstacleAvoidanceStep=0;			//variable used to track in which step in the avoidance process the robot is
    static uint8_t noObstacleAnymore=0;				// ounter used to count for how many function calls no obstacle was detected

	switch(obstacleAvoidanceStep)
	{
	case 0:									//set all variables to the proper starting values, set the scanning range and start the scan for an unobstructed path
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
	case 1:							// wait until scan has finished, calculate the avoidance path, indicate that the message buffer should now be filled and the message send to the computer
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
	case 2:	//start travel along the avoidance direction

		if(!isRotating())
		{
			ignoreObstacle(true);
			isAllowedToDrive(true);
			setDesiredSpeed(TRAVELSPEED);
			obstacleAvoidanceStep=3;
		}
		break;
		//travel as lon as needded until no obstacle i detected anymore, or when the obstacle comes too close to the robot create more distance to the obstacle.
		//if number of times no obstacle is detected is high enough indicate that the obstacle was aovided successfully
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
		if(fabs(getObstacleDirection())>BACKAREA)	// if data doesnt make sense start from the beginning again
		{
			obstacleAvoidanceStep=5;
		}
		break;
	case 5:				//an error occurred somewhere ->everything needs to be reset properly
		setDesiredSpeed(0);
		ignoreObstacle(false);
		obstacleAvoidanceStep=0;
		setDesiredBearing(intendedTravelBearing);
		break;
	}
	return false;
}
/**
 * @brief: with each call steps through the supplied Led pattern
 *
 * @param[in] doRotate if it hsould do blink the led, set to true if not set to false, setting it to false will also put out all red ledss
 * @param[in] ledPattern pattern in which the leds should blink
 * @param[in] patternSize number of differend steps in the blinking pattern
 */
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
