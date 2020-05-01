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

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <audio_processing.h>
#include <IRSensorReading.h>

//uncomment to send the FFTs results from the real microphones
//#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

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
    //inits the motors
    startDirectionDetectionThread();
    startMotors();
    IRProcessingStart();


#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {




	   // chprintf((BaseSequentialStream *) &SD3,"state:%d, xPosition: %f, yPosition: %f\n\r",state,getXPosition(),getYPosition());
		setScanningRange(-M_PI/4, 3*M_PI/4);
		if(getScanStatus()==IDLE)
		{
				doScanning(true);
		}
		float freeBearing[2]={0,0};
		//chprintf((BaseSequentialStream *) &SD3,"State :%u\n\r",getScanStatus());
		if(getScanStatus()==FINISHEDSCANNING)
		{
			getFreeBearing(freeBearing,2);
			chprintf((BaseSequentialStream *) &SD3,"LOW HIGH: %f, HIGH LOW: %f\n\r",freeBearing[0],freeBearing[1]);
			setDesiredBearing(freeBearing[0]);
		}
		chThdSleep(MS2ST(100));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
