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
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <audio_processing.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

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
//static complex_float unoptimizedBuffer[FFT_SIZE];
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts

        //SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);
        /*for(uint16_t bufferCount=0;bufferCount<FFT_SIZE;bufferCount++)
        {
        	unoptimizedBuffer[bufferCount].real = bufferCmplxInput[bufferCount*2];
        	unoptimizedBuffer[bufferCount].imag=0;
        }*/

        if(size == FFT_SIZE){


        	//doFFT_c(FFT_SIZE,unoptimizedBuffer);
        	/*for(uint16_t bufferCount=0;bufferCount<FFT_SIZE*2;bufferCount++)
        	{
        		if(bufferCount%2==0)
        		{
        			bufferCmplxInput[bufferCount]=unoptimizedBuffer[bufferCount/2].real;
        		}
        		else
        		{
        			bufferCmplxInput[bufferCount]=unoptimizedBuffer[bufferCount/2].imag;
        		}
        	}*/

			doFFT_optimized(FFT_SIZE, bufferCmplxInput);
            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            //SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

        }
#endif  /* SEND_FROM_MIC */
       // chprintf((BaseSequentialStream *)&SDU1,"Frequency: %f\n\r",getToneFrequency());
        int leftMotorSpeed=0;
        int rightMotorSpeed=0;
        uint32_t volume=getSoundLevel();

        //leftMotorSpeed=100;
        //rightMotorSpeed=-100;
        float soundAngle=getRelativeAngle();
        uint8_t patternToUse=0;
        if(soundAngle<M_PI/4 && soundAngle>-M_PI/4  )
        {
        	set_led(LED1,1);
			set_led(LED3,0);
			set_led(LED5,0);
			set_led(LED7,0);
        }
        else if(soundAngle>M_PI/4 && soundAngle<3*M_PI/4)
        {
        	set_led(LED1,0);
			set_led(LED3,0);
			set_led(LED5,0);
			set_led(LED7,1);
        }
        else if(soundAngle>3*M_PI/4)
        {
        	set_led(LED1,0);
			set_led(LED3,0);
			set_led(LED5,1);
			set_led(LED7,0);
        }
        else if(soundAngle<-M_PI/4 && soundAngle>-3* M_PI/4)
        {
        	set_led(LED1,0);
			set_led(LED3,1);
			set_led(LED5,0);
			set_led(LED7,0);
        }
        else if(soundAngle<-3*M_PI/4)
        {
        	set_led(LED1,0);
			set_led(LED3,0);
			set_led(LED5,1);
			set_led(LED7,0);
        }
        else
        {
        	if(isnanf(soundAngle))
        	{
        		set_rgb_led(LED2,200,200,0);
        		set_rgb_led(LED7,200,200,0);
        	}
        	set_led(LED1,1);
			set_led(LED3,0);
			set_led(LED5,1);
			set_led(LED7,0);
        }



        if(volume>10000)
        {
			left_motor_set_speed(leftMotorSpeed);
			right_motor_set_speed(rightMotorSpeed);
        }
        else
        {
        	left_motor_set_speed(0);
			right_motor_set_speed(0);
        }

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
