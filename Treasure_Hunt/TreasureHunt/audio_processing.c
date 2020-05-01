#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static float toneFrequency=0;
static float relativeAngle=0;
int Frequency_Position;

static float anglePhase=0;



const unsigned int freqBin=(unsigned int)(FREQUENCYTOFIND/FREQUENCYCOEFFIZIENT);
void microPhonePhaseShift(complexNumber_t *complexFFTNumber1,complexNumber_t *complexFFTNumber2,float *phaseShift, uint8_t phaseArraySize)
{
	static float oldPhase[NBOFPHASES];
	for(uint8_t phaseCounter=0; phaseCounter<NBOFPHASES;phaseCounter++)
	{

		float phase1=atan2f(complexFFTNumber1[phaseCounter].imaginaryPart,complexFFTNumber1[phaseCounter].realPart);
		float phase2=atan2f((complexFFTNumber2[phaseCounter].imaginaryPart),(complexFFTNumber2[phaseCounter].realPart));

		if(fabs(phase1-phase2)<MAXPHASEANGLE)
		{
			phaseShift[phaseCounter]=(phase1-phase2);

			if(fabs(phaseShift[phaseCounter]-oldPhase[phaseCounter])<MAXPHASEANGLE)
			{
				oldPhase[phaseCounter]=phaseShift[phaseCounter];
			}
			else
			{
				phaseShift[phaseCounter]=oldPhase[phaseCounter];
			}
		}
		else
		{
			phaseShift[phaseCounter]=oldPhase[phaseCounter];
			//chprintf((BaseSequentialStream *)&SD3,"Skipped!\n\r");
		}
	}
}

float calculateDirectionOfSound(float *phaseShift,uint8_t phaseArraySize,uint16_t usedFrequency)
{
	float angle=0;
	if(phaseArraySize==NBOFPHASES)
	{
		// angle calculation from Phase difference between the Front/Back Mic and the Left/Right Mic
		float angleLR1 = asin(SPEEDOFSOUND*(phaseShift[0])/(usedFrequency*MICDISTANCELEFTRIGHT*2*M_PI));
		float angleFB1 = acos(SPEEDOFSOUND*(phaseShift[1])/(usedFrequency*MICDISTANCEFRONTBACK*2*M_PI));
		// calculating the second solution for Angle, not necessary for front/back phaseshift
		float angleLR2 = (angleLR1>0) ? (M_PI-angleLR1):(-M_PI-angleLR1);



		// if the calculation gives NaN,
		// then use phase shift data to determine which solution of the acos/asin should be used
		// if no NaN is present proceed to determine the correct solution.
		if(isnanf(angleLR1) && !isnanf(angleFB1))
		{
			if(phaseShift[0]>0)
			{
				angle=angleFB1;
			}
			else
			{
				angle=-angleFB1;
			}
			set_body_led(1);
		}
		else if(isnanf(angleFB1) && !isnanf(angleLR1))
		{
			if(phaseShift[1]>0)
			{
				angle=angleLR1;
			}
			else
			{
				angle=angleLR2;
			}
			set_body_led(1);
		}
		else if(isnanf(angleFB1) && isnanf(angleLR1))
		{
			angle=NAN;
			chprintf((BaseSequentialStream *)&SD3,"NaN\n\r");
		}
		else
		{

			float leftRightWeight=0;
			float frontBackWeight=0;
			if((fabs(angleLR1)< fabs(angleFB1-M_PI/2)))
			{
				leftRightWeight=STRONGWEIGHT;
				frontBackWeight=WEAKWEIGHT;
			}
			else if((fabs(angleLR1)> fabs(angleFB1-M_PI/2)))
			{
				leftRightWeight=WEAKWEIGHT;
				frontBackWeight=STRONGWEIGHT;
			}
			else
			{
				chprintf((BaseSequentialStream *)&SD3," O.O \n\r");
			}


			float angleDiff[]={fabs(angleLR1-angleFB1),fabs(angleLR1+angleFB1),fabs(angleLR2-angleFB1),fabs(angleLR2+angleFB1)};
			uint8_t smallestDiff=LEFTRIGHT1;				//A number greater than 4, so that if nothing was the smallest, this event can be detected
			for(uint8_t diffSorting=1;diffSorting<sizeof(angleDiff)/sizeof(angleDiff[0]);diffSorting++)
			{
				if(angleDiff[smallestDiff]>angleDiff[diffSorting])
				{
					smallestDiff=diffSorting;
				}
			}
			switch(smallestDiff)
			{
			case LEFTRIGHT1:
				angle =	angleLR1*leftRightWeight+angleFB1*frontBackWeight;		//toDo: determine which side is positive left side is positive
				break;
			case LEFTRIGHT2:
				angle =	angleLR1*leftRightWeight-angleFB1*frontBackWeight;
				break;
			case FRONTBACKP:

				angle =	angleLR2*leftRightWeight+angleFB1*frontBackWeight;
				break;
			case FRONTBACKN:
				angle =	angleLR2*leftRightWeight-angleFB1*frontBackWeight;
				break;
			}

			set_body_led(0);
			chprintf((BaseSequentialStream *)&SD3,"smallestDiff : %u\n\r",smallestDiff);
		}
		// functions used to debug

		//chprintf((BaseSequentialStream *)&SD3,"angleLR1: %f, angleLR2: %f ,angleFB1 %f, angleFB2 %f\n\r",angleLR1,angleLR2,angleFB1,-angleFB1);

		//chprintf((BaseSequentialStream *)&SD3,"phaseLR %f phase FB: %f \n\r",phaseShift[0],phaseShift[1]);

	}


	return angle;
}
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t bufferCounter=0;
	static float phaseShift[NBOFPHASES]={0};
	//static uint8_t bufferOverflow=0;
	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	for(uint16_t sampleCounter=0;sampleCounter<num_samples;sampleCounter++)
	{
		if(sampleCounter%4==0)
		{
			micRight_cmplx_input[bufferCounter*2]=(float)data[sampleCounter];
		}
		else if(sampleCounter%4==1)
		{
			micLeft_cmplx_input[bufferCounter*2]=(float)data[sampleCounter];
		}
		else if(sampleCounter%4==2)
		{
			micBack_cmplx_input[bufferCounter*2]=(float)data[sampleCounter];
		}
		else if(sampleCounter%4==3)
		{
			micFront_cmplx_input[bufferCounter*2]=(float)data[sampleCounter];
			bufferCounter++;
		}

		if(bufferCounter>=FFT_SIZE)
		{
			sampleCounter=num_samples;
		}
		else
		{
			micRight_cmplx_input[bufferCounter*2+1]=0;
			micLeft_cmplx_input[bufferCounter*2+1]=0;
			micBack_cmplx_input[bufferCounter*2+1]=0;
			micFront_cmplx_input[bufferCounter*2+1]=0;
		}
	}
	if(bufferCounter>=FFT_SIZE)
	{
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);

		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);


		bufferCounter=0;
==
		Frequency_Position=2*(Frequency_Goal/FREQUENCYCOEFFIZIENT);
		//Filtering of the unwanted frequency and putting the good one in separated variables
		//calculation of the position
		micLeftPhase.realPart=micLeft_cmplx_input[Frequency_Position];
		micLeftPhase.imaginaryPart=micLeft_cmplx_input[Frequency_Position+1];
		micRightPhase.realPart=micRight_cmplx_input[Frequency_Position];
		micRightPhase.imaginaryPart=micRight_cmplx_input[Frequency_Position+1];
		micFrontPhase.realPart=micFront_cmplx_input[Frequency_Position];
		micFrontPhase.imaginaryPart=micFront_cmplx_input[Frequency_Position+1];
		micBackPhase.realPart=micBack_cmplx_input[Frequency_Position];
		micBackPhase.imaginaryPart=micBack_cmplx_input[Frequency_Position+1];
        ==
		complexNumber_t micFFTComplexNumbers1[NBOFPHASES]={0};
		complexNumber_t micFFTComplexNumbers2[NBOFPHASES]={0};

		if(micRight_output[freqBin]>MINSOUNDLEVEL && micLeft_output[freqBin]>MINSOUNDLEVEL && micFront_output[freqBin]>MINSOUNDLEVEL && micBack_output[freqBin]>MINSOUNDLEVEL)
		{

			micFFTComplexNumbers1[0].realPart=micLeft_cmplx_input[2*freqBin]; 	micFFTComplexNumbers1[0].imaginaryPart=micLeft_cmplx_input[(2*freqBin)+1];
			micFFTComplexNumbers1[1].realPart=micFront_cmplx_input[2*freqBin];	micFFTComplexNumbers1[1].imaginaryPart=micFront_cmplx_input[(2*freqBin)+1];


			micFFTComplexNumbers2[0].realPart=micRight_cmplx_input[2*freqBin]; 	micFFTComplexNumbers2[0].imaginaryPart=micRight_cmplx_input[(2*freqBin)+1];
			micFFTComplexNumbers2[1].realPart=micBack_cmplx_input[2*freqBin]; 	micFFTComplexNumbers2[1].imaginaryPart=micBack_cmplx_input[(2*freqBin)+1];


			float phaseShiftRaw[NBOFPHASES]={0};


			microPhonePhaseShift(micFFTComplexNumbers1,micFFTComplexNumbers2,phaseShiftRaw,NBOFPHASES);

			for(uint8_t phaseCounter=0;phaseCounter<NBOFPHASES;phaseCounter++)
			{
				phaseShift[phaseCounter]=FILTERCOEFFIZIENT*phaseShiftRaw[phaseCounter]+(1-FILTERCOEFFIZIENT)*phaseShift[phaseCounter];
			}

			float anglebla = calculateDirectionOfSound(phaseShift,NBOFPHASES,FREQUENCYTOFIND);
			anglePhase = anglebla;
			float angleCalculated= (anglebla)/M_PI*180;

			chprintf((BaseSequentialStream *)&SD3,"Angle %f\n\r",angleCalculated);

		}
		else
		{
			//chprintf((BaseSequentialStream *)&SD3,"F1: %f,F2: %f,F3: %f,F4: %f\n\r",maxFrequencys[0],maxFrequencys[1],maxFrequencys[2],maxFrequencys[3]);
		}



		//
		//please ignore
		chBSemSignal(&sendToComputer_sem);
		//toneFrequency=calculateMaxFrequency(micFront_output,FFT_SIZE);
	}

}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
float getAngle(void)
{
	return anglePhase;
}

uint32_t getSoundLevel(void)
{
	return (uint32_t)(micLeft_output[freqBin]+micRight_output[freqBin]+micFront_output[freqBin]+micBack_output[freqBin])/4;
}
float calculateMaxFrequency(float *frequencyBuffer,uint16_t bufferSize)
{
	float maxFrequency[2]={0};
	if(bufferSize==FFT_SIZE)
	{
		for(uint16_t freqBufCounter=0;freqBufCounter<FFT_SIZE/2;freqBufCounter++)
		{

			if(frequencyBuffer[freqBufCounter]>maxFrequency[0]&&frequencyBuffer[freqBufCounter]>MINSOUNDLEVEL)
			{
				maxFrequency[0]=frequencyBuffer[freqBufCounter];
				maxFrequency[1]=freqBufCounter;
			}
		}
		maxFrequency[1]=maxFrequency[1]*FREQUENCYCOEFFIZIENT;
	}
	chprintf((BaseSequentialStream *)&SD3,"F: %f,l %f\n\r",maxFrequency[1],maxFrequency[0]);
	return maxFrequency[1];
}

float getRelativeAngle(void)
{
	return relativeAngle;
}



