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

static float relativeAngle=0;		// angle from which the sound comes from,


//calculation of the frequencyposition
static const int freqBin=(unsigned int)(FREQUENCYTOFIND/FREQUENCYCOEFFIZIENT);

/**
 * @brief: non public function, which calculates the phaseshift between the argument of number1 and the argument of number2.
 * when the phaseshift is to0 great, or too different from the phase the calculation before the difference of the calculation before is saved in the location where phaseShift points to
 *
 * @param[in] complexFFTNumber1 pointer onto the variable which contains the first complex number, can also be an array of complex numbers
 * @param[in] complexFFTNumber2 pointer onto the variable which contains the second complex number, can also be an array of complex numbers
 * @param[out] phaseShift pointer onto variable where the difference between the argument of the first complex number and the second complex number should be saved
 * @param[in] phaseArraySize the size of the array, when arrays are used.
 */
void microPhonePhaseShift(complexNumber_t *complexFFTNumber1,complexNumber_t *complexFFTNumber2,float *phaseShift, uint8_t phaseArraySize)
{
	static float oldPhase[NBOFPHASES];				//variable used to store the phase of the previous calculation
	if(phaseArraySize==NBOFPHASES)
	{
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
}
/**
 * @brief: Non public function. calculates the direction of sound from the supplied phases.
 * Takes 2 phase. function assumes that the first phase are from microphones situated at +90 and -90 from 0 degrees
 * It also assumes that the second phase come from microphones situated a 0 and 180 degrees from 0 degrees
 * calculation is based on the following formula:
 * angle = asin(SPEEDOFSOUND*(phaseShift)/(usedFrequency*MICDISTANCELEFTRIGHT*2*M_PI))
 *
 * @param[in] phaseShift array with the phasedifferences in it
 * @param[in] size of the array
 * @param[in] frequency which should be used in the direction calculation
 * @return angle or 3*PI if the no angle can be calculated from the supplied phasedifferences
 */

float calculateDirectionOfSound(float *phaseShift,uint8_t phaseArraySize,uint16_t usedFrequency)
{
	float angle=0;
	if(phaseArraySize==NBOFPHASES)
	{
		// angle calculation from Phase difference between the Front/Back Mic and the Left/Right Mic
		float angleLR1 = asin(SPEEDOFSOUND*(phaseShift[0])/(usedFrequency*MICDISTANCELEFTRIGHT*2*M_PI));
		float angleFB1 = acos(SPEEDOFSOUND*(phaseShift[1])/(usedFrequency*MICDISTANCEFRONTBACK*2*M_PI));
		// calculating the second solution for Angle, not necessary for front/back phaseshift, because the second solution is just the negativ of the other
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
		}
		else if(isnanf(angleFB1) && isnanf(angleLR1))
		{
			angle=3*M_PI;		// since 3*Pi is impossible
		}
		else
		{
			//since acos and asin both are defined on a limited numbers range, 4 solutions are possible
			// in this part the 4 solutions are compared to determine which is the correcet one
			float leftRightWeight=0;
			float frontBackWeight=0;
			// if the left and right microphone gives an solution to the front or the back, then this is the more trustworthy solution
			if((fabs(angleLR1)< fabs(angleFB1-M_PI/2)))
			{
				leftRightWeight=STRONGWEIGHT;
				frontBackWeight=WEAKWEIGHT;
			}
			else if((fabs(angleLR1)> fabs(angleFB1-M_PI/2)))	// if the front and back microphone give a solution to the right or the left of the robot, those are more trustworthy
			{
				leftRightWeight=WEAKWEIGHT;
				frontBackWeight=STRONGWEIGHT;
			}
			//calculate the difference between the sollutions of acos and asin, the solution of asin which is closest to a solution of acos is choosen as the correct solution.
			//In theory one solution of acos should be exactly the same as one solution of asin
			float angleDiff[]={fabs(angleLR1-angleFB1),fabs(angleLR1+angleFB1),fabs(angleLR2-angleFB1),fabs(angleLR2+angleFB1)};
			uint8_t smallestDiff=LEFTRIGHT1;				//A number greater than 4, so that if nothing was the smallest, this event can be detected
			for(uint8_t diffSorting=1;diffSorting<sizeof(angleDiff)/sizeof(angleDiff[0]);diffSorting++)
			{
				if(angleDiff[smallestDiff]>angleDiff[diffSorting])
				{
					smallestDiff=diffSorting;
				}
			}
			//calculate the angle from one of the solution of acos and asin with a weighted average
			switch(smallestDiff)
			{
			case LEFTRIGHT1:
				angle =	angleLR1*leftRightWeight+angleFB1*frontBackWeight;
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
		}
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
	//if buffer full, do fft and calculate sound angle
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



		complexNumber_t micFFTComplexNumbers1[NBOFPHASES]={0};
		complexNumber_t micFFTComplexNumbers2[NBOFPHASES]={0};
		//only calculate angle if the sound of the searched for frequency is high enough
		if(micRight_output[freqBin]>MINSOUNDLEVEL && micLeft_output[freqBin]>MINSOUNDLEVEL && micFront_output[freqBin]>MINSOUNDLEVEL && micBack_output[freqBin]>MINSOUNDLEVEL)
		{
			//Filtering of the unwanted frequency and putting the good one in separated variables
			micFFTComplexNumbers1[0].realPart=micLeft_cmplx_input[2*freqBin]; 	micFFTComplexNumbers1[0].imaginaryPart=micLeft_cmplx_input[(2*freqBin)+1];
			micFFTComplexNumbers1[1].realPart=micFront_cmplx_input[2*freqBin];	micFFTComplexNumbers1[1].imaginaryPart=micFront_cmplx_input[(2*freqBin)+1];

			micFFTComplexNumbers2[0].realPart=micRight_cmplx_input[2*freqBin]; 	micFFTComplexNumbers2[0].imaginaryPart=micRight_cmplx_input[(2*freqBin)+1];
			micFFTComplexNumbers2[1].realPart=micBack_cmplx_input[2*freqBin]; 	micFFTComplexNumbers2[1].imaginaryPart=micBack_cmplx_input[(2*freqBin)+1];

			//calculate phaseshift and filter it
			float phaseShiftRaw[NBOFPHASES]={0};
			microPhonePhaseShift(micFFTComplexNumbers1,micFFTComplexNumbers2,phaseShiftRaw,NBOFPHASES);

			for(uint8_t phaseCounter=0;phaseCounter<NBOFPHASES;phaseCounter++)
			{
				phaseShift[phaseCounter]=FILTERCOEFFIZIENT*phaseShiftRaw[phaseCounter]+(1-FILTERCOEFFIZIENT)*phaseShift[phaseCounter];
			}
			//calculate angle
			relativeAngle = calculateDirectionOfSound(phaseShift,NBOFPHASES,FREQUENCYTOFIND);
		}
	}

}

uint32_t getSoundLevel(void)
{
	return (uint32_t)(micLeft_output[freqBin]+micRight_output[freqBin]+micFront_output[freqBin]+micBack_output[freqBin])/4;
}
float getRelativeAngle(void)
{
	return relativeAngle;
}



