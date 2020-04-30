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

static complexNumber_t micLeftPhase;
static complexNumber_t micRightPhase;
static complexNumber_t micFrontPhase;
static complexNumber_t micBackPhase;


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



		//calculation of the position
		Frequency_Position=2*(Frequency_Goal/FREQUENCYCOEFFIZIENT);

		//Filtering of the unwanted frequency and putting the good one in separated variables
		micLeftPhase.realPart=micLeft_cmplx_input[Frequency_Position];
		micLeftPhase.imaginaryPart=micLeft_cmplx_input[Frequency_Position+1];

		micRightPhase.realPart=micRight_cmplx_input[Frequency_Position];
		micRightPhase.imaginaryPart=micRight_cmplx_input[Frequency_Position+1];

		micFrontPhase.realPart=micFront_cmplx_input[Frequency_Position];
		micFrontPhase.imaginaryPart=micFront_cmplx_input[Frequency_Position+1];

		micBackPhase.realPart=micBack_cmplx_input[Frequency_Position];
		micBackPhase.imaginaryPart=micBack_cmplx_input[Frequency_Position+1];


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
	return maxFrequency[1];
}
float getToneFrequency(void)
{
	return toneFrequency;
}

float getRelativeAngle(void)
{
	return relativeAngle;
}



