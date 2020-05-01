#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define FREQUENCYCOEFFIZIENT 15.23
#define MINSOUNDLEVEL 4000

#define FREQUENCYTOFIND 2000	//
#define MAXPHASEANGLE 3.1f		//might need checking
#define SPEEDOFSOUND 343.2f
#define MICDISTANCEFRONTBACK 0.056f
#define MICDISTANCELEFTRIGHT 0.062f
#define NBOFPHASES 2
#define FILTERCOEFFIZIENT 0.06f
#define MAXANGLEDIFF M_PI/2
#define WEAKWEIGHT 0.2f
#define STRONGWEIGHT (1-WEAKWEIGHT)
#define LEFTRIGHTMICCORRECTION 0.15f
#define FRONTBACKMICCORRECTION 0.15f



//#define NBOFAVERGINGPOINTS 12
typedef enum
{
	LEFTRIGHT1=0,					//chose Solution for -90� <alpha< 90
	LEFTRIGHT2,					// chose Solution for alpha =[180...90] or alpha = [-180 ... -90]
	FRONTBACKP,					//chose solution for 0<beta<180�
	FRONTBACKN,					//Chose solution for 0>beta>-180
	SOLIUTIONERROR=500					//error. random number
}correctSolution_t;
//static const uint16_t fftBin=(uint16_t)2*(FREQUENCYTOFIND/FREQUENCYCOEFFIZIENT);
#define MINSOUNDLEVEL 1500
#define Frequency_Goal 2000

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

typedef struct{
	float realPart;
	float imaginaryPart;
} complexNumber_t;

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

float calculateMaxFrequency(float *frequencyBuffer,uint16_t bufferSize);
float getToneFrequency(void);
<<<<<<< HEAD
float getAngle(void);
uint32_t getSoundLevel(void);
=======
float getRelativeAngle(void);

>>>>>>> FannySoundDirectionDetection

#endif /* AUDIO_PROCESSING_H */
