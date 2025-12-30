/*
 * Encoder.h
 *
 *  Created on: Oct 4, 2025
 *      Author: jakub
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

typedef struct
{
	TIM_HandleTypeDef *Tim;			// Handle to the timer used for the encoder
	uint16_t PulsesPerRevolution;	// Number of pulses per one motor revolution
	float SampleTime;				// Sampling period in seconds

	uint32_t LastCounterValue;		// Timer counter value from the previous iteration

	volatile int64_t TotalPulses;	// Total sum of pulses
	float AngularVelocity;			// Calculated velocity in RPM
	float Angle;					// Current wheel angle in degrees

}Encoder_t;

//Initialization
void Encoder_Init(Encoder_t *Encoder, TIM_HandleTypeDef *Tim, uint16_t PulsesPerRevolution, float SampleTime);
//Updating encoder values
void Encoder_Update(Encoder_t *Encoder);


#endif /* INC_ENCODER_H_ */
