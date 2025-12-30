/*
 * Encoder.c
 *
 *  Created on: Oct 4, 2025
 *      Author: jakub
 */

#include "Encoder.h"

#define VELOCITY_FILTER_ALPHA 0.7f

void Encoder_Init(Encoder_t *Encoder, TIM_HandleTypeDef *Tim, uint16_t PulsesPerRevolution, float SampleTime)
{
	Encoder->Tim = Tim;
	Encoder->PulsesPerRevolution = PulsesPerRevolution;
	Encoder->SampleTime = SampleTime;

	Encoder->LastCounterValue = 0;

	Encoder->TotalPulses = 0;
	Encoder->AngularVelocity = 0.0;
	Encoder->Angle = 0.0;

	__HAL_TIM_SET_COUNTER(Encoder->Tim, 0);					// Reset counter value
	HAL_TIM_Encoder_Start(Encoder->Tim, TIM_CHANNEL_ALL);	// Start the Encoder
}
void Encoder_Update(Encoder_t *Encoder)
{
	// Get current timer counter value
    uint32_t CurrentCounterValue = __HAL_TIM_GetCounter(Encoder->Tim);

    // Calculating change in pulses
    int32_t Delta = (int32_t)(CurrentCounterValue - Encoder->LastCounterValue);

    // Update total pulses
    Encoder->TotalPulses += Delta;

    // Calculate velocity
    if (Encoder->SampleTime > 0.0001f)
    {
        float InstantVelocity = ((float)Delta / (float)Encoder->PulsesPerRevolution) / Encoder->SampleTime * 60.0f;

        // Low pass filter encoder data
        Encoder->AngularVelocity = (Encoder->AngularVelocity * VELOCITY_FILTER_ALPHA) + (InstantVelocity * (1.0f - VELOCITY_FILTER_ALPHA));
    }

    Encoder->LastCounterValue = CurrentCounterValue;
}
