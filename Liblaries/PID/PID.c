/*
 * PID.c
 *
 * Created on: Oct 12, 2025
 * Author: jakub
 */
#include "PID.h"

void PID_Init(PID_t *Pid, float P, float I, float D, float SampleTime, float MaxValue, float MinValue)
{
	Pid->P = P;
	Pid->I = I;
	Pid->D = D;

	Pid->SampleTime = SampleTime;
	Pid->MaxValue = MaxValue;
	Pid->MinValue = MinValue;

	// Reset values
	Pid->Integrator = 0.0f;
	Pid->LastError = 0.0f;
	Pid->LastMeasurement = 0.0f;
	Pid->Clamp = 0;
}

float PID_Compute(PID_t *Pid, float MeasuredValue, float SetValue)
{
	float Error = SetValue - MeasuredValue;

	//Proportional value
	float P = Pid->P * Error;

	// Integral value with anti windup check
	// If clamped, we stop adding to the integrator to prevent windup
	if(Pid->Clamp == 0)
	{
	    Pid->Integrator += Pid->SampleTime * Pid->I * Error;
	}

	 //Derivative value
	/* Instead of calculating (Error - LastError) we use (MeasuredValue - LastMeasurementValue)
	   It prevents "Derivative Kick" when setpoint changes rapidly*/
	float D = 0.0f;
	if (Pid->SampleTime > 0) // Prevent division by 0
	{
		D = -1.0f * ((MeasuredValue - Pid->LastMeasurement) / Pid->SampleTime) * Pid->D;
	}

	// Calculate total output
	float Output = P + Pid->Integrator + D;
	float OutputLast = Output;

	// Check saturation limits
	if (Output > Pid->MaxValue)
	{
		Output = Pid->MaxValue;
	}
	else if (Output < Pid->MinValue)
	{
		Output = Pid->MinValue;
	}

	//Anti-Windup logic with clamping
	/*
	 If output is saturated (Output != OutputLast) and error sign is the same as output
	 sign (meaning Error is trying to push further into saturation, then we clamp
	*/
	uint8_t ClampigSaturationCheck = (Output != OutputLast) ? 1 : 0;

	int8_t ErrorSign = Signum(Error);
	int8_t OutputSign = Signum(Output);

	if ((ErrorSign == OutputSign) && (ClampigSaturationCheck == 1))
	{
		Pid->Clamp = 1;	//Stop integration
	}
	else
	{
		Pid->Clamp = 0;	//Don't stop integration
	}

	Pid->LastError = Error;

    Pid->LastMeasurement = MeasuredValue;

	return Output;
}

int8_t Signum(float Value)
{
	if (Value > 0.0f) return 1;
	if (Value < 0.0f) return -1;
	return 0;
}
