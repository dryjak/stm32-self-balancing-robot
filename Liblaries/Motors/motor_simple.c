/*
 * motor_simple.c
 *
 *  Created on: Jul 26, 2025
 *      Author: jakub
 */

#include "main.h"
#include "motor_simple.h"




MotorStatus_t Motor_SetRideParameters(Motor_t *Motor, uint8_t PWM, uint8_t Dir)
{
	//Make sure PWM is in range from 0 to 100
	if (PWM > MOTOR_PWM_MAX)
	{
		Motor->MotorPWM = MOTOR_PWM_MAX;
	}
	else
	{
		Motor->MotorPWM = PWM;
	}

	//setting direction
	if (Dir == MOTOR_DIR_FORWARD) 	// Go forward
	{
		HAL_GPIO_WritePin(Motor->MotorDir1Port, Motor->MotorDir1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Motor->MotorDir2Port, Motor->MotorDir2Pin, GPIO_PIN_RESET);
	}
	else // Go backward
	{
		HAL_GPIO_WritePin(Motor->MotorDir1Port, Motor->MotorDir1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Motor->MotorDir2Port, Motor->MotorDir2Pin, GPIO_PIN_SET);
	}
	Motor->Direction = Dir;

	return MOTOR_OK;
}

void Motor_Ride(Motor_t *Motor)
{
	// Update CCR to change duty cycle
	__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Channel, Motor->MotorPWM);
}


MotorStatus_t Motor_Init(Motor_t *Motor, TIM_HandleTypeDef *Timer, uint32_t TimerChannel, uint16_t PWM, GPIO_TypeDef *Dir1Port, uint16_t Dir1Pin, GPIO_TypeDef *Dir2Port, uint16_t Dir2Pin)
{
	Motor->htim = Timer;
	Motor->Channel = TimerChannel;

	//initial speed
	Motor->MotorPWM = PWM;

	// GPIO configuration
	Motor->MotorDir1Port = Dir1Port;
	Motor->MotorDir1Pin = Dir1Pin;
	Motor->MotorDir2Port = Dir2Port;
	Motor->MotorDir2Pin = Dir2Pin;

	return MOTOR_OK;
}
