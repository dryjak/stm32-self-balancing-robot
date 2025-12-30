/*
 * motor_simple.h
 *
 *  Created on: Jul 26, 2025
 *      Author: jakub
 */

#ifndef INC_MOTOR_SIMPLE_H_
#define INC_MOTOR_SIMPLE_H_

#include "main.h"

// Direction definitions for better readability
#define MOTOR_DIR_FORWARD   1
#define MOTOR_DIR_BACKWARD  0

// PWM Limits corresponding to Timer ARR = 99
#define MOTOR_PWM_MAX       100
#define MOTOR_PWM_MIN       0

typedef enum{
	MOTOR_OK = 0,
	MOTOR_ERROR
}MotorStatus_t;

// Main Motor handle structure
typedef struct
{
	TIM_HandleTypeDef	*htim;
	uint32_t 			Channel;

	GPIO_TypeDef 		*MotorDir1Port;
	uint16_t			MotorDir1Pin;

	GPIO_TypeDef 		*MotorDir2Port;
	uint16_t			MotorDir2Pin;

	uint8_t 			MotorPWM;
	uint8_t				Direction;

}Motor_t;

//Initialization
/*  This function DOES NOT start the PWM timer. Use HAL_TIM_PWM_Start() manually. */
MotorStatus_t Motor_Init(Motor_t *Motor, TIM_HandleTypeDef *Timer, uint32_t TimerChannel, uint16_t PWM, GPIO_TypeDef *Dir1Port, uint16_t Dir1Pin, GPIO_TypeDef *Dir2Port, uint16_t Dir2Pin);
//Set target parameters (PWM and Direction) in the struct and updates GPIOs.
MotorStatus_t Motor_SetRideParameters(Motor_t *Motor, uint8_t PWM, uint8_t Dir);
//Call Motor_Ride() to apply the PWM change to the timer.
void Motor_Ride(Motor_t *Motor);


#endif /* INC_MOTOR_SIMPLE_H_ */
