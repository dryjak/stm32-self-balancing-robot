/*
 * MPU6050.h
 *
 *  Created on: Sep 6, 2025
 *      Author: jakub
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

// MPU6050 I2C Address (AD0 low)
#define MPU6050_ADDR        0xD0

// MPU6050 Register Map
#define WHO_AM_I 			0x75
#define ADR0		 		0x68
#define ADR1 				0x69
#define PWR_MGMT_1			0x6B
#define GYRO_CONFIG 		0x1B
#define ACCEL_CONFIG 		0x1C
#define ACCEL_XOUT_H		0x3B
#define GYRO_XOUT_H			0x43

// Configuration Constants
#define MPU6050_TIMEOUT 	100
#define DT					0.01

// Calibration Offsets (Calculated for specific hardware)
// There is function below to calibrate your MPU6050
#define ACCEL_OFFSET_X		-621.659973
#define ACCEL_OFFSET_Y		-218.160004
#define ACCEL_OFFSET_Z		866.640625

#define GYRO_OFFSET_X		-86.4550018
#define GYRO_OFFSET_Y		-58.5900002
#define GYRO_OFFSET_Z		-310.559998

// Structure to store floating point sensor data (X, Y, Z)
typedef struct
{
	float X;
	float Y;
	float Z;
}Data_t;

// Structure to store raw sensor data (int16_t)
typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}DataRaw_t;

//Main MPU6050 handle structure
typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint16_t address;
	Data_t AccelOffset;
	Data_t GyroOffset;

}MPU6050_t;

// MPU6050 operation status
typedef enum
{
	MPU6050_OK,
	MPU6050_ERROR
}MPU6050_STATE_t;

/* --- Initialization and Setup --- */
MPU6050_STATE_t MPU6050_Init(MPU6050_t *MPU6050, I2C_HandleTypeDef *Hi2c, uint16_t Address);
MPU6050_STATE_t MPU6050_WakeUp(MPU6050_t *MPU6050);
uint8_t MPU6050_WHO_AM_I (MPU6050_t *MPU6050);

/* --- Data Acquisition --- */
MPU6050_STATE_t MPU6050_Angle(MPU6050_t *MPU6050, float *Roll, float *Pitch, float *Yaw);
MPU6050_STATE_t MPU6050_DegFromAccel(MPU6050_t *MPU6050, float *Roll, float *Pitch);
MPU6050_STATE_t MPU6050_DegFromGyro(MPU6050_t *MPU6050, float *RollG, float *PitchG, float *YawG);

/* --- Calibration --- */
MPU6050_STATE_t MPU6050_CalibrateAccel(MPU6050_t *MPU6050, Data_t *AccelCalibratedValues);
MPU6050_STATE_t MPU6050_CalibrateGyro(MPU6050_t *MPU6050, Data_t *GyroCalibratedValues);

#endif /* INC_MPU6050_H_ */
