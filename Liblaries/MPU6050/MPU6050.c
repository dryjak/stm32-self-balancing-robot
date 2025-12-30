/*
 * MPU6050.c
 *
 *  Created on: Sep 6, 2025
 *      Author: jakub
 */

#include "MPU6050.h"
#include "math.h"

// Sensor sensitivity scales
#define ACCEL_SCALE_FACTOR  16384.0f // +/- 2g
#define GYRO_SCALE_FACTOR  	65.5f	 // +/- 500 deg/s

// Complementary filter constant
#define FILTER_ALPHA            0.98f

/* --- Private Function Prototypes --- */
// Help functions to read and write to MPU6050
static uint8_t Read8(MPU6050_t *MPU6050, uint8_t Register);
static MPU6050_STATE_t Write8(MPU6050_t *MPU6050, uint8_t Register, uint8_t Value);
static int16_t Read16(MPU6050_t *MPU6050, uint8_t Register);

// Setting up proper range of accelerometer and gyroscope
static MPU6050_STATE_t MPU6050_SetGyroRange(MPU6050_t *MPU6050);
static MPU6050_STATE_t MPU6050_SetAccelerationRange(MPU6050_t *MPU6050);

//Functions to read raw accelerometer and gyroscope data
static MPU6050_STATE_t MPU6050_ReadAccelerationRaw(MPU6050_t *MPU6050, DataRaw_t *AccelRaw);
static MPU6050_STATE_t MPU6050_ReadAcceleration(MPU6050_t *MPU6050, Data_t *Accelerations);
static MPU6050_STATE_t MPU6050_ReadGyroRaw(MPU6050_t *MPU6050, DataRaw_t *GyroRaw);
static MPU6050_STATE_t MPU6050_ReadGyro(MPU6050_t *MPU6050, Data_t *GyroCalculated);

//Complementary filter function to estimate current angle
static void ComplementaryFilter(float *roll, float *pitch, float roll_accel, float pitch_accel);

//Initialize MPU6050
MPU6050_STATE_t MPU6050_Init(MPU6050_t *MPU6050, I2C_HandleTypeDef *Hi2c, uint16_t Address)
{
    uint8_t CheckID;

    MPU6050->hi2c     = Hi2c;
    MPU6050->address  = Address;

    CheckID = Read8(MPU6050, 0x75);
    if (CheckID != 0x68) 										//Checking sensor ID
    {
        return MPU6050_ERROR;
    }

    if (MPU6050_WakeUp(MPU6050) != MPU6050_OK) 					//Waking up the sensor
    {
        return MPU6050_ERROR;
    }
    if (MPU6050_SetAccelerationRange(MPU6050) != MPU6050_OK) 	//Setting acceleration range
    {
        return MPU6050_ERROR;
    }
    if (MPU6050_SetGyroRange(MPU6050) != MPU6050_OK) 			//Setting gyroscope range
    {
        return MPU6050_ERROR;
    }

    MPU6050->AccelOffset.X = ACCEL_OFFSET_X;					//Load calibration offsets
    MPU6050->AccelOffset.Y = ACCEL_OFFSET_Y;
    MPU6050->AccelOffset.Z = ACCEL_OFFSET_Z;

    MPU6050->GyroOffset.X  = GYRO_OFFSET_X;
    MPU6050->GyroOffset.Y  = GYRO_OFFSET_Y;
    MPU6050->GyroOffset.Z  = GYRO_OFFSET_Z;

    return MPU6050_OK;
}

uint8_t MPU6050_WHO_AM_I (MPU6050_t *MPU6050) 		//Reading WHO_AM_I register
{
    return Read8(MPU6050, 0x75);
}

MPU6050_STATE_t MPU6050_WakeUp(MPU6050_t *MPU6050)	//Waking up MPU6050 from sleep mode
{
    uint8_t Value = Read8(MPU6050, PWR_MGMT_1);

    Value &= ~(1 << 6);  // disable sleep
    Value &= ~(1 << 5);  // disable cycle
    Value |= (1 << 3);   // disable temperature sensor

    return Write8(MPU6050, PWR_MGMT_1, Value);
}

//Calculates Roll and Pitch angles based on Accelerometer data
MPU6050_STATE_t MPU6050_DegFromAccel(MPU6050_t *MPU6050, float *Roll, float *Pitch)
{
    Data_t Accel;
    MPU6050_ReadAcceleration(MPU6050, &Accel);

    //Angles calculations
    *Roll  = atan2f(Accel.Y, Accel.Z) * 180.0f / M_PI;
    *Pitch = atan2f(-Accel.X, sqrtf(Accel.Y*Accel.Y + Accel.Z*Accel.Z)) * 180.0f / M_PI;

    return MPU6050_OK;
}
//Integrates Gyroscope data to calculate angles (depends on DT)
MPU6050_STATE_t MPU6050_DegFromGyro(MPU6050_t *MPU6050, float *RollG, float *PitchG, float *YawG)
{
    Data_t Gyro;
    MPU6050_ReadGyro(MPU6050, &Gyro);

    *RollG  += (Gyro.X) * DT;
    *PitchG += (Gyro.Y) * DT;
    *YawG   += (Gyro.Z) * DT;

    return MPU6050_OK;
}

//Main function to get filtered angles
MPU6050_STATE_t MPU6050_Angle(MPU6050_t *MPU6050, float *Roll, float *Pitch, float *Yaw)
{
    float RollAccel, PitchAccel;

    //Get accel angles
    MPU6050_DegFromAccel(MPU6050, &RollAccel, &PitchAccel);

    static uint8_t initialized = 0;
    if (!initialized)
    {
        *Roll = RollAccel;
        *Pitch = PitchAccel;
        *Yaw = 0.0f;
        initialized = 1;

        MPU6050_DegFromGyro(MPU6050, Roll, Pitch, Yaw);
        return MPU6050_OK;
    }

    //Get gyro angles
    MPU6050_DegFromGyro(MPU6050, Roll, Pitch, Yaw);

    //Apply filter
    ComplementaryFilter(Roll, Pitch, RollAccel, PitchAccel);

    return MPU6050_OK;
}

//Complementary filter implementation
//Combines high-pass filtered gyro data with low-pass filtered accel data.
static void ComplementaryFilter(float *roll, float *pitch, float roll_accel, float pitch_accel)
{
    const float alpha = 0.98f;
    *roll  = alpha * (*roll)  + (1.0f - alpha) * roll_accel;
    *pitch = alpha * (*pitch) + (1.0f - alpha) * pitch_accel;
}

/* ---------------- PRIVATE HELP FUNCTIONS ---------------- */

// Function to read 8 bits from MPU6050
static uint8_t Read8(MPU6050_t *MPU6050, uint8_t Register)
{
    uint8_t Value;
    HAL_I2C_Mem_Read(MPU6050->hi2c, (MPU6050->address) << 1, Register, 1, &Value, 1, MPU6050_TIMEOUT);
    return Value;
}
// Function to write 8 bits to MPU6050
static MPU6050_STATE_t Write8(MPU6050_t *MPU6050, uint8_t Register, uint8_t Value)
{
    return HAL_I2C_Mem_Write(MPU6050->hi2c, (MPU6050->address) << 1, Register, 1, &Value, 1, MPU6050_TIMEOUT);
}
// Function to read 16 bits from MPU6050
static int16_t Read16(MPU6050_t *MPU6050, uint8_t Register)
{
    uint8_t Value[2];
    HAL_I2C_Mem_Read(MPU6050->hi2c, (MPU6050->address) << 1, Register, 1, Value, 2, MPU6050_TIMEOUT);
    return (int16_t)((Value[0] << 8) | Value[1]);
}
// Setting up gyro range
static MPU6050_STATE_t MPU6050_SetGyroRange(MPU6050_t *MPU6050)
{
    uint8_t RegisterValue = Read8(MPU6050, GYRO_CONFIG);
    RegisterValue &= ~( (1 << 3) | (1 << 4) );
    RegisterValue |= (1 << 3);// ±500°/s
    return Write8(MPU6050, GYRO_CONFIG, RegisterValue);
}
// Setting up accelerometer range
static MPU6050_STATE_t MPU6050_SetAccelerationRange(MPU6050_t *MPU6050)
{
    uint8_t RegisterValue = Read8(MPU6050, ACCEL_CONFIG);
    RegisterValue &= ~( (1 << 4) | (1 << 5) ); // ±2g
    return Write8(MPU6050, ACCEL_CONFIG, RegisterValue);
}

// Read raw data from Accelerometer
static MPU6050_STATE_t MPU6050_ReadAccelerationRaw(MPU6050_t *MPU6050, DataRaw_t *AccelRaw)
{
    uint8_t buf[6] = {0};
    if (HAL_I2C_Mem_Read(MPU6050->hi2c, (MPU6050->address)<<1, ACCEL_XOUT_H, 1, buf, 6, MPU6050_TIMEOUT) != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    AccelRaw->X = (int16_t)((buf[0] << 8) | buf[1]);
    AccelRaw->Y = (int16_t)((buf[2] << 8) | buf[3]);
    AccelRaw->Z = (int16_t)((buf[4] << 8) | buf[5]);
    return MPU6050_OK;
}

static MPU6050_STATE_t MPU6050_ReadAcceleration(MPU6050_t *MPU6050, Data_t *Accelerations)
{
    DataRaw_t Raw = {0};
    MPU6050_ReadAccelerationRaw(MPU6050, &Raw);

    if(MPU6050_ReadAccelerationRaw(MPU6050, &Raw) != MPU6050_OK)
    {
        return MPU6050_ERROR;
    }

    const float ScaleFactor = 16384.0f; // ±2g

    Accelerations->X = (float)(Raw.X - MPU6050->AccelOffset.X) / ScaleFactor;
    Accelerations->Y = (float)(Raw.Y - MPU6050->AccelOffset.Y) / ScaleFactor;
    Accelerations->Z = (float)(Raw.Z - MPU6050->AccelOffset.Z) / ScaleFactor;

    return MPU6050_OK;
}

// Function to calibrate accelerometer
MPU6050_STATE_t MPU6050_CalibrateAccel(MPU6050_t *MPU6050, Data_t *AccelCalibratedValues)
{
    DataRaw_t Accelerations;
    int64_t SumX = 0, SumY = 0, SumZ = 0;
    uint16_t i;
    const uint16_t Samples = 200;
    for(i = 0; i < Samples; i++)
    {
        if (MPU6050_ReadAccelerationRaw(MPU6050, &Accelerations) != MPU6050_OK)
        {
            HAL_Delay(2);
            return MPU6050_ERROR;
        }
        SumX += Accelerations.X;
        SumY += Accelerations.Y;
        SumZ += Accelerations.Z;
        HAL_Delay(1);
    }

    const float ScaleFactor = 16384.0f; // ±2g raw per 1g
    MPU6050->AccelOffset.X = (float)SumX / Samples;
    MPU6050->AccelOffset.Y = (float)SumY / Samples;
    //For Z axis, we subtract the expected 1g (in raw units) to get the offset
    MPU6050->AccelOffset.Z = ((float)SumZ / Samples) - ScaleFactor;

    AccelCalibratedValues->X = (float)SumX / Samples;
    AccelCalibratedValues->Y = (float)SumY / Samples;
    AccelCalibratedValues->Z = ((float)SumZ / Samples) - ScaleFactor;

    return MPU6050_OK;
}
// Read raw data from gyroscope
static MPU6050_STATE_t MPU6050_ReadGyroRaw(MPU6050_t *MPU6050, DataRaw_t *GyroRaw)
{
    uint8_t buf[6] = {0};
    if (HAL_I2C_Mem_Read(MPU6050->hi2c, (MPU6050->address)<<1, GYRO_XOUT_H, 1, buf, 6, MPU6050_TIMEOUT) != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    GyroRaw->X = (int16_t)((buf[0] << 8) | buf[1]);
    GyroRaw->Y = (int16_t)((buf[2] << 8) | buf[3]);
    GyroRaw->Z = (int16_t)((buf[4] << 8) | buf[5]);

    return MPU6050_OK;
}

static MPU6050_STATE_t MPU6050_ReadGyro(MPU6050_t *MPU6050, Data_t *GyroCalculated)
{
    DataRaw_t Raw;
    MPU6050_ReadGyroRaw(MPU6050, &Raw);

    const float ScaleFactor = 65.5f; // ±250°/s

    //see gyro dryf
    //GyroCalculated->X = (float)((Raw.X) / ScaleFactor);
    //GyroCalculated->Y = (float)((Raw.Y) / ScaleFactor);
    //GyroCalculated->Z = (float)((Raw.Z) / ScaleFactor);

    GyroCalculated->X = (float)((Raw.X - MPU6050->GyroOffset.X) / ScaleFactor);
    GyroCalculated->Y = (float)((Raw.Y - MPU6050->GyroOffset.Y) / ScaleFactor);
    GyroCalculated->Z = (float)((Raw.Z - MPU6050->GyroOffset.Z) / ScaleFactor);

    return MPU6050_OK;
}

// Calibrate gyro
MPU6050_STATE_t MPU6050_CalibrateGyro(MPU6050_t *MPU6050, Data_t *GyroCalibratedValues)
{
    DataRaw_t Gyro;
    int64_t SumX = 0, SumY = 0, SumZ = 0;
    uint16_t i;
    const uint16_t Samples = 200;
    for(i = 0; i < Samples; i++)
    {
        if (MPU6050_ReadGyroRaw(MPU6050, &Gyro) != MPU6050_OK)
        {
            HAL_Delay(2);
            return MPU6050_ERROR;
        }
        SumX += Gyro.X;
        SumY += Gyro.Y;
        SumZ += Gyro.Z;
        HAL_Delay(1);
    }
    MPU6050->GyroOffset.X = (float)SumX / Samples;
    MPU6050->GyroOffset.Y = (float)SumY / Samples;
    MPU6050->GyroOffset.Z = (float)SumZ / Samples;

    //Data used to hard Code Values of Gyro CalibratedValues
    GyroCalibratedValues->X = (float)SumX / Samples;
    GyroCalibratedValues->Y = (float)SumY / Samples;
    GyroCalibratedValues->Z = (float)SumZ / Samples;

    return MPU6050_OK;
}
