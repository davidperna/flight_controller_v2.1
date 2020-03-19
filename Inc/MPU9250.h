/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_H_
#define MPU9250_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define MPU9250_ADDRESS 0x68

typedef enum
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
}GyroRange;

typedef enum
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
}AccelRange;

typedef enum
{
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
}DlpfBandwidth;

typedef enum
{
  LP_ACCEL_ODR_0_24HZ = 0,
  LP_ACCEL_ODR_0_49HZ = 1,
  LP_ACCEL_ODR_0_98HZ = 2,
  LP_ACCEL_ODR_1_95HZ = 3,
  LP_ACCEL_ODR_3_91HZ = 4,
  LP_ACCEL_ODR_7_81HZ = 5,
  LP_ACCEL_ODR_15_63HZ = 6,
  LP_ACCEL_ODR_31_25HZ = 7,
  LP_ACCEL_ODR_62_50HZ = 8,
  LP_ACCEL_ODR_125HZ = 9,
  LP_ACCEL_ODR_250HZ = 10,
  LP_ACCEL_ODR_500HZ = 11
}LpAccelOdr;

long map(long x, long in_min, long in_max, long out_min, long out_max);
int MPU9250_begin();
int MPU9250_setAccelRange(AccelRange range);
int MPU9250_setGyroRange(GyroRange range);
int MPU9250_setDlpfBandwidth(DlpfBandwidth bandwidth);
int MPU9250_setSrd(uint8_t srd);
int MPU9250_enableDataReadyInterrupt();
int MPU9250_disableDataReadyInterrupt();
int MPU9250_enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr);
int MPU9250_readSensor();
int MPU9250_convertRawData(uint8_t *raw_data);
float MPU9250_getAccelX_mss();
float MPU9250_getAccelY_mss();
float MPU9250_getAccelZ_mss();
float MPU9250_getGyroX_rads();
float MPU9250_getGyroY_rads();
float MPU9250_getGyroZ_rads();
float MPU9250_getMagX_uT();
float MPU9250_getMagY_uT();
float MPU9250_getMagZ_uT();
float MPU9250_getTemperature_C();

int MPU9250_calibrateGyro();
float MPU9250_getGyroBiasX_rads();
float MPU9250_getGyroBiasY_rads();
float MPU9250_getGyroBiasZ_rads();
void MPU9250_setGyroBiasX_rads(float bias);
void MPU9250_setGyroBiasY_rads(float bias);
void MPU9250_setGyroBiasZ_rads(float bias);
int MPU9250_calibrateAccel();
float MPU9250_getAccelBiasX_mss();
float MPU9250_getAccelScaleFactorX();
float MPU9250_getAccelBiasY_mss();
float MPU9250_getAccelScaleFactorY();
float MPU9250_getAccelBiasZ_mss();
float MPU9250_getAccelScaleFactorZ();
void MPU9250_setAccelCalX(float bias,float scaleFactor);
void MPU9250_setAccelCalY(float bias,float scaleFactor);
void MPU9250_setAccelCalZ(float bias,float scaleFactor);
int MPU9250_calibrateMag();
float MPU9250_getMagBiasX_uT();
float MPU9250_getMagScaleFactorX();
float MPU9250_getMagBiasY_uT();
float MPU9250_getMagScaleFactorY();
float MPU9250_getMagBiasZ_uT();
float MPU9250_getMagScaleFactorZ();
void MPU9250_setMagCalX(float bias,float scaleFactor);
void MPU9250_setMagCalY(float bias,float scaleFactor);
void MPU9250_setMagCalZ(float bias,float scaleFactor);

// private functions
int MPU9250_PRIV_writeRegister(uint8_t subAddress, uint8_t data);
int MPU9250_PRIV_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
int MPU9250_PRIV_writeAK8963Register(uint8_t subAddress, uint8_t data);
int MPU9250_PRIV_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
int MPU9250_PRIV_whoAmI();
int MPU9250_PRIV_whoAmIAK8963();


// FIFO functions
int MPU9250FIFO_enableFifo(bool accel,bool gyro,bool mag,bool temp);
int MPU9250FIFO_readFifo();
void MPU9250FIFO_getFifoAccelX_mss(size_t *size,float* data);
void MPU9250FIFO_getFifoAccelY_mss(size_t *size,float* data);
void MPU9250FIFO_getFifoAccelZ_mss(size_t *size,float* data);
void MPU9250FIFO_getFifoGyroX_rads(size_t *size,float* data);
void MPU9250FIFO_getFifoGyroY_rads(size_t *size,float* data);
void MPU9250FIFO_(size_t *size,float* data);
void MPU9250FIFO_getFifoMagX_uT(size_t *size,float* data);
void MPU9250FIFO_getFifoMagY_uT(size_t *size,float* data);
void MPU9250FIFO_getFifoMagZ_uT(size_t *size,float* data);
void MPU9250FIFO_getFifoTemperature_C(size_t *size,float* data);



#endif /* MPU9250_H_ */
