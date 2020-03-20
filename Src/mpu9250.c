/*
 * MPU9250.c
 *
 *  Created on: Mar 18, 2020
 *      Author: David Perna
 */
/*
MPU9250.cpp
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

#include "MPU9250.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdlib.h>
#include "stdbool.h"

// i2c
uint8_t _address = MPU9250_ADDRESS;
//TwoWire *_i2c;
const uint32_t _i2cRate = 400000; // 400 kHz
size_t _numBytes; // number of bytes received from I2C

// track success of interacting with sensor
int _status;

// buffer for reading from sensor
uint8_t _buffer[21];

// data counts
int16_t _axcounts,_aycounts,_azcounts;
int16_t _gxcounts,_gycounts,_gzcounts;
int16_t _hxcounts,_hycounts,_hzcounts;
int16_t _tcounts;

// data buffer
float _ax, _ay, _az;
float _gx, _gy, _gz;
float _hx, _hy, _hz;
float _t;

// wake on motion
uint8_t _womThreshold;

// scale factors
float _accelScale;
float _gyroScale;
float _magScaleX, _magScaleY, _magScaleZ;
const float _tempScale = 333.87f;
const float _tempOffset = 21.0f;

// configuration
AccelRange _accelRange;
GyroRange _gyroRange;
DlpfBandwidth _bandwidth;
uint8_t _srd;

// gyro bias estimation
size_t _numSamples = 100;
double _gxbD, _gybD, _gzbD;
float _gxb, _gyb, _gzb;

// accel bias and scale factor estimation
double _axbD, _aybD, _azbD;
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;
float _axb, _ayb, _azb;
float _axs = 1.0f;
float _ays = 1.0f;
float _azs = 1.0f;

// magnetometer bias and scale factor estimation
uint16_t _maxCounts = 1000;
float _deltaThresh = 0.3f;
uint8_t _coeff = 8;
uint16_t _counter;
float _framedelta, _delta;
float _hxfilt, _hyfilt, _hzfilt;
float _hxmax, _hymax, _hzmax;
float _hxmin, _hymin, _hzmin;
float _hxb, _hyb, _hzb;
float _hxs = 1.0f;
float _hys = 1.0f;
float _hzs = 1.0f;
float _avgs;

// transformation matrix
/* transform the accel and gyro axes to match the magnetometer axes */
const int16_t tX[3] = {0,  1,  0};
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};
// constants
const float G = 9.807f;
const float _d2r = 3.14159265359f/180.0f;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_184 = 0x01;
const uint8_t ACCEL_DLPF_92 = 0x02;
const uint8_t ACCEL_DLPF_41 = 0x03;
const uint8_t ACCEL_DLPF_20 = 0x04;
const uint8_t ACCEL_DLPF_10 = 0x05;
const uint8_t ACCEL_DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_184 = 0x01;
const uint8_t GYRO_DLPF_92 = 0x02;
const uint8_t GYRO_DLPF_41 = 0x03;
const uint8_t GYRO_DLPF_20 = 0x04;
const uint8_t GYRO_DLPF_10 = 0x05;
const uint8_t GYRO_DLPF_5 = 0x06;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

// fifo
bool _enFifoAccel,_enFifoGyro,_enFifoMag,_enFifoTemp;
size_t _fifoSize,_fifoFrameSize;
float _axFifo[85], _ayFifo[85], _azFifo[85];
size_t _aSize;
float _gxFifo[85], _gyFifo[85], _gzFifo[85];
size_t _gSize;
float _hxFifo[73], _hyFifo[73], _hzFifo[73];
size_t _hSize;
float _tFifo[256];
size_t _tSize;


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* starts communication with the MPU-9250 */
int MPU9250_begin()
{
  // select clock source to gyro
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -1;
  }
  // enable I2C master mode
  if(MPU9250_PRIV_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(MPU9250_PRIV_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  MPU9250_PRIV_writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(1);
  // reset the AK8963
  MPU9250_PRIV_writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to gyro
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((MPU9250_PRIV_whoAmI() != 113)&&(MPU9250_PRIV_whoAmI() != 115)){
    return -5;
  }
  // enable accelerometer and gyro
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }
  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(MPU9250_PRIV_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
    return -8;
  }
  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 184Hz as default
  if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){
    return -9;
  }
  if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
    return -10;
  }
  _bandwidth = DLPF_BANDWIDTH_184HZ;
  // setting the sample rate divider to 0 as default
  if(MPU9250_PRIV_writeRegister(SMPDIV,0x00) < 0){
    return -11;
  }
  _srd = 0;
  // enable I2C master mode
  if(MPU9250_PRIV_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	return -12;
  }
	// set the I2C bus speed to 400 kHz
	if( MPU9250_PRIV_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( MPU9250_PRIV_whoAmIAK8963() != 72 ){
    return -14;
	}
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  MPU9250_PRIV_readAK8963Registers(AK8963_ASA,3,_buffer);
  _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  // set AK8963 to Power Down
  if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -19;
  }
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  MPU9250_PRIV_readAK8963Registers(AK8963_HXL,7,_buffer);
  // estimate gyro bias
  if (MPU9250_calibrateGyro() < 0) {
    return -20;
  }
  if (MPU9250_calibrateAccel() < 0) {
      return -21;
  }
  if (MPU9250_calibrateMag() < 0) {
      return -22;
  }

  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU9250_setAccelRange(AccelRange range) {
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
        return -1;
      }
      _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
      break;
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
        return -1;
      }
      _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
        return -1;
      }
      _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -1;
      }
      _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
      break;
    }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU9250_setGyroRange(GyroRange range) {
  switch(range) {
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(MPU9250_PRIV_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
        return -1;
      }
      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(MPU9250_PRIV_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
        return -1;
      }
      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
      break;
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(MPU9250_PRIV_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
        return -1;
      }
      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(MPU9250_PRIV_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -1;
      }
      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  _gyroRange = range;
  return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU9250_setDlpfBandwidth(DlpfBandwidth bandwidth) {
  switch(bandwidth) {
    case DLPF_BANDWIDTH_184HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0){ // setting accel bandwidth to 92Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 92Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0){ // setting accel bandwidth to 41Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ // setting accel bandwidth to 20Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0){ // setting accel bandwidth to 10Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0){ // setting accel bandwidth to 5Hz
        return -1;
      }
      if(MPU9250_PRIV_writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
        return -2;
      }
      break;
    }
  }
  _bandwidth = bandwidth;
  return 1;
}

/* sets the sample rate divider to values other than default */
int MPU9250_setSrd(uint8_t srd) {
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if(MPU9250_PRIV_writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
    return -1;
  }
  if(srd > 9){
    // set AK8963 to Power Down
    if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    MPU9250_PRIV_readAK8963Registers(AK8963_HXL,7,_buffer);
  } else {
    // set AK8963 to Power Down
    if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    MPU9250_PRIV_readAK8963Registers(AK8963_HXL,7,_buffer);
  }
  /* setting the sample rate divider */
  if(MPU9250_PRIV_writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
    return -4;
  }
  _srd = srd;
  return 1;
}

/* enables the data ready interrupt */
int MPU9250_enableDataReadyInterrupt() {
  /* setting the interrupt */
  if (MPU9250_PRIV_writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
    return -1;
  }
  if (MPU9250_PRIV_writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int MPU9250_disableDataReadyInterrupt() {
  if(MPU9250_PRIV_writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  }
  return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU9250_enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr) {
  // set AK8963 to Power Down
  MPU9250_PRIV_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  MPU9250_PRIV_writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(1);
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_1,0x00) < 0){ // cycle 0, sleep 0, standby 0
    return -1;
  }
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_2,DIS_GYRO) < 0){ // disable gyro measurements
    return -2;
  }
  if(MPU9250_PRIV_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
    return -3;
  }
  if(MPU9250_PRIV_writeRegister(INT_ENABLE,INT_WOM_EN) < 0){ // enabling interrupt to wake on motion
    return -4;
  }
  if(MPU9250_PRIV_writeRegister(MOT_DETECT_CTRL,(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0){ // enabling accel hardware intelligence
    return -5;
  }
  _womThreshold = map(womThresh_mg, 0, 1020, 0, 255);
  if(MPU9250_PRIV_writeRegister(WOM_THR,_womThreshold) < 0){ // setting wake on motion threshold
    return -6;
  }
  if(MPU9250_PRIV_writeRegister(LP_ACCEL_ODR,(uint8_t)odr) < 0){ // set frequency of wakeup
    return -7;
  }
  if(MPU9250_PRIV_writeRegister(PWR_MGMNT_1,PWR_CYCLE) < 0){ // switch to accel low power mode
    return -8;
  }
  return 1;
}

/* configures and enables the FIFO buffer  */
int MPU9250FIFO_enableFifo(bool accel,bool gyro,bool mag,bool temp) {
  if(MPU9250_PRIV_writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0){
    return -1;
  }
  if(MPU9250_PRIV_writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(mag*FIFO_MAG)|(temp*FIFO_TEMP)) < 0){
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoMag = mag;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + mag*7 + temp*2;
  return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int MPU9250_readSensor() {
  // grab the data from the MPU9250
  if (MPU9250_PRIV_readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
    return -1;
  }

  return MPU9250_convertRawData(_buffer);
}

int MPU9250_convertRawData(uint8_t *raw_data)
{
	// combine into 16 bit values
    _axcounts = (((int16_t)raw_data[0]) << 8) | raw_data[1];
    _aycounts = (((int16_t)raw_data[2]) << 8) | raw_data[3];
    _azcounts = (((int16_t)raw_data[4]) << 8) | raw_data[5];
    _tcounts = (((int16_t)raw_data[6]) << 8) | raw_data[7];
    _gxcounts = (((int16_t)raw_data[8]) << 8) | raw_data[9];
    _gycounts = (((int16_t)raw_data[10]) << 8) | raw_data[11];
    _gzcounts = (((int16_t)raw_data[12]) << 8) | raw_data[13];
    _hxcounts = (((int16_t)raw_data[15]) << 8) | raw_data[14];
    _hycounts = (((int16_t)raw_data[17]) << 8) | raw_data[16];
    _hzcounts = (((int16_t)raw_data[19]) << 8) | raw_data[18];
    // transform and convert to float values
    _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
    _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
    _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
    _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
    _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
    _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
    _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
    _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
    _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
    _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
	return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float MPU9250_getAccelX_mss() {
  return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float MPU9250_getAccelY_mss() {
  return _ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float MPU9250_getAccelZ_mss() {
  return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float MPU9250_getGyroX_rads() {
  return _gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float MPU9250_getGyroY_rads() {
  return _gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float MPU9250_getGyroZ_rads() {
  return _gz;
}

/* returns the magnetometer measurement in the x direction, uT */
float MPU9250_getMagX_uT() {
  return _hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float MPU9250_getMagY_uT() {
  return _hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float MPU9250_getMagZ_uT() {
  return _hz;
}

/* returns the die temperature, C */
float MPU9250_getTemperature_C() {
  return _t;
}

/* reads data from the MPU9250 FIFO and stores in buffer */
int MPU9250FIFO_readFifo() {
  // get the fifo size
  MPU9250_PRIV_readRegisters(FIFO_COUNT, 2, _buffer);
  _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the MPU9250
    if (MPU9250_PRIV_readRegisters(FIFO_READ,_fifoFrameSize,_buffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
      _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
      _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
      // transform and convert to float values
      _axFifo[i] = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale)-_axb)*_axs;
      _ayFifo[i] = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale)-_ayb)*_ays;
      _azFifo[i] = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale)-_azb)*_azs;
      _aSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
      // transform and convert to float values
      _tFifo[i] = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      _gxcounts = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      _gycounts = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      _gzcounts = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to float values
      _gxFifo[i] = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
      _gyFifo[i] = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
      _gzFifo[i] = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
      _gSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoMag) {
      // combine into 16 bit values
      _hxcounts = (((int16_t)_buffer[1 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[0 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      _hycounts = (((int16_t)_buffer[3 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[2 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      _hzcounts = (((int16_t)_buffer[5 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[4 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      // transform and convert to float values
      _hxFifo[i] = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
      _hyFifo[i] = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
      _hzFifo[i] = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
      _hSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO_getFifoAccelX_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO_getFifoAccelY_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO_getFifoAccelZ_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void MPU9250FIFO_getFifoGyroX_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void MPU9250FIFO_getFifoGyroY_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void MPU9250FIFO_getFifoGyroZ_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO_getFifoMagX_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hxFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO_getFifoMagY_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hyFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO_getFifoMagZ_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hzFifo,_hSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO_getFifoTemperature_C(size_t *size,float* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(float));
}

/* estimates the gyro biases */
int MPU9250_calibrateGyro() {
  // set the range, bandwidth, and srd
  if (MPU9250_setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (MPU9250_setSrd(19) < 0) {
    return -3;
  }

  // take samples and find bias
  _gxbD = 0;
  _gybD = 0;
  _gzbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
	  MPU9250_readSensor();
    _gxbD += (MPU9250_getGyroX_rads() + _gxb)/((double)_numSamples);
    _gybD += (MPU9250_getGyroY_rads() + _gyb)/((double)_numSamples);
    _gzbD += (MPU9250_getGyroZ_rads() + _gzb)/((double)_numSamples);
    HAL_Delay(20);
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;

  // set the range, bandwidth, and srd back to what they were
  if (MPU9250_setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (MPU9250_setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (MPU9250_setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float MPU9250_getGyroBiasX_rads() {
  return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float MPU9250_getGyroBiasY_rads() {
  return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float MPU9250_getGyroBiasZ_rads() {
  return _gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void MPU9250_setGyroBiasX_rads(float bias) {
  _gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void MPU9250_setGyroBiasY_rads(float bias) {
  _gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void MPU9250_setGyroBiasZ_rads(float bias) {
  _gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int MPU9250_calibrateAccel() {
  // set the range, bandwidth, and srd
  if (MPU9250_setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  if (MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (MPU9250_setSrd(19) < 0) {
    return -3;
  }

  // take samples and find min / max
  _axbD = 0;
  _aybD = 0;
  _azbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
	  MPU9250_readSensor();
    _axbD += (MPU9250_getAccelX_mss()/_axs + _axb)/((double)_numSamples);
    _aybD += (MPU9250_getAccelY_mss()/_ays + _ayb)/((double)_numSamples);
    _azbD += (MPU9250_getAccelZ_mss()/_azs + _azb)/((double)_numSamples);
    HAL_Delay(20);
  }
  if (_axbD > 9.0f) {
    _axmax = (float)_axbD;
  }
  if (_aybD > 9.0f) {
    _aymax = (float)_aybD;
  }
  if (_azbD > 9.0f) {
    _azmax = (float)_azbD;
  }
  if (_axbD < -9.0f) {
    _axmin = (float)_axbD;
  }
  if (_aybD < -9.0f) {
    _aymin = (float)_aybD;
  }
  if (_azbD < -9.0f) {
    _azmin = (float)_azbD;
  }

  // find bias and scale factor
  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_aymin + _aymax) / 2.0f;
    _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (MPU9250_setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (MPU9250_setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (MPU9250_setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float MPU9250_getAccelBiasX_mss() {
  return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float MPU9250_getAccelScaleFactorX() {
  return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float MPU9250_getAccelBiasY_mss() {
  return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float MPU9250_getAccelScaleFactorY() {
  return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float MPU9250_getAccelBiasZ_mss() {
  return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float MPU9250_getAccelScaleFactorZ() {
  return _azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void MPU9250_setAccelCalX(float bias,float scaleFactor) {
  _axb = bias;
  _axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void MPU9250_setAccelCalY(float bias,float scaleFactor) {
  _ayb = bias;
  _ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void MPU9250_setAccelCalZ(float bias,float scaleFactor) {
  _azb = bias;
  _azs = scaleFactor;
}

/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
int MPU9250_calibrateMag() {
  // set the srd
  if (MPU9250_setSrd(19) < 0) {
    return -1;
  }

  // get a starting set of data
  MPU9250_readSensor();
  _hxmax = MPU9250_getMagX_uT();
  _hxmin = MPU9250_getMagX_uT();
  _hymax = MPU9250_getMagY_uT();
  _hymin = MPU9250_getMagY_uT();
  _hzmax = MPU9250_getMagZ_uT();
  _hzmin = MPU9250_getMagZ_uT();

  // collect data to find max / min in each channel
  _counter = 0;
  while (_counter < _maxCounts) {
    _delta = 0.0f;
    _framedelta = 0.0f;
    MPU9250_readSensor();
    _hxfilt = (_hxfilt*((float)_coeff-1)+(MPU9250_getMagX_uT()/_hxs+_hxb))/((float)_coeff);
    _hyfilt = (_hyfilt*((float)_coeff-1)+(MPU9250_getMagY_uT()/_hys+_hyb))/((float)_coeff);
    _hzfilt = (_hzfilt*((float)_coeff-1)+(MPU9250_getMagZ_uT()/_hzs+_hzb))/((float)_coeff);
    if (_hxfilt > _hxmax) {
      _delta = _hxfilt - _hxmax;
      _hxmax = _hxfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hyfilt > _hymax) {
      _delta = _hyfilt - _hymax;
      _hymax = _hyfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hzfilt > _hzmax) {
      _delta = _hzfilt - _hzmax;
      _hzmax = _hzfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hxfilt < _hxmin) {
      _delta = abs(_hxfilt - _hxmin);
      _hxmin = _hxfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hyfilt < _hymin) {
      _delta = abs(_hyfilt - _hymin);
      _hymin = _hyfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hzfilt < _hzmin) {
      _delta = abs(_hzfilt - _hzmin);
      _hzmin = _hzfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_framedelta > _deltaThresh) {
      _counter = 0;
    } else {
      _counter++;
    }
    HAL_Delay(20);
  }

  // find the magnetometer bias
  _hxb = (_hxmax + _hxmin) / 2.0f;
  _hyb = (_hymax + _hymin) / 2.0f;
  _hzb = (_hzmax + _hzmin) / 2.0f;

  // find the magnetometer scale factor
  _hxs = (_hxmax - _hxmin) / 2.0f;
  _hys = (_hymax - _hymin) / 2.0f;
  _hzs = (_hzmax - _hzmin) / 2.0f;
  _avgs = (_hxs + _hys + _hzs) / 3.0f;
  _hxs = _avgs/_hxs;
  _hys = _avgs/_hys;
  _hzs = _avgs/_hzs;

  // set the srd back to what it was
  if (MPU9250_setSrd(_srd) < 0) {
    return -2;
  }
  return 1;
}

/* returns the magnetometer bias in the X direction, uT */
float MPU9250_getMagBiasX_uT() {
  return _hxb;
}

/* returns the magnetometer scale factor in the X direction */
float MPU9250_getMagScaleFactorX() {
  return _hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float MPU9250_getMagBiasY_uT() {
  return _hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float MPU9250_getMagScaleFactorY() {
  return _hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float MPU9250_getMagBiasZ_uT() {
  return _hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float MPU9250_getMagScaleFactorZ() {
  return _hzs;
}

/* sets the magnetometer bias (uT) and scale factor in the X direction */
void MPU9250_setMagCalX(float bias,float scaleFactor) {
  _hxb = bias;
  _hxs = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Y direction */
void MPU9250_setMagCalY(float bias,float scaleFactor) {
  _hyb = bias;
  _hys = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Z direction */
void MPU9250_setMagCalZ(float bias,float scaleFactor) {
  _hzb = bias;
  _hzs = scaleFactor;
}

/* writes a byte to MPU9250 register given a register address and data */
int MPU9250_PRIV_writeRegister(uint8_t subAddress, uint8_t data){

  uint8_t temp_buffer[1] = {data};
  if(HAL_I2C_Mem_Write(&hi2c1, _address<<1, subAddress, 1, temp_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	  return -1;

  /* read back the register */
  MPU9250_PRIV_readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int MPU9250_PRIV_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if(HAL_I2C_Mem_Read(&hi2c1, _address<<1, subAddress, 1 , dest, count, HAL_MAX_DELAY) == HAL_OK)
	  return 1;
  else
	  return -1;
}

/* writes a register to the AK8963 given a register address and data */
int MPU9250_PRIV_writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // store the data for write
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_DO,data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
	// read the register and confirm
	if (MPU9250_PRIV_readAK8963Registers(subAddress,1,_buffer) < 0) {
    return -5;
  }
	if(_buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}

/* reads registers from the AK8963 */
int MPU9250_PRIV_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
	if (MPU9250_PRIV_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
    return -3;
  }
	HAL_Delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	_status = MPU9250_PRIV_readRegisters(EXT_SENS_DATA_00,count,dest);
  return _status;
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int MPU9250_PRIV_whoAmI(){
  // read the WHO AM I register
  if (MPU9250_PRIV_readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int MPU9250_PRIV_whoAmIAK8963(){
  // read the WHO AM I register
  if (MPU9250_PRIV_readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}


