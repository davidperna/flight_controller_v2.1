
//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================

#ifndef MAHONYAHRS_H_
#define MAHONYAHRS_H_
#include <math.h>

void Mahony_begin(float sampleFrequency);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float Mahony_getRoll();
float Mahony_getPitch();
float Mahony_getYaw();
float Mahony_getRollRadians();
float Mahony_getPitchRadians();
float Mahony_getYawRadians();

static float Mahony_invSqrt(float x);
void Mahony_computeAngles();

#endif /* MAHONYAHRS_H_ */
