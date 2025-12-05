
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include <stdint.h>
// MPU6050 structure
extern uint32_t timer;
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    double *myOutput;
} MPU6050_t;


// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


void mpu6050_init(MPU6050_t *DataStruct, double *output);
void mpu6050_read_Accel(MPU6050_t *DataStruct);
void mpu6050_read_Gyro(MPU6050_t *DataStruct);
void mpu6050_read_Temp(MPU6050_t *DataStruct);
void mpu6050_read_All(MPU6050_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void mpu6050_calibrate(MPU6050_t *DataStruct);
void I2C_ResetBus(void);
void MPU6050_HardReset(void);
#endif /* INC_MPU6050_H_ */
