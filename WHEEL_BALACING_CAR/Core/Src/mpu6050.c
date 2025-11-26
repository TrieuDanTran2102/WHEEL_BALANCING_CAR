/*
 * mpu6050.c
 *
 *  Created on: Oct 23, 2025
 *      Author: Windows
 */
#include <mpu6050.h>
#include <math.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c1;

uint8_t device[]="the device is ready  \n";
uint8_t devicenot[]="the device is not ready . Check cable \n";
uint8_t write[]="Writing to register 27   \n";
uint8_t writenot[]="Failed writing to register \n";
uint8_t READY[]=" SMPLRT_DIV_REG READY!!!! \n";
uint8_t READYnot[]=" SMPLRT_DIV_REG NOOOOO READY!!!! \n";
uint8_t READY1[]=" ACCEL_CONFIG_REG READY!!!! \n";
uint8_t READYnot1[]=" ACCEL_CONFIG_REG NOOOOO READY!!!! \n";
uint8_t READY2[]=" GYRO_CONFIG_REG READY!!!! \n";
uint8_t READYnot2[]=" GYRO_CONFIG_REG NOOOOO READY!!!! \n";
#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75     //Dùng để kiểm tra ID của cảm biến (giá trị mặc định 0x68).
#define PWR_MGMT_1_REG 0x6B   //Thanh ghi quản lý nguồn, dùng để đánh thức hoặc reset cảm biến.
#define SMPLRT_DIV_REG 0x19   //Chia tần số mẫu (Sample Rate Divider).
#define ACCEL_CONFIG_REG 0x1C  // Cấu hình thang đo (±2g, ±4g, ±8g, ±16g).
#define ACCEL_XOUT_H_REG 0x3B //Byte cao của dữ liệu gia tốc trục X. (Liền sau là XOUT_L, YOUT_H/L, ZOUT_H/L).
#define TEMP_OUT_H_REG 0x41   //Byte cao dữ liệu nhiệt độ.
#define GYRO_CONFIG_REG 0x1B  //Cấu hình thang đo của con quay hồi chuyển (±250, 500, 1000, 2000°/s).
#define GYRO_XOUT_H_REG 0x43  //	Byte cao của dữ liệu con quay hồi chuyển trục X.


// SETUP 6050
uint8_t check_global;
double readroll,readpick;
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;  //Hằng số hiệu chỉnh giá trị gia tốc trục Z.

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void mpu6050_init(MPU6050_t *DataStruct, double *output)
{
	DataStruct->myOutput = output;
	uint8_t check; //dùng để nhận giá trị ID cảm biến (đọc từ thanh ghi WHO_AM_I).
	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, i2c_timeout);
	check_global = check;
	if(check==104)
		{
			Data=0;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

			Data=0x07;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);


			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

		}
		else{

		}


}

void mpu6050_read_Accel(MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6 ,i2c_timeout);
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void mpu6050_read_Gyro(MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];

	//Read 6 byte

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

	DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


	DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void mpu6050_read_Temp(MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[2];
	int16_t temp;

	//Read 2 bytes of data starting from TEMP_OUT_H_REG register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1 , Rec_Data, 2, i2c_timeout);

	temp=(int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void mpu6050_read_All(MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[14];
	int16_t temp;

	//READ 14 bytes
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1 , Rec_Data, 14, i2c_timeout );

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer)/1000;
    timer= HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
    		DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if(roll_sqrt != 0.0)
    {
    	roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;

    }
    else
    {
    	roll=0.0;
    }

    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
    	KalmanY.angle = pitch;
    	DataStruct->KalmanAngleY = pitch;
    }
    else
    {
    	DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
    {
            DataStruct->Gx = -DataStruct->Gx;
    }
    readroll = roll;
    readpick=pitch;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    *DataStruct->myOutput = DataStruct->KalmanAngleX;
}


//void mpu6050_init()
//{
//	  HAL_StatusTypeDef ret =  HAL_I2C_IsDeviceReady(&hi2c1,  (0b1101000 << 1) + 0, 1, 100);
//	  if (ret == HAL_OK)
//	  {
//		  HAL_UART_Transmit(&huart2, device, sizeof(device), 100 );
//	  }
//	  else{
//		  HAL_UART_Transmit(&huart2, devicenot, sizeof(devicenot), 100 );
//	  }
//	  uint8_t temp_data =0b00001000;
//	  HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(&hi2c1, (0b1101000 << 1) + 0, 27, 1, &temp_data , 1 , 100);
//	  if (ret1 == HAL_OK)
//	  {
//		  HAL_UART_Transmit(&huart2, write, sizeof(write), 100 );
//	  }
//	  else{
//		  HAL_UART_Transmit(&huart2, writenot, sizeof(writenot), 100 );
//	  }
//
//}

