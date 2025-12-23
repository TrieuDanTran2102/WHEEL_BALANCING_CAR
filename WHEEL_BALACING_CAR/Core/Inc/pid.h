/*
 * pid.h
 *
 *  Created on: Oct 29, 2025
 *      Author: ACER
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define SAMPLE_TIME_DEFAULT 5 // Default sample time in milliseconds
#define GetTime() HAL_GetTick()

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct {
    float Kp;          // Proportional Tuning Constant
    float Ki;          // Integral Tuning Constant
    float Kd;          // Derivative Tuning Constant

    uint32_t SampleTime; // Sample Time in milliseconds
    uint32_t LastTime;
    int16_t Integral_Max;
    int16_t Integral_Min;
    int16_t Output_Max;
    int16_t Output_Min;
    double *MyInput;   // Pointer to the Input variable
    double *MyOutput;  // Pointer to the Output variable
    double *MySetpoint; // Pointer to the Setpoint variable

    double sumError; // Integral sum
    double lastError; // Last input value
}PID_TypeDef;

void PID_Init(PID_TypeDef *PID, double* input, double* output, double* setpoint, float Kp, float Ki, float Kd, uint32_t SampleTime);
void setTunings(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void setSampleTime(PID_TypeDef *PID, uint32_t NewSampleTime);
void setLimit(PID_TypeDef *PID, int16_t Integral_Max, int16_t Integral_Min, int16_t Output_Max, int16_t Output_Min);
void computePID(PID_TypeDef *PID);

#endif /* INC_PID_H_ */
