/*
 * pid.h
 *
 *  Created on: Oct 29, 2025
 *      Author: ACER
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define OUTPUT_MIN 0
#define OUTPUT_MAX 0

#define SAMPLE_TIME_DEFAULT 100 // Default sample time in milliseconds
#define GetTime() HAL_GetTick()

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef enum {
    _PID_ON_DIRECT = 0,
    _PID_ON_REVERSE = 1
}PIDCD_TypeDef;

typedef struct {
    PIDCD_TypeDef Direction;

    float Kp;          // Proportional Tuning Constant
    float Ki;          // Integral Tuning Constant
    float Kd;          // Derivative Tuning Constant

    uint32_t SampleTime; // Sample Time in milliseconds
    uint32_t LastTime;

    double *MyInput;   // Pointer to the Input variable
    double *MyOutput;  // Pointer to the Output variable
    double *MySetpoint; // Pointer to the Setpoint variable

    double sumError; // Integral sum
    double lastError; // Last input value
}PID_TypeDef;

void PID_Init(PID_TypeDef *PID, double* input, double* output, double* setpoint, float Kp, float Ki, float Kd, PIDCD_TypeDef Direction);
void setDirection(PID_TypeDef *PID, PIDCD_TypeDef Direction);
void setTunings(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void setSampleTime(PID_TypeDef *PID, uint32_t NewSampleTime);

void computePID(PID_TypeDef *PID);

#endif /* INC_PID_H_ */
