/*
 * pid.c
 *
 *  Created on: Oct 29, 2025
 *      Author: ACER
 */

#include "PID.h"

void PID_Init(PID_TypeDef *PID, double* input, double* output, double* setpoint, float Kp, float Ki, float Kd, uint32_t SampleTime)
{
    PID->MyInput = input;
    PID->MyOutput = output;
    PID->MySetpoint = setpoint;

    setSampleTime(PID, SampleTime);
    setTunings(PID, Kp, Ki, Kd);

    PID->LastTime = GetTime() - PID->SampleTime;
}

void setSampleTime(PID_TypeDef *PID, uint32_t NewSampleTime){
    PID->SampleTime = NewSampleTime;
}

void setTunings(PID_TypeDef *PID, float Kp, float Ki, float Kd){
    float SampleTimeInSec = (float)PID->SampleTime / 1000;

    PID->Kp = Kp;
    PID->Ki = Ki * SampleTimeInSec;
    PID->Kd = Kd / SampleTimeInSec;
}

void setLimit(PID_TypeDef *PID, int16_t Integral_Max, int16_t Integral_Min, int16_t Output_Max, int16_t Output_Min)
{
    PID->Integral_Max = Integral_Max;
    PID->Integral_Min = Integral_Min;
    PID->Output_Max = Output_Max;
    PID->Output_Min = Output_Min;
}
void computePID(PID_TypeDef *PID){
    uint32_t now = GetTime();
    uint32_t timeChange = now - PID->LastTime;

    if (timeChange >= PID->SampleTime){
		double input = *(PID->MyInput);
		double setpoint = *(PID->MySetpoint);
		double error = setpoint - input;

        PID->sumError += (double)(PID->Ki * error);
        if (PID->sumError > PID->Integral_Max) PID->sumError = PID->Integral_Max;
        if (PID->sumError < PID->Integral_Min) PID->sumError = PID->Integral_Min;

        double dInput = (double)(error - PID->lastError);

        double output = (double)(PID->Kp * error + PID->sumError + PID->Kd * dInput);

        *PID->MyOutput = output;
        PID->lastError = error;
        PID->LastTime = now;
    }
}
