/*
 * motor.h
 *
 *  Created on: Nov 11, 2025
 *      Author: ACER
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#define CPR 44
#define RATE 45
#define MINUTE 60

#define RPM_MAX 110
#define PWM_MAX 1000
#define PERCENT_MAX 100
#define PERCENT_MIN 0
#define DELTA_MAX 80

typedef enum {M1, M2, ALL} Motor;
typedef enum {F, R, S} Direction;

typedef struct {
	volatile short encoder_cnt_pre;
	volatile int16_t per;
	volatile uint16_t dir;
	double *Rotational_Speed;
}Motor_TypeDef;

void Motor1_Forward(void);
void Motor2_Forward(void);

void Motor1_Reverse(void);
void Motor2_Reverse(void);

void Motor1_Stop(void);
void Motor2_Stop(void);

void Motor1_SetSpeed(uint16_t percent);
void Motor2_SetSpeed(uint16_t percent);

void Motor_Controller(Motor, Direction, uint16_t percent);

void Motor_CalculateVelocity(Motor_TypeDef *MT, TIM_HandleTypeDef *htim);
void Motor_getPercent(Motor_TypeDef *MT, double PID_Output);

Motor parseMotor(const char *str);
Direction parseDirection(const char *str);


#endif /* INC_MOTOR_H_ */
