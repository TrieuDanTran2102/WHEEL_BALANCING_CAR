/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: ACER
 */

#include "motor.h"

void Motor1_Forward(void)
{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void Motor2_Forward(void)
{
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}

void Motor1_Reverse(void)
{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}

void Motor2_Reverse(void)
{
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void Motor1_Stop(void)
{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void Motor2_Stop(void)
{
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void Motor1_SetSpeed(uint16_t percent)
{
	if (percent > PERCENT_MAX) percent = PERCENT_MAX;
	if (percent < PERCENT_MIN) percent = PERCENT_MIN;

	uint16_t speed = percent * PWM_MAX / 100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

void Motor2_SetSpeed(uint16_t percent)
{
	if (percent > PERCENT_MAX) percent = PERCENT_MAX;
	if (percent < PERCENT_MIN) percent = PERCENT_MIN;

	uint16_t speed = percent * PWM_MAX / 100;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
}

Motor parseMotor(const char *str)
{
	if (!strcmp(str, "M1")) return M1;
	if (!strcmp(str, "M2")) return M2;
	if (!strcmp(str, "ALL")) return ALL;
	return -1;
}

Direction parseDirection(const char *str)
{
	if (!strcmp(str, "F")) return F;
	if (!strcmp(str, "R")) return R;
	if (!strcmp(str, "S")) return S;
	return -1;
}

void Motor_Controller(Motor Motor, Direction Direction, uint16_t percent)
{
	if (percent > 80) percent = 80;

	switch(Motor)
	{
		case M1:
			if (Direction == S) {
				Motor1_Stop();
				break;
			}
			if (Direction == F) {
				Motor1_Forward();
			}
			else {
				Motor1_Reverse();
			}
			Motor1_SetSpeed(percent);
			break;
		case M2:
			if (Direction == S) {
				Motor2_Stop();
				break;
			}
			if (Direction == F) {
				Motor2_Forward();
			}
			else  {
				Motor2_Reverse();
			}
			Motor2_SetSpeed(percent);
			break;
		case ALL:
			if (Direction == S) {
			    Motor1_Stop();
			    Motor2_Stop();
				break;
			}
			if (Direction == F) {
			    Motor1_Forward();
			    Motor2_Forward();
			}
			else {
			    Motor1_Reverse();
			    Motor2_Reverse();
			}
		    Motor1_SetSpeed(percent);
		    Motor2_SetSpeed(percent);
			break;
		default:
			printf("Control Motor Error!\r\n");
			break;
	}
}

void Motor_Init(Motor_TypeDef *MT, double* Rotational_Speed)
{
	MT->Rotational_Speed = Rotational_Speed;
}

void Motor_CalculateVelocity(Motor_TypeDef *MT, TIM_HandleTypeDef *htim)
{
	int32_t encoder_cnt = __HAL_TIM_GET_COUNTER(htim);
	int32_t delta_cnt = encoder_cnt - MT->encoder_cnt_pre;

	// xử lý overflow 16-bit
	if (delta_cnt > 32767) delta_cnt -= 65536;
	if (delta_cnt < -32768) delta_cnt += 65536;

	MT->encoder_cnt_pre = encoder_cnt;

	// lọc nhiễu bằng Low-pass filter
	float alpha = 0.1;
	*MT->Rotational_Speed = ((1 - alpha) * (*MT->Rotational_Speed) + alpha * delta_cnt);

//	*MT->Rotational_Speed = (int)((1 - alpha) * (*MT->Rotational_Speed) + alpha * delta_cnt);

//	*MT->Rotational_Speed = delta_cnt;
}

void Motor_getPercent(Motor_TypeDef *MT, double PID_Output)
{
	MT->dir = PID_Output > 0 ? F : R;
//	MT->per = tanh(0.01 * fabs(PID_Output)) * PERCENT_MAX;

	MT->per = fabs(PID_Output) > PERCENT_MAX ? PERCENT_MAX : fabs(PID_Output);
}
