/*
 * motor.c
 *
 *  Created on: 2022. 2. 17.
 *      Author: LEE
 */
#include "motor.h"
//#include "stm32f4xx.h"

extern int16_t throttle;
extern int16_t duty2;
extern int16_t preduty2;
extern TIM_HandleTypeDef htim4;

void Move(CONTROLLER_SIGNAL _signal)
{
	if(_signal != STOP) Throttle();

	switch(_signal)
	{
	case FORWARD:
		Forward();
		break;
	case BACKWARD:
		Backward();
		break;
	case RIGHT:
		Right();
		break;
	case LEFT:
		Left();
		break;
	case FR45:
		Fr45();
		break;
	case FL45:
		Fl45();
		break;
	case BR45:
		Br45();
		break;
	case BL45:
		Bl45();
		break;
	case CW:
		Cw();
		break;
	case CCW:
		Ccw();
		break;
	case STOP:
		Stop();
		break;
	default:
		Stop();
		break;
	}
}

void Throttle()
{
	if(duty2 > 500 && duty2 < 1000)
	{
		throttle = (duty2-500)*2;
	}

	TIM4->CCR1 = throttle;
	TIM4->CCR2 = throttle;
	TIM4->CCR3 = throttle;
	TIM4->CCR4 = throttle;
}
/*
void Throttle()
{
	int16_t tmpDuty;

	tmpDuty = TIM2->CCR4;

	if(tmpDuty > 1000)
	{
		if(duty2 <= 500) tmpDuty = 500;
		else if(duty2 > 500 && duty2 < 1000) tmpDuty = duty2;
		else tmpDuty = 1000;
	}
	else if(tmpDuty < 500) tmpDuty = 500;
	//if(((duty2 < tmpDuty) && (tmpDuty - duty2 < 100)) || ((duty2 > tmpDuty) && (duty2 - tmpDuty < 100)) )

	duty2 = tmpDuty;
	TIM4->CCR1 = (duty2-500)*2;
	TIM4->CCR2 = (duty2-500)*2;
	TIM4->CCR3 = (duty2-500)*2;
	TIM4->CCR4 = (duty2-500)*2;
}
*/
void MotorInit()
{
	preduty2 = duty2;
}

void Forward()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Backward()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Right()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Left()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Fr45()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);  //RR_CW_CCW

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void Fl45()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}


void Br45()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Bl45()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);  //RR_CW_CCW

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void Cw()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Ccw()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET);  //FR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET);  //FL_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);  //RR_CW_CCW
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET);  //RL_CW_CCW

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Stop()
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}
