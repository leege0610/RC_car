/*
 * motor.h
 *
 *  Created on: 2022. 2. 17.
 *      Author: LEE
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "controller.h"
//#include "stm32f4xx.h"

void Move(CONTROLLER_SIGNAL _signal);
void Forward();
void Backward();
void Right();
void Left();
void Fr45();
void Fl45();
void Br45();
void Bl45();
void Cw();
void Ccw();
void Stop();
void Throttle();
void MotorInit();

#endif /* INC_MOTOR_H_ */
