/*
 * controller.h
 *
 *  Created on: 2022. 2. 17.
 *      Author: LEE
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

typedef enum {
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT,
	FR45,
	FL45,
	BR45,
	BL45,
	CW,
	CCW,
	STOP
}CONTROLLER_SIGNAL;

CONTROLLER_SIGNAL Control();

#endif /* INC_CONTROLLER_H_ */
