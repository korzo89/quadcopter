/*
 * ultrasonic.h
 *
 *  Created on: 07-05-2014
 *      Author: Korzo
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

//-----------------------------------------------------------------

#define US_MAX_DIST     200

//-----------------------------------------------------------------

void ultrasonicConfig(void);

float ultrasonicGetDistance(void);

//-----------------------------------------------------------------

#endif /* ULTRASONIC_H_ */
