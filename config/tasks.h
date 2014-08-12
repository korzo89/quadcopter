/*
 * tasks.h
 *
 *  Created on: 11 sie 2014
 *      Author: Korzo
 */

#ifndef TASKS_H_
#define TASKS_H_

#include <FreeRTOSConfig.h>

//-----------------------------------------------------------------

#define TASK_NAME(_name)            ((const signed char*)_name)

#define SYSTEM_INIT_TASK_PRIORITY   1
#define RCP_TASK_PRIORITY           1
#define GUI_TASK_PRIORITY           2
#define GPS_TASK_PRIORITY           1
#define CONTROL_TASK_PRIORITY       2
#define IMU_TASK_PRIORITY           2

#define SYSTEM_INIT_TASK_STACK      256
#define RCP_TASK_STACK              200
#define GUI_TASK_STACK              300
#define GPS_TASK_STACK              configMINIMAL_STACK_SIZE
#define CONTROL_TASK_STACK          200
#define IMU_TASK_STACK              300

//-----------------------------------------------------------------

#endif /* TASKS_H_ */
