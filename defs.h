/*
 * defs.h
 *
 *  Created on: 12-05-2014
 *      Author: Korzo
 */

#ifndef DEFS_H_
#define DEFS_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

//-----------------------------------------------------------------

#define PACK_STRUCT     __attribute__((packed))

//-----------------------------------------------------------------

typedef enum
{
	RES_OK,
	RES_ERR_BAD_PARAM,
	RES_ERR_FATAL,
	RES_ERR_IO
} result_t;

//-----------------------------------------------------------------

#endif /* DEFS_H_ */
