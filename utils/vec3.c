/*
 * vec3.c
 *
 *  Created on: 31 maj 2014
 *      Author: Korzo
 */

#include "vec3.h"

//-----------------------------------------------------------------

float inv_sqrt(float x)
{
    unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
    float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}
