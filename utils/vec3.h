/*
 * vec3.h
 *
 *  Created on: 31 maj 2014
 *      Author: Korzo
 */

#ifndef VEC3_H_
#define VEC3_H_

//-----------------------------------------------------------------

typedef struct
{
    float x, y, z;
} vec3_t;

//-----------------------------------------------------------------

#define POW2(x)                     ( (x) * (x) )

#define VEC3_NEW(xx, yy, zz)        (vec3_t){ .x = (xx), .y = (yy), .z = (zz) }

#define VEC3_MULT_NEW(v, s)         (vec3_t){ .x = v.x * s, .y = v.y * s, .z = v.z * s }
#define VEC3_DIV_NEW(v, s)          (vec3_t){ .x = v.x / s, .y = v.y / s, .z = v.z / s }
#define VEC3_SUM_NEW(a, b)          (vec3_t){ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z }
#define VEC3_DIFF_NEW(a, b)         (vec3_t){ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z }

#define VEC3_MULT(v, s)             { v.x *= s; v.y *= s; v.z *= s; }
#define VEC3_DIV(v, s)              { v.x /= s; v.y /= s; v.z /= s; }
#define VEC3_SUM(a, b)              { a.x += b.x; a.y += b.y; a.z += b.z; }
#define VEC3_DIFF(a, b)             { a.x -= b.x; a.y -= b.y; a.z -= b.z; }

#define VEC3_DOT(a, b)              ( a.x * b.x + a.y * b.y + a.z * b.z )
#define VEC3_CROSS(a, b)            (vec3_t){                     \
                                       .x = a.y * b.z - b.y * a.z, \
                                       .y = a.z * b.x - b.z * a.x, \
                                       .z = a.x * b.y - b.x * a.y }

#define VEC3_INV_NORM(v)            ( inv_sqrt(POW2(v.x) + POW2(v.y) + POW2(v.z)) )
#define VEC3_NORM(v)                ( 1.0f / VEC3_INV_NORM(v) )
#define VEC3_NORMALIZE(v)           { float inv_norm = VEC3_INV_NORM(v); \
                                      VEC3_MULT(v, inv_norm); }
#define VEC3_NORMALIZE_VAR(v, f)    { f = VEC3_INV_NORM(v); \
                                      VEC3_MULT(v, f); }

//-----------------------------------------------------------------

float inv_sqrt(float x);

//-----------------------------------------------------------------

#endif /* VEC3_H_ */
