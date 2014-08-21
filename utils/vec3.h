/*
 * vec3.h
 *
 *  Created on: 31 maj 2014
 *      Author: Korzo
 */

#ifndef VEC3_H_
#define VEC3_H_

//-----------------------------------------------------------------

struct vec3
{
    float x;
    float y;
    float z;
};

//-----------------------------------------------------------------

#define POW2(_x)                     ((_x) * (_x))

#define VEC3_NEW(_x, _y, _z)        (struct vec3){ .x = (_x), .y = (_y), .z = (_z) }

#define VEC3_MULT_NEW(_v, _s)         (struct vec3){ .x = _v.x * _s, .y = _v.y * _s, .z = _v.z * _s }
#define VEC3_DIV_NEW(_v, _s)          (struct vec3){ .x = _v.x / _s, .y = _v.y / _s, .z = _v.z / _s }
#define VEC3_SUM_NEW(_a, _b)          (struct vec3){ .x = _a.x + _b.x, .y = _a.y + _b.y, .z = _a.z + _b.z }
#define VEC3_DIFF_NEW(_a, _b)         (struct vec3){ .x = _a.x - _b.x, .y = _a.y - _b.y, .z = _a.z - _b.z }

#define VEC3_MULT(_v, _s)             do { _v.x *= _s; _v.y *= _s; _v.z *= _s; } while(0)
#define VEC3_DIV(_v, _s)              do { _v.x /= _s; _v.y /= _s; _v.z /= _s; } while(0)
#define VEC3_SUM(_a, _b)              do { _a.x += _b.x; _a.y += _b.y; _a.z += _b.z; } while(0)
#define VEC3_DIFF(_a, _b)             do { _a.x -= _b.x; _a.y -= _b.y; _a.z -= _b.z; } while(0)

#define VEC3_DOT(_a, _b)                (_a.x * _b.x + _a.y * _b.y + _a.z * _b.z)
#define VEC3_CROSS(_a, _b)              (struct vec3){                          \
                                            .x = _a.y * _b.z - _b.y * _a.z,     \
                                            .y = _a.z * _b.x - _b.z * _a.x,     \
                                            .z = _a.x * _b.y - _b.x * _a.y }

#define VEC3_INV_NORM(_v)               (inv_sqrt(POW2(_v.x) + POW2(_v.y) + POW2(_v.z)))
#define VEC3_NORM(_v)                   (1.0f / VEC3_INV_NORM(_v))
#define VEC3_NORMALIZE(_v)              do { float inv_norm = VEC3_INV_NORM(v); \
                                        VEC3_MULT(v, inv_norm); }while(0)
#define VEC3_NORMALIZE_VAR(_v, _f)      do { _f = VEC3_INV_NORM(_v);    \
                                        VEC3_MULT(_v, _f); } while(0)

//-----------------------------------------------------------------

float inv_sqrt(float x);

//-----------------------------------------------------------------

#endif /* VEC3_H_ */
