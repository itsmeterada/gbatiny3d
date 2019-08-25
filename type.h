#ifndef TYPE_H
#define TYPE_H

#include <stdint.h>

// ----------------------------------------------
// defines
// ----------------------------------------------
#define PSHIFT           14
#define PRES             (1 << PSHIFT)    // 16384
#define PROUNDBIT        (1 << (PSHIFT-1))

// ----------------------------------------------
// typedef
// ----------------------------------------------

typedef int8_t   INT8;
typedef uint8_t  UINT8;
typedef int16_t   INT16;
typedef uint16_t  UINT16;
typedef int32_t   INT32;
typedef uint32_t  UINT32;
/*
typedef int64_t   INT64;
typedef uint64_t  UINT64;
*/

typedef struct {
  INT32 x, y, z;
} Vector3;

typedef struct {
  INT16 x, y, z;
} Vector3i;

// fixed point identity matrix
typedef struct {
  INT32 m[4][4] = {
      {PRES,    0,    0,    0},
      {   0, PRES,    0,    0},
      {   0,    0, PRES,    0},
      {   0,    0,    0, PRES}
  };
} Matrix4;

// ----------------------------------------------
// functions
// ----------------------------------------------
// fixed point multiplication
static long pMultiply(INT32 x, INT32 y) {
  return ( (x * y) + PROUNDBIT) >> PSHIFT;
}

#endif // TYPE_H
