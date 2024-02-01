#ifndef LIB_SRC_UTIL_H_
#define LIB_SRC_UTIL_H_

/* function-like MACROS */
#define MIN(a, b) ((a) < (b) ? (a) : (b))  //< get the minumum of two values
#define MAX(a, b) ((a) > (b) ? (a) : (b))  //< get the maximum of two values
#define CLIP(v, _min, _max) (MAX(MIN(v, _max), _min))  //< clamp v between
                                                       //< _min and _max
#define ABS(a) ((a) < 0 ? (-a) : (a))
#endif  // LIB_SRC_UTIL_H_
