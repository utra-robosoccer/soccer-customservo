#include <math.h>

#define min(a, b) (a < b) ? a : b
#define max(a, b) (a < b) ? b : a
#define sub_uint8(a, b) min(a - (int8_t)b, (uint8_t)(a - b))
#define sub_uint16(a, b) min(a - (int16_t)b, (uint16_t)(a - b))