#ifndef NOTCH_H
#define NOTCH_H

#include "defines.h"

//#define NOTCH_FREQ              5.681f
//#define NOTCH_WIDTH	            0.015f      // Notch Filter relative width (app. 0.015)
//#define NOTCH_ATTENUATION	      10		      // Notch Filter attenuation (app.  10.0)
#define NOTCH_DT                0.001F	    // Servo loop sampling rate (0.001)

#define NOTCH_FACTOR            10000000L
#define NOTCH_IN_MULT           10000

#define NOTCH_FILTER_MAX_DEPTH  30U

enum {
  NOTCH_NUM_1,
  NOTCH_NUM_2,
  NOTCH_NUM_LAST
};

/**< Define type for calculations to allow simple switching */
typedef int64_t     notch_t;

void NOTCH_GetCoefficients(uint8_t num, notch_t *c1, notch_t *c2, notch_t *c3, notch_t *c4, notch_t *c5);
void NOTCH_RecalcCoeff(uint8_t num, uint16_t freq, uint8_t width, uint8_t depth);
int32_t NOTCH_Calculate(uint8_t num, int32_t x);

#endif
