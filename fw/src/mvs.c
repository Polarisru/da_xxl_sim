#include "defines.h"
#include "mvs.h"

int16_t MVS_Voting(int16_t a, int16_t b, int16_t c)
{
  uint8_t res = 0;

  if (a >= b)
  {
    res += 1U;
  }
  if (a >= c)
  {
    res += 2U;
  }
  if (b >= c)
  {
    res += 4U;
  }

  switch (res)
  {
    case 1:
    case 6:
      return a;
    case 0:
    case 7:
      return b;
    default:
      return c;
  }
}
