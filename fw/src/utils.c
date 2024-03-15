#include "utils.h"

#define STR_MAX_LEN   (16U - 1U)

/** \brief Get length of the string
 *
 * \param [in] str Pointer to string
 * \return Length of string as uint8_t
 *
 */
uint8_t UTILS_GetLength(char *str)
{
  uint8_t i = 0;

  while (i < STRINGS_MAX_LEN)
  {
    if (((uint8_t)str[i] == 0U) || ((uint8_t)str[i] == 0xFFU))
    {
      break;
    }
    i++;
  }

  return i;
}

/** \brief Get sign of the integer value
 *
 * \param [in] x Integer value
 * \return =1 for positive, =-1 for negative, 0 otherwise
 *
 */
int8_t UTILS_Sign(int32_t x)
{
    if (x > 0)
    {
        return 1;
    }
    if (x < 0)
    {
        return -1;
    }
    return 0;
}

/** \brief Set limitations to signal
 *
 * \param [in] in Input signal value
 * \param [in] limit Limitation value
 * \return Output signal with applied limitation as int16_t
 *
 */
int16_t UTILS_LimitValue(int32_t in, uint16_t limit)
{
  int16_t result = (int16_t)in;

  if (in > (int32_t)limit)
  {
    result = limit;
  }
  if (in < -(int32_t)limit)
  {
    result = -limit;
  }

  return result;
}

/** \brief Simple int to string converter
 *
 * \param [in] value UINT32 value
 * \param [in] min_len Minimal length (zero-padded)
 * \return char* Pointer to a resulting string
 *
 */
char *UTILS_IntToStr(uint32_t value, uint8_t min_len)
{
  static char b[STR_MAX_LEN + 1U];
  uint8_t i;
  char ch;

  i = 0U;
  while (value != 0U)
  {
    b[i] = (uint8_t)(value % 10U) + '0';
    i++;
    value = value / 10U;
    if (i >= STR_MAX_LEN)
    {
      break;
    }
  }

  while ((i < min_len) && (i < STR_MAX_LEN))
  {
    b[i] = '0';
    i++;
  }
  b[i] = 0U;

  /**< Reverse the string */
  int start = 0;
  int stop = (int)i - 1;
  while (start < stop)
  {
    ch = b[start];
    b[start] = b[stop];
    b[stop] = ch;
    start++;
    stop--;
  }

  return b;
}

/** \brief Simple float to string converter
 *
 * \param [in] value Float value
 * \param [in] pos Number of positions after decimal point
 * \return char* Pointer to a resulting string
 *
 */
char *UTILS_FloatToStr(float value, uint8_t pos)
{
  static char b[STR_MAX_LEN + 1U];
  uint32_t uval32;
  uint32_t mul = 1;
  uint8_t i;
  char ch;

  for (i = 0U; i < pos; i++)
  {
    mul *= 10U;
  }
  uval32 = (uint32_t)(value * (float)mul);

  i = 0U;
  while (uval32 != 0U)
  {
    b[i] = (uint8_t)(uval32 % 10U) + '0';
    i++;
    uval32 = uval32 / 10U;
    if (i == pos)
    {
      b[i] = '.';
      i++;
    }
    if (i >= STR_MAX_LEN)
    {
      break;
    }
  }

  if (i == 0U)
  {
    b[i] = '0';
    i++;
    b[i] = '.';
    i++;
  }
  if (b[i - 1U] == '.')
  {
    b[i] = '0';
    i++;
  }
  b[i] = 0U;

  /**< Reverse the string */
  int start = 0;
  int stop = (int)i - 1;
  while (start < stop)
  {
    ch = b[start];
    b[start] = b[stop];
    b[stop] = ch;
    start++;
    stop--;
  }

  return b;
}
