#include "utils.h"
#include "version.h"

#ifndef FW_VERSION
  #define FW_VERSION "unknown"
#endif

#ifndef JOB_NAME
  #define JOB_NAME "unknown"
#endif

#ifndef BUILD_NUMBER
  #define BUILD_NUMBER "unknown"
#endif

#ifndef GIT_COMMIT
  #define GIT_COMMIT "unknown"
#endif

#ifndef SERVO_VARIANT
  #define SERVO_VARIANT "unknown"
#endif

static const char VERSION_HW[2] = {1, 0}; 

static const char VERSION_FW[2] = {0, 5};

static const char VERSION_REVISION[] =
    "{\"CI\":{\"Version\":\""FW_VERSION"\"," \
    "\"Project\":\""JOB_NAME"\"," \
    "\"Variant\":\""SERVO_VARIANT"\"," \
    "\"Build\":\""BUILD_NUMBER"\"," \
    "\"Git\":\""GIT_COMMIT"\"}}";

/** \brief Get HW revision
 *
 * \return HW revision as uint16_t
 *
 */
uint16_t* VERSION_GetHW(void)
{
  return (uint16_t*)(uintptr_t)VERSION_HW;
}

/** \brief Get FW revision
 *
 * \return FW revision as uint16_t
 *
 */
uint16_t* VERSION_GetFW(void)
{
  return (uint16_t*)(uintptr_t)VERSION_FW;
}

/** \brief Get FW revision as string
 *
 * \param void
 * \return Pointer to FW string (xx.xx)
 *
 */
char* VERSION_GetStringFW(void)
{
  static char str[16];

  (void)strcpy(str, UTILS_IntToStr(VERSION_FW[0], 2));
  (void)strcat(str, ".");
  (void)strcat(str, UTILS_IntToStr(VERSION_FW[1], 2));

  return str;
}

/** \brief Get char from revision string
 *
 * \param [in] pos Position in the string
 * \return Character at position
 *
 */
char VERSION_GetRevisionStr(uint8_t pos)
{
  uint8_t len = (uint8_t)strlen(VERSION_REVISION);
  if (pos == 0xFFU)
  {
    return len;
  }
  if (pos >= len)
  {
    return 0;
  }

  return (uint8_t)VERSION_REVISION[pos];
}
