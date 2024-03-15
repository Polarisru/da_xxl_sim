#ifndef HDC1080_H
#define HDC1080_H

#include "defines.h"

#define HDC1080_ADDRESS             0x68

#define HDC1080_BAUDRATE            200000UL

#define HDC1080_REG_TEMPERATURE     0x00
#define HDC1080_REG_HUMIDITY        0x01
#define HDC1080_REG_CONFIGURATION   0x02
#define HDC1080_REG_SERIAL1         0xFB
#define HDC1080_REG_SERIAL2         0xFC
#define HDC1080_REG_SERIAL3         0xFD
#define HDC1080_REG_MANUFACTURER    0xFE
#define HDC1080_REG_DEVICEID        0xFF

#define HDC1080_SIGN_MANUFACTURER   0x5449
#define HDC1080_SIGN_DEVICEID       0x1050

#ifndef DEF_UNITTEST
  #define HDC1080_CHANNEL             SERCOM1
#endif

#ifndef DEF_UNITTEST
void HDC1080_Configuration(Sercom *channel);
#endif
float HDC1080_ReadTemperature(void);
float HDC1080_ReadHumidity(void);

#endif
