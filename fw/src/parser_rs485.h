#ifndef PARSER_RS485_H
#define PARSER_RS485_H

#include "defines.h"

typedef struct
{
  uint8_t cmd;
  uint8_t id;
  uint8_t arg_1;
  uint8_t arg_2;
} TCommData;

void PARSER_ProcessRS485(TCommData *input, TCommData *output);

#endif
