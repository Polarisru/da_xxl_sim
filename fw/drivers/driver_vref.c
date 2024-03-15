#include "driver_vref.h"

void VREF_Init(uint8_t source)
{
  SUPC->VREF.reg = SUPC_VREF_SEL(source) | SUPC_VREF_VREFOE;
}
