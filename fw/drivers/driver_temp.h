#ifndef DRV_TEMP_H
#define DRV_TEMP_H

#include "defines.h"
#include "nvmctrl.h"

#define CONF_TSENS_CAL_FCAL ((TSENS_FUSES_FCAL_Msk & (*((uint32_t *)TSENS_FUSES_FCAL_ADDR))) >> TSENS_FUSES_FCAL_Pos)

#define CONF_TSENS_CAL_TCAL ((TSENS_FUSES_TCAL_Msk & (*((uint32_t *)TSENS_FUSES_TCAL_ADDR))) >> TSENS_FUSES_TCAL_Pos)

#define CONF_TSENS_CAL_GAIN_VAL                                                                                        \
    ((TSENS_FUSES_GAIN_0_Msk & (*((uint32_t *)TSENS_FUSES_GAIN_0_ADDR))) >> TSENS_FUSES_GAIN_0_Pos)                    \
      | (((TSENS_FUSES_GAIN_1_Msk & (*((uint32_t *)TSENS_FUSES_GAIN_1_ADDR))) >> TSENS_FUSES_GAIN_1_Pos) << 20)

#define CONF_TSENS_CAL_OFFSET                                                                                          \
    ((TSENS_FUSES_OFFSET_Msk & (*((uint32_t *)TSENS_FUSES_OFFSET_ADDR))) >> TSENS_FUSES_OFFSET_Pos)

#define CONF_TSENS_CAL_GAIN (CONF_TSENS_CAL_GAIN_VAL) * (CONF_CPU_FREQUENCY / 48000000.0f)

void TEMP_Init(void);
float TEMP_GetValue(void);
uint8_t TEMP_GetByteValue(void);

#endif
