#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmocka.h"

#include "monitor.h"

void FLASH_EraseRowEE(uint32_t *dst)
{
  (void) dst;
}

void FLASH_WriteWordsEE(uint32_t *dst, uint32_t *src, uint32_t n_words)
{
  (void) dst;
  (void) src;
  (void) n_words;
}

void SPI_Select(uint8_t dummy)
{
  (void) dummy;
}

uint16_t SPI_Receive(uint8_t bits)
{
  (void) bits;
  return mock_type(uint16_t);
}

uint8_t HALL_ReadValues(void)
{
  return mock_type(uint8_t);
}

bool I2C_ReadReg(uint8_t channel, uint8_t addr, uint8_t reg, uint8_t *rx_data, uint8_t rx_len)
{
  //check_expected(channel);
  //check_expected(addr);
  //check_expected(reg);
  //check_expected(rx_data);
  //check_expected(rx_len);
  (void) channel;
  (void) addr;
  (void) reg;
  (void) rx_data;
  (void) rx_len;

  for (uint8_t i = 0; i < rx_len; i++)
    *rx_data++ = mock_type(uint8_t);

  return true;
}

//uint8_t MONITOR_GetCurrent(void)
//{
//  return mock_type(uint8_t);
//}
//
//uint8_t MONITOR_GetVoltage(uint8_t channel)
//{
//  if (channel == MONITOR_U_CH1)
//    return 0x81;
//  else
//    return 0x91;
//}
//
//uint8_t MONITOR_GetTemperature(uint8_t channel)
//{
////  if (channel == MONITOR_TEMP_MOTOR)
////    return 0x80;
////  else
////    return 0x90;
//  (void) channel;
//  return mock_type(uint8_t);
//}
//
//uint8_t MONITOR_GetMaxCurrent(void)
//{
//  return 0xA0;
//}

void MOTOR_DoTest(uint8_t *resp_1, uint8_t *resp_2)
{
  *resp_1 = mock_type(uint8_t);
  *resp_2 = mock_type(uint8_t);
}

void DRV8320_DoBrake(bool on)
{
  (void) on;
}

void DRV8320_DoFree(bool on)
{
  (void) on;
}

void DRV8320_SetDir(bool dir)
{
  (void) dir;
}

bool GPIO_GetPin(uint8_t port, uint8_t pin)
{
  (void) port;
  (void) pin;

  return mock_type(bool);
}

void GPIO_TogglePin(uint8_t port, uint8_t pin)
{
  (void) port;
  (void) pin;
}

void HEARTBEAT_Enable(void)
{

}

void HEARTBEAT_Disable(void)
{

}
