#ifndef DRV_FLASH_H
#define DRV_FLASH_H

#include "defines.h"

#ifndef FLASH_PAGE_SIZE
  #define FLASH_PAGE_SIZE   64U
#endif

#define FLASH_ROW_SIZE      (uint16_t)(FLASH_PAGE_SIZE * 4U)

void FLASH_EraseRow(uint32_t *dst);
void FLASH_EraseRowEE(uint32_t *dst);
void FLASH_EraseFull(uint32_t *start_address);
void FLASH_Lock(void);
void FLASH_WriteWords(uint32_t *dst, uint32_t *src, uint32_t n_words);
void FLASH_WriteWordsEE(uint32_t *dst, uint32_t *src, uint32_t n_words);

#endif
