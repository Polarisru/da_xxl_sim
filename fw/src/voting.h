#ifndef VOTING_H
#define VOTING_H

#include "defines.h"

#define VOTING_CNT_MAX    100U

enum {
	VOTING_OK,
	VOTING_OK_ACE2ACE,
	VOTING_OK_ACE2VOT,
	VOTING_OK_ACEVBOTH,
	VOTING_ERROR
};

uint16_t VOTING_GetDist(uint16_t p1, uint16_t p2);
uint8_t VOTING_Process(void);

#endif
