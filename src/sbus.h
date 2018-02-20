#ifndef __SBUS_H
#define __SBUS_H

#include <stdint.h>

extern uint16_t _channels[18];
extern uint32_t _lostFrames;
extern uint32_t frames;
extern uint8_t _failsafe;

uint8_t sbus_receive(uint8_t b);

#endif
