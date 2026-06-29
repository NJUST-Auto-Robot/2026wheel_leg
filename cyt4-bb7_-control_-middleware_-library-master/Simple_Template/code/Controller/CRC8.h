#ifndef _CRC8_H
#define _CRC8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t _lut[256];

extern void Crc8_init(uint8_t poly); // poly为crc校验常数  0xD5
extern uint8_t Crc8_calc(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif