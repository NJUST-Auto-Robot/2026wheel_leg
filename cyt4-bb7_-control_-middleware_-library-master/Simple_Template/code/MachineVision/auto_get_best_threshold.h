#ifndef _AUTO_GET_BEST_THRESHOLD_H_
#define _AUTO_GET_BEST_THRESHOLD_H_

#include "zf_common_typedef.h"

#define GRAYSCALE 256

uint8_t dajinfa(uint8_t* image, uint8_t width, uint8_t height);
void GARY_TO_BINARY(uint8_t* image, uint8_t width, uint8_t height, uint8_t threshold);
void GARY_TO_BINARY_Pro(uint8_t* image, uint8_t width, uint8_t height, uint8_t min_threshold, uint8_t max_threshold);


#endif 
