#include "MachineVision/auto_get_best_threshold.h"

uint8_t dajinfa(uint8_t* image, uint8_t width, uint8_t height)
{
  uint8_t best_threshold = 0;
  
  uint8_t x = width;
  uint8_t y = height;
  
  uint16_t pixelcount[GRAYSCALE] = {0};
  float pixelpro[GRAYSCALE] = {0};
  
  float pixelsum = x * y;
  float graysum = 0;
  
  float w0 = 0;
  float w1 = 0;
  float u = 0;
  float u0 = 0;
  float u1 = 0;
  float u0temp = 0;
  float u1temp = 0;
  float sigmatemp = 0;
  float deltmax = 0;
  
  for(uint8_t i = 0; i < y; i++)
  {
    for(uint8_t j = 0; j < x; j++)
    {
      pixelcount[image[i * width + j]] += 1;
      graysum += image[i * width + j];
    }
  }
  
  for(uint16_t k = 0; k < GRAYSCALE; k++)
  {
    pixelpro[k] = (float)pixelcount[k] / pixelsum;
  }
  
  for(uint16_t k = 0; k < GRAYSCALE; k++)
  {
    w0 += pixelpro[k];
    u0temp += k * pixelpro[k];

    w1 = 1 - w0;
    u1temp = graysum /pixelsum - u0temp;

    if(w0 == 0)
      u0 = 0;
    else
      u0 = u0temp / w0;
    
    if(w1 == 0)
      u1 = 0;
    else
      u1 = u1temp / w1;

    u = u0temp + u1temp;

    sigmatemp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);

    if(sigmatemp >= deltmax)
    {
      deltmax = sigmatemp;
      best_threshold = k;
    }
    else
    {
      break;
    }
  }
 
  return best_threshold;
}

void GARY_TO_BINARY(uint8_t* image, uint8_t width, uint8_t height, uint8_t threshold)
{
  for(uint8_t y = 0; y < height; y++)
  {
    for(uint8_t x = 0; x < width; x++)
    {
      if(image[y * width + x] <= threshold)
        image[y * width + x] = 255;
      else 
        image[y * width + x] = 0;
    }
  }
}

void GARY_TO_BINARY_Pro(uint8_t* image, uint8_t width, uint8_t height, uint8_t max_threshold, uint8_t min_threshold)
{
  for(uint8_t y = 0; y < height; y++)
  {
    for(uint8_t x = 0; x < width; x++)
    {
      if(image[y * width + x] <= max_threshold && image[y * width + x] >= min_threshold )
        image[y * width + x] = 255;
      else 
        image[y * width + x] = 0;
    }
  }
}