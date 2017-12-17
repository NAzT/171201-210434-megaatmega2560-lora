#include <Arduino.h>

typedef struct __attribute((__packed__)) {
  uint8_t type; 
  uint8_t seq1; 
  uint8_t seq2; 
  uint32_t field1;
  uint32_t field2; 
} CMMC_SENSOR_T;