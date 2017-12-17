#include <Arduino.h>

typedef struct __attribute((__packed__)) {
  uint8_t type; 
  uint16_t seq; 
  uint32_t field1;
  uint32_t field2; 
} CMMC_SENSOR_T;