#ifndef COMMON_H
#define COMMON_H

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros();
void IRAM_ATTR delay_us(uint32_t us);

#endif