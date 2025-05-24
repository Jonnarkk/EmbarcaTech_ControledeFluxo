#ifndef BUZZER_H
#define  BUZZER_H

#include <stdio.h>
#include "pico/stdlib.h"

void buzz(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration);

#endif