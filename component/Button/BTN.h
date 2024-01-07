#ifndef BUTTON
#define BUTTON

#include "driver/gpio.h"

// Khoi tao button cho esp32 che do input
void button_init(gpio_num_t BUTTON_PIN, gpio_mode_t GPIO_MODE_INPUT);

// Kiem tra tin hieu o cong la muc cao hay thap
int button_get_level(gpio_num_t BUTTON_PIN);


#endif