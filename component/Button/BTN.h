#ifndef BUTTON
#define BUTTON
#include <sys/types.h>

// Khoi tao button cho esp32 che do input
void button_init(uint8_t BUTTON_PIN, uint8_t GPIO_MODE_INPUT);

// Kiem tra tin hieu o cong la muc cao hay thap
int button_get_level(uint8_t BUTTON_PIN);


#endif