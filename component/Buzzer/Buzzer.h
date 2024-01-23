#ifndef BUZZER
#define BUZZER
#include <sys/types.h>

// khoi tao timer cho buzzer
void Buzzer_init(uint8_t BUZZER_PIN);

// thay doi duty circle cho do to cua am thanh
void Buzzer_set_duty(uint8_t level);
#endif