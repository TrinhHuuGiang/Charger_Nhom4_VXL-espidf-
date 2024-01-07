#include "driver/gpio.h"
#include "Relay.h"

// Khoi tao ket noi relay cho esp32 che do output, level 0 là mac dinh NC_NO, level 1 nguoc lai
void relay_init(gpio_num_t RELAY_PIN, gpio_mode_t GPIO_MODE_OUTPUT, uint32_t LEVEL)
{
    gpio_set_direction(RELAY_PIN,GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, LEVEL);
}

//setlevel
void relay_set_level(gpio_num_t RELAY_PIN, uint32_t LEVEL)
{
    gpio_set_level(RELAY_PIN, LEVEL);
}
