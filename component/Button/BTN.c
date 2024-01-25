#include <sys/types.h> // uint8_t...
#include "driver/gpio.h"// driver gpio
#include "BTN.h"


// Khoi tao button cho esp32 che do input
void button_init(uint8_t BUTTON_PIN, uint8_t GPIO_MODE_INPUT)
{
    // khai bao chan se su dung lam gpio cho nut bam
    gpio_pad_select_gpio(BUTTON_PIN);

    // khai bao chuc nang input
    gpio_set_direction(BUTTON_PIN,GPIO_MODE_INPUT);

    // enable pullup 
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_pullup_en(BUTTON_PIN);

    /*!<GPIO_INTR_POSEDGE: GPIO interrupt type : rising edge                  */
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE);
}

// Kiem tra tin hieu o cong la muc cao hay thap
int button_get_level(uint8_t BUTTON_PIN)
{
    return gpio_get_level(BUTTON_PIN);
}
