#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "BTN.h"
#include "Relay.h"

// cac button duoc ket noi tro 4.7k pull up
#define BUTTON_PIN_1 25 //BTN 1 cam vao chan 25
#define BUTTON_PIN_2 26 //BTN 2 cam vao chan 26
#define BUTTON_PIN_3 27 //BTN 3 cam vao chan 27
#define RELAY_PIN_1 5   //RELAY 1 cam vao chan 5
#define RELAY_PIN_2 17  //RELAY 2 cam vao chan 17
#define RELAY_PIN_3 16  //BTN 1 cam vao chan 16

void app_main(void)
{
    button_init(BUTTON_PIN_1,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_2,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_3,GPIO_MODE_INPUT);
    relay_init(RELAY_PIN_1,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_2,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_3,GPIO_MODE_OUTPUT,0);
    while (1)
    {
        if(button_get_level(BUTTON_PIN_1)==1) //0 nhan nut
        relay_set_level(RELAY_PIN_1, 0); // mac dinh
        else
        relay_set_level(RELAY_PIN_1, 1);

        if(button_get_level(BUTTON_PIN_2)==1)
        relay_set_level(RELAY_PIN_2, 0);
        else
        relay_set_level(RELAY_PIN_2, 1);

        if(button_get_level(BUTTON_PIN_3)==1)
        relay_set_level(RELAY_PIN_3, 0);
        else
        relay_set_level(RELAY_PIN_3, 1);

        vTaskDelay(1);
    }
}
