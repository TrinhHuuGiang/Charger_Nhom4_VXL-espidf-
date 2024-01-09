#include <stdio.h>
#include "freertos/FreeRTOS.h" // tao task freertos
#include "freertos/task.h" //
#include "driver/gpio.h"// driver gpio
#include "sdkconfig.h" // cau hinh menuconfig
#include "esp_log.h" // log thong bao
#include "BTN.h"
#include "Relay.h"
#include "MCP41010.h"

// cac button duoc ket noi tro 4.7k pull up
#define BUTTON_PIN_1 25 //BTN 1 cam vao chan 25
#define BUTTON_PIN_2 26 //BTN 2 cam vao chan 26
#define BUTTON_PIN_3 27 //BTN 3 cam vao chan 27

// relay
#define RELAY_PIN_1 5   //RELAY 1 cam vao chan 5
#define RELAY_PIN_2 17  //RELAY 2 cam vao chan 17
#define RELAY_PIN_3 16  //BTN 1 cam vao chan 16

// co 3 thiet bi dung spi,2 mcp41010 va 1 doc microsd
#define CS_PIN_1 4 // cs cho MCP41010 pin 1
#define CS_PIN_2 2 // cs cho MCP41010 pin 2
#define CS_PIN_3 15 // cs cho microsd cards
#define MOSI_PIN 13// mosi (si) pin
#define MISO_PIN 12 // miso (so) pin
#define SCK_PIN 14// serial clock

// co 3 thiet bi dung i2c

// co 2 thiet bi dung 1 wire

#define TAG "MAIN"

void app_main(void)
{
    button_init(BUTTON_PIN_1,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_2,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_3,GPIO_MODE_INPUT);
    relay_init(RELAY_PIN_1,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_2,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_3,GPIO_MODE_OUTPUT,0);

    MCP_t dev1;
    mcp41010_init(&dev1, KOHMS_MCP41010, 2, MOSI_PIN, SCK_PIN,CS_PIN_1);
    uint8_t cur_value1 = 0;
    uint8_t DIRECTION1 = 1;
    uint8_t potentiometer1 = METER_0;
    MCP_t dev2;
    mcp41010_init(&dev2, KOHMS_MCP41010, 2, MOSI_PIN, SCK_PIN,CS_PIN_2);
    uint8_t cur_value2 = 0;
    uint8_t DIRECTION2 = 1;
    uint8_t potentiometer2 = METER_0;

    while (1)
    {
	MCP41_setWiper(&dev1, cur_value1, potentiometer1);
	float resistance1 = MCP41_getK(&dev1, cur_value1);
	ESP_LOGI(TAG, "cur_value=%d resistance=%f KOhms", cur_value1, resistance1);

    if(DIRECTION1) {
        cur_value1++;
    if(cur_value1 == 255) DIRECTION1 = 0;
    } else {
        cur_value1--;
    if(cur_value1 == 0) DIRECTION1 = 1;
    }


    MCP41_setWiper(&dev2, cur_value2, potentiometer2);
	float resistance2 = MCP41_getK(&dev2, cur_value2);
	ESP_LOGI(TAG, "cur_value=%d resistance=%f KOhms", cur_value2, resistance2);

    if(DIRECTION2) {
        cur_value2++;
    if(cur_value2 == 255) DIRECTION2 = 0;
    } else {
        cur_value2--;
    if(cur_value2 == 0) DIRECTION2 = 1;
    }

    vTaskDelay(50);

    }

#pragma region testrelay, button
    /* hàm test bấm button , relay bật tắt
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
    }*/
    #pragma endregion
}
