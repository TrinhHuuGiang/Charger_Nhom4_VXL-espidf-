#include <stdio.h>
#include "freertos/FreeRTOS.h" // tao task freertos
#include "freertos/task.h" //
#include "driver/gpio.h"// driver gpio
#include "sdkconfig.h" // cau hinh menuconfig
#include "esp_log.h" // log thong bao
#include "BTN.h"
#include "Relay.h"
#include "MCP41010.h"
#include "LCD1602.h"
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
    MCP_t dev1;
    uint8_t cur_value1 = 0;
    uint8_t DIRECTION1 = 1;
    uint8_t potentiometer1 = METER_0;
    MCP_t dev2;
    uint8_t cur_value2 = 0;
    uint8_t DIRECTION2 = 1;
    uint8_t potentiometer2 = METER_0;

// co 3 thiet bi dung i2c
#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2

// co 2 thiet bi dung 1 wire

#define TAG "MAIN"

void Button_Relay_Demotask(void* param);
void MCP_DemoTask(void* param);
void LCD_DemoTask(void* param);

void app_main(void)
{
    //btn_relay
    button_init(BUTTON_PIN_1,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_2,GPIO_MODE_INPUT);
    button_init(BUTTON_PIN_3,GPIO_MODE_INPUT);
    relay_init(RELAY_PIN_1,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_2,GPIO_MODE_OUTPUT,0);
    relay_init(RELAY_PIN_3,GPIO_MODE_OUTPUT,0);

    //mcp41010

    mcp41010_init(&dev1, KOHMS_MCP41010, 2, MOSI_PIN, SCK_PIN,CS_PIN_1);

    mcp41010_init(&dev2, KOHMS_MCP41010, 2, MOSI_PIN, SCK_PIN,CS_PIN_2);

    //lcd
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(&Button_Relay_Demotask, "dm bt_rl Task", 2048, NULL, 6, NULL);
    xTaskCreate(&MCP_DemoTask, "dm mcp Task", 2048, NULL, 5, NULL);
    xTaskCreate(&LCD_DemoTask, "dm LCD Task", 2048, NULL, 5, NULL);
}


void Button_Relay_Demotask(void* param)
{
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

        vTaskDelay(10/portTICK_RATE_MS);
    }
}

void MCP_DemoTask(void* param)
{
    while (1)
    {
	MCP41010_setWiper(&dev1, cur_value1, potentiometer1);
	float resistance1 = MCP41010_getK(&dev1, cur_value1);
	ESP_LOGI(TAG, "cur_value_1=%d resistance_1=%f KOhms", cur_value1, resistance1);

    if(DIRECTION1) {
        cur_value1++;
    if(cur_value1 == 255) DIRECTION1 = 0;
    } else {
        cur_value1--;
    if(cur_value1 == 0) DIRECTION1 = 1;
    }


    MCP41010_setWiper(&dev2, cur_value2, potentiometer2);
	float resistance2 = MCP41010_getK(&dev2, cur_value2);
	ESP_LOGI(TAG, "cur_value_2=%d resistance_2=%f KOhms\n", cur_value2, resistance2);

    if(DIRECTION2) {
        cur_value2++;
    if(cur_value2 == 255) DIRECTION2 = 0;
    } else {
        cur_value2--;
    if(cur_value2 == 0) DIRECTION2 = 1;
    }

    vTaskDelay(1000/portTICK_RATE_MS);

    }
}

void LCD_DemoTask(void* param)
{
    char num[20];
    while (true) {
        LCD_home();
        LCD_clearScreen();
        LCD_writeStr("16x2 I2C LCD");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        LCD_writeStr("Lets Count 0-10!");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        for (int i = 0; i <= 10; i++) {
            LCD_setCursor(8, 1);
            sprintf(num, "%d", i);
            LCD_writeStr(num);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
  
    }
}



