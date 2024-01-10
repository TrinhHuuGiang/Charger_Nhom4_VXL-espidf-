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

// co 3 thiet bi dung i2c
#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2

// co 2 thiet bi dung 1 wire

#define TAG "MAIN"



void app_main(void)
{
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
        while (true) {
        LCD_clearScreen();
        LCD_setCursor(0,0);
        LCD_writeStr("Hello World");
        vTaskDelay(1000 / portTICK_RATE_MS);
        LCD_setCursor(0,1);
        LCD_writeStr("Ahihi");
        vTaskDelay(1000 / portTICK_RATE_MS);
        LCD_clearScreen();
    }
}
