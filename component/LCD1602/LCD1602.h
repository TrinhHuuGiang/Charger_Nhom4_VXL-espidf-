#pragma once
#include "driver/gpio.h"

// them hem i2c init cho LCD_init neu da bo ina219
void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows);
void LCD_turnOff();
void LCD_turnOn();
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_home(void);
void LCD_clearScreen(void);
void LCD_writeChar(char c);
void LCD_writeStr(char* str); 