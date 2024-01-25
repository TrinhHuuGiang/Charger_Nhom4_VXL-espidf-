//system
#include <string.h>
#include <sys/types.h>
#include <stdio.h>
//freertos
#include "freertos/FreeRTOS.h" // tao task freertos
#include "freertos/task.h" //task
//esp32 config
#include "driver/gpio.h"// driver gpio, interrupt
#include "sdkconfig.h" // cau hinh menuconfig
#include "esp_log.h" // log thong bao
#include "esp_err.h" // bao loi
#include <assert.h> //kiem tra loi

// thu vien component
#include "LCD1602.h"
#include "ds18b20.h"


//define
#define LCD_BUFFER_SIZE 16

//Tag
static const char *TAG = "Main";

/*__________Khai bao bien______________________________________*/
//dia chi 18b20 tu menuconfig
DeviceAddress tempSensors[2];

//lcd
static char LCD_BUFFER[LCD_BUFFER_SIZE];

/*__________Khai bao ham_______________________________________*/

// update dia chi 18b20 tu menuconfig
void hexToUint8Array(uint64_t hexValue, uint8_t array[8]);

/*____________APP_MAIN________________________________*/
void app_main(void)
{
    //_________________KHOITAO LCD___________//
    LCD_init(CONFIG_LCD_ADDR, CONFIG_SDA_PIN, CONFIG_SCL_PIN, CONFIG_LCD_COLS, CONFIG_LCD_ROWS);
    LCD_clearScreen();
    LCD_setCursor(0,0); // cot truoc hang sau 
    LCD_writeStr("Smart Charger");
    LCD_setCursor(0,1);
    LCD_writeStr("Nhom 4|ML:145579");
    vTaskDelay(1000 / portTICK_PERIOD_MS);//dung 1 s

    //_________________KHOITAO 18B20___________//
    ds18b20_init(CONFIG_DQ_PIN); // khoi tao gpio lam bus 1 wire
    //cap nhat dia chi tu config vao mang dia chi temp sensor
    hexToUint8Array(CONFIG_18B_ADDR1, tempSensors[0]);
    hexToUint8Array(CONFIG_18B_ADDR2, tempSensors[1]);
    //log nhiet do
	ds18b20_setResolution(tempSensors,2,10); //khai bao dia chi 2 cam bien, cai dat do phan giai 10bit

    // in ra dia chi 2 sensor len terminal
    printf("Address 1: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", 
        tempSensors[0][0],tempSensors[0][1],tempSensors[0][2],tempSensors[0][3],tempSensors[0][4],
        tempSensors[0][5],tempSensors[0][6],tempSensors[0][7]);
	printf("Address 2: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", 
        tempSensors[1][0],tempSensors[1][1],tempSensors[1][2],tempSensors[1][3],tempSensors[1][4],
        tempSensors[1][5],tempSensors[1][6],tempSensors[1][7]);
	while (1) {
		ds18b20_requestTemperatures();// yeu cau cac cam bien cap nhat nhiet do
		float temp1 = ds18b20_getTempF((DeviceAddress *)tempSensors[0]); // lay nhiet do F tu 1 trong 2 18B20
		float temp2 = ds18b20_getTempF((DeviceAddress *)tempSensors[1]); //... con lai
		float temp3 = ds18b20_getTempC((DeviceAddress *)tempSensors[0]); // lay do c
		float temp4 = ds18b20_getTempC((DeviceAddress *)tempSensors[1]); // ...

        //PRINT ON TERMINAL
        printf("Temperatures: (1) %0.1fF||(2) %0.1fF\n", temp1,temp2);
		printf("Temperatures: (1) %0.1fC||(2) %0.1fC\n", temp3,temp4);

        //PRINT ON LCD
        LCD_clearScreen();
        // Gán giá trị khoảng trắng cho mảng lcdBuffer
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"T1:%3.0fF|T2:%3.0fF",temp1,temp2);

        LCD_setCursor(0,0); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
        
        // Gán giá trị khoảng trắng cho mảng lcdBuffer
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"T1:%3.0fC|T2:%3.0fC",temp3,temp4);
        LCD_setCursor(0,1); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);

		vTaskDelay(5000 / portTICK_PERIOD_MS); // dung 5 s
	}
}


/*____________Dinhnghia_______________________________*/
// do esp32 dung cau truc little endian VD: 0x123456 -> 0x56 0x34 0x12 khi luu vao flash
void hexToUint8Array(uint64_t hexValue, uint8_t array[8]) {
    for (int i = 0; i < 8; i++) {
        // Lấy giá trị của 1 byte từ giá trị hex và gán vào mảng
        array[7-i] = (uint8_t)(hexValue >> (i * 8));
    }
}