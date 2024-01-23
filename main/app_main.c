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
#include "driver/spi_master.h"//thu vien spi master
// thu vien component
#include "Relay.h"
#include "MCP41010.h"
#include "LCD1602.h"
#include "ina219.h" 
#include "Buzzer.h" 
#include "BTN.h"    //button
#include "MicroSD.h"
//Tag
static const char *TAG = "Main";

/*__________Khai bao bien______________________________________*/
// trang thai nut bam
static bool power_stat = 0,menu_stat = 0,select_stat = 0; // trang thai 0 la chua duoc nhan nut
//gia tri MCP41010 cho tung module
static MCP_t mcp1, mcp2; 
static uint8_t cur_value1 = 0, DIRECTION1 = 1, potentiometer1 = METER_0;
static uint8_t cur_value2 = 0, DIRECTION2 = 1, potentiometer2 = METER_0;
//gia tri INA219 cho tung module
static ina219_t ina1,ina2;
static float bus_voltage1, shunt_voltage1, current1, power1;
static float bus_voltage2, shunt_voltage2, current2, power2;
//gia tri 18B20 cho tung module
DeviceAddress tempSensors[2];



/*__________Khai bao ham_______________________________________*/
//khai bao ham ngat cho nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args);
static void IRAM_ATTR menuPin_interrupt_handler(void *args);
static void IRAM_ATTR selectPin_interrupt_handler(void *args);

//Ham khoi tao component
void Component_init();

/*____________APP_MAIN________________________________*/
void app_main(void)
{
}


/*____________Dinhnghia_______________________________*/
//khai bao ham ngat cho tung nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args)
{
    power_stat = !power_stat; // doi trang thai
}
static void IRAM_ATTR menuPin_interrupt_handler(void *args)
{
    menu_stat = !menu_stat; // doi trang thai
}
static void IRAM_ATTR selectPin_interrupt_handler(void *args)
{
    select_stat = !select_stat; // doi trang thai
}

//khoi tao component:Button, Relay, Buzzer, MCP41010, INA219, 18B20, LCD, SD
void Component_init()
{
    ESP_LOGI(TAG,"Dang khoi tao thiet bi:");
/*__________BUTTON__________*/
    // cai driver ngat
    ESP_LOGI(TAG,"ISR");
    gpio_install_isr_service(0);
    // Khoi tao button
    ESP_LOGI(TAG,"BUTTON");
    button_init(CONFIG_POWER_PIN, GPIO_MODE_INPUT);
    button_init(CONFIG_MENU_PIN, GPIO_MODE_INPUT);
    button_init(CONFIG_SELECT_PIN, GPIO_MODE_INPUT);
    //them ngat, ham ngat cho gpio button
    ESP_LOGI(TAG,"BUTTON_ISR");
    gpio_isr_handler_add(CONFIG_POWER_PIN, powerPin_interrupt_handler, (void *)CONFIG_POWER_PIN);
    gpio_isr_handler_add(CONFIG_MENU_PIN, menuPin_interrupt_handler, (void *)CONFIG_MENU_PIN);
    gpio_isr_handler_add(CONFIG_SELECT_PIN, selectPin_interrupt_handler, (void *)CONFIG_SELECT_PIN);
/*__________RELAY___________*/
    // khoi tao relay
    ESP_LOGI(TAG,"RELAY");
    relay_init(CONFIG_RELAY_NGUON,GPIO_MODE_OUTPUT,1); //relay nguon bat
    relay_init(CONFIG_RELAY_PIN_1,GPIO_MODE_OUTPUT,0); //tat
    relay_init(CONFIG_RELAY_PIN_2,GPIO_MODE_OUTPUT,0); //tat

/*__________BUZZER__________*/
    ESP_LOGI(TAG,"BUZZER");
    Buzzer_init(CONFIG_BUZZER_PIN);

/*__________MCP41010________*/
    // cau hinh hspi bus su dung chung cho micro sd va mcp41010
    ESP_LOGI(TAG,"MCP41");
    MicroSD_bus_config(CONFIG_MOSI_PIN, CONFIG_MISO_PIN, CONFIG_SCK_PIN, HSPI_HOST);
    MCP41010_init(&mcp1, KOHMS_MCP41010, CONFIG_CS_PIN_1, HSPI_HOST);
    MCP41010_init(&mcp2, KOHMS_MCP41010, CONFIG_CS_PIN_2, HSPI_HOST);

/*__________INA219__________*/
    //reset vung nho, khoi tao, cau hinh, hieu chuan INA219 so 1
    memset(&ina1, 0, sizeof(ina219_t));
    assert(CONFIG_SHUNT_RES > 0);
    ESP_ERROR_CHECK(ina219_init_desc(&ina1, CONFIG_I2C_ADDR1, CONFIG_I2C_PORT, CONFIG_SDA_PIN, 
    CONFIG_SCL_PIN));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&ina1));
    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&ina1, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
            INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));
    ESP_LOGI(TAG, "Calibrating INA219");
    ESP_ERROR_CHECK(ina219_calibrate(&ina1, (float)CONFIG_MAX_CURRENT, 
    (float)CONFIG_SHUNT_RES / 1000.0f));

    //reset vung nho, khoi tao, cau hinh, hieu chuan INA219 so 2
    memset(&ina2, 0, sizeof(ina219_t));
    assert(CONFIG_SHUNT_RES > 0);
    ESP_ERROR_CHECK(ina219_init_desc(&ina2, CONFIG_I2C_ADDR2, CONFIG_I2C_PORT, CONFIG_SDA_PIN, 
    CONFIG_SCL_PIN));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&ina2));
    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&ina2, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
            INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));
    ESP_LOGI(TAG, "Calibrating INA219");
    ESP_ERROR_CHECK(ina219_calibrate(&ina2, (float)CONFIG_MAX_CURRENT, 
    (float)CONFIG_SHUNT_RES / 1000.0f));

/*__________18B20___________*/
/*__________LCD_____________*/
/*__________SD______________*/


}