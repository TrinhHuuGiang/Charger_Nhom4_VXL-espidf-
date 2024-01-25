// su dung file nay de dieu chinh mcp1 thong qua button select
// button power duoc su dung de bat tat relay nguon
// gia tri dien tro se in ra lcd dong 1
// ina219 se do dong dien, dien ap qua pin1 va hien thi o dong 2
//chưa hoàn thiện

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

//define
#define LCD_BUFFER_SIZE 16

//Tag
static const char *TAG = "Main";

/*__________Khai bao bien______________________________________*/
// trang thai nut bam
static bool power_stat = 1,menu_stat = 0,select_stat = 0; // trang thai 0 la chua duoc nhan nut
// trang thai relay
static bool relay_power_stat = 1,relay_1_stat = 0, relay_2_stat = 0;
//gia tri MCP41010 cho tung module
static MCP_t mcp1, mcp2; 
static uint8_t cur_value1 = 0, DIRECTION1 = 1, potentiometer1 = METER_0;
static uint8_t cur_value2 = 0, DIRECTION2 = 1, potentiometer2 = METER_0;
//gia tri INA219 cho tung module
static ina219_t ina1,ina2;
static float bus_voltage1, shunt_voltage1, current1, power1;
static float bus_voltage2, shunt_voltage2, current2, power2;
//lcd
static char LCD_BUFFER[LCD_BUFFER_SIZE];
// task status
bool task1_stat = true;

/*__________Khai bao ham_______________________________________*/
//khai bao ham ngat cho nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args);
static void IRAM_ATTR menuPin_interrupt_handler(void *args);
static void IRAM_ATTR selectPin_interrupt_handler(void *args);


//Ham khoi tao component
void Component_init();

//bat tat coi
void Buzzer_set_duty_task(void *pvParameters);

//INA get current, voltage
void INA_get_current_Voltage();

//task
void task1(void *pvParameters); //lay va hien thi do dac len man ihin

/*____________APP_MAIN________________________________*/
void app_main(void)
{
    // khởi tạo các ngoại vi
    Component_init();
    xTaskCreate(task1, "doluong_hienthi", 2048, NULL, 2, NULL);
    return;
}
/*____________Dinhnghia_______________________________*/
//khai bao ham ngat cho tung nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args)
{
    power_stat = !power_stat; // doi trang thai bien power
    relay_power_stat=power_stat; //doi trang thai bien relay nguon
    if(power_stat)
    {
        //buzzer keu
        xTaskCreate(Buzzer_set_duty_task, NULL, 2048, NULL, 2, NULL);
        //bat nguon
        relay_set_level(CONFIG_RELAY_NGUON, power_stat);
        //set task
        task1_stat = true;
    }
    else
    {
        //buzzer keu
        xTaskCreate(Buzzer_set_duty_task, NULL, 2048, NULL, 2, NULL);
        //tat nguon
        relay_set_level(CONFIG_RELAY_NGUON, power_stat);
        //set task
        task1_stat = false;
    }
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
/*__________RELAY___________*/
    // khoi tao relay
    ESP_LOGI(TAG,"RELAY");
    relay_init(CONFIG_RELAY_NGUON,GPIO_MODE_OUTPUT,relay_power_stat); //relay nguon bat
    relay_init(CONFIG_RELAY_PIN_1,GPIO_MODE_OUTPUT,relay_1_stat); //tat
    relay_init(CONFIG_RELAY_PIN_2,GPIO_MODE_OUTPUT,relay_2_stat); //tat

/*__________BUZZER__________*/
    ESP_LOGI(TAG,"BUZZER");
    Buzzer_init(CONFIG_BUZZER_PIN);

/*__________MCP41010________*/
    // cau hinh hspi bus su dung chung cho micro sd va mcp41010
    ESP_LOGI(TAG,"MCP41");
    MCP41010_bus_config(CONFIG_MOSI_PIN,CONFIG_SCK_PIN, HSPI_HOST);
    MCP41010_init(&mcp1, KOHMS_MCP41010, CONFIG_CS_PIN_1, HSPI_HOST);
    MCP41010_init(&mcp2, KOHMS_MCP41010, CONFIG_CS_PIN_2, HSPI_HOST);

/*__________INA219__________*/
    ESP_LOGI(TAG,"INA219");
    i2cdev_init();// cai driver giao tiep i2c
    //reset vung nho, khoi tao, cau hinh, hieu chuan INA219 so 1
    memset(&ina1, 0, sizeof(ina219_t));
    assert(CONFIG_SHUNT_RES > 0);
    ESP_ERROR_CHECK(ina219_init_desc(&ina1, CONFIG_INA_ADDR1, CONFIG_I2C_PORT, CONFIG_SDA_PIN, 
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
    ESP_ERROR_CHECK(ina219_init_desc(&ina2, CONFIG_INA_ADDR2, CONFIG_I2C_PORT, CONFIG_SDA_PIN, 
    CONFIG_SCL_PIN));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&ina2));
    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&ina2, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
            INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));
    ESP_LOGI(TAG, "Calibrating INA219");
    ESP_ERROR_CHECK(ina219_calibrate(&ina2, (float)CONFIG_MAX_CURRENT, 
    (float)CONFIG_SHUNT_RES / 1000.0f));

/*__________LCD_____________*/
    ESP_LOGI(TAG,"LCD");
    LCD_init(CONFIG_LCD_ADDR, CONFIG_SDA_PIN, CONFIG_SCL_PIN, CONFIG_LCD_COLS, CONFIG_LCD_ROWS);
    LCD_clearScreen();
    LCD_setCursor(0,0); // cot truoc hang sau 
    LCD_writeStr("Smart Charger");
    LCD_setCursor(0,1);
    LCD_writeStr("Nhom 4|ML:145579");
    vTaskDelay(pdMS_TO_TICKS(1000));
/*__________18B20___________*/


/*__________SD______________*/

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
}

void Buzzer_set_duty_task(void *pvParameters)
{
    Buzzer_set_duty(150);
    vTaskDelay(pdMS_TO_TICKS(100));
    Buzzer_set_duty(0);
    vTaskDelete(NULL);
}

void INA_get_current_Voltage(){
        
        //log ina219 , day len lcd va terminal
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&ina1, &bus_voltage1));
        ESP_ERROR_CHECK(ina219_get_shunt_voltage(&ina1, &shunt_voltage1));
        ESP_ERROR_CHECK(ina219_get_current(&ina1, &current1));
        ESP_ERROR_CHECK(ina219_get_power(&ina1, &power1));

        // in dong dien, dien ap cua ina sau khi lắp pin len terminal
        // in dien tro tinh toan cua mcp len terminal
        // co 2 truong hop can xem la khi bam button, relay mo ra thi dong dien, dien ap thay doi ra sao

        printf("_____________________________________________________________________\n");
        printf("INA1: VBUS: %.04f V, VSHUNT: %.04f mV, IBUS: %.04f mA, PBUS: %.04f mW\n",
                bus_voltage1, shunt_voltage1 * 1000, current1 * 1000, power1 * 1000);

        // in len lcd
        LCD_clearScreen();
        // in ina1
        // Gán giá trị khoảng trắng cho mảng lcdBuffer
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"1:%4.1fmA|%2.1fV",current1*1000,bus_voltage1);
        LCD_setCursor(0,1); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
}

void task1(void *pvParameters){
    while(1)
    {
        if (task1_stat)
        {
            INA_get_current_Voltage();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
