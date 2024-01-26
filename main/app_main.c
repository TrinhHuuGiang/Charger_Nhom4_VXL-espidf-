//system
#include <string.h>
#include <sys/types.h>
#include <stdio.h>
//freertos
#include "freertos/FreeRTOS.h" // tao task freertos
#include "freertos/task.h" //task
//esp32 config
#include "esp_sleep.h"
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
#include "ds18b20.h"
#include "MicroSD.h"
//define
#define LCD_BUFFER_SIZE 16

//Tag
static const char *TAG = "Main";


/*__________Khai bao bien______________________________________*/
// trang thai nut bam
static bool power_stat = 1, menu_stat = 0,select_stat = 0; // trang thai 0 la chua duoc nhan nut
// trang thai relay
static bool relay_power_stat = 1, relay_1_stat = 0, relay_2_stat = 0; //trang thai relay
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
//18b20
//dia chi 18b20 tu menuconfig
DeviceAddress tempSensors[2];
static float temp1, temp2; // luu gia tri do C pin 1,2
// task status
bool task1_stat = true;
bool task2_stat = true;





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

//lay du lieu nhiet do tu 18b20
// do esp32 dung cau truc little endian VD: 0x123456 -> 0x56 0x34 0x12 khi luu vao flash
void hexToUint8Array(uint64_t hexValue, uint8_t array[8]);
void _18B20_get_temp();






/*______________task__________________________________*/
//task1 do update cac bien thong so do dac
// in1219,18b20, terminal
void task1(void *pvParameters); 
// task2: kiểm tra các biến đo được có phù hợp điều kiện
// esp32
void task2(void *pvParameters); 




/*____________APP_MAIN________________________________*/
void app_main(void)
{   
    esp_sleep_enable_ext0_wakeup(CONFIG_POWER_PIN, 0);
    // khởi tạo các ngoại vi
    Component_init();
    xTaskCreate(task1, "doluong_hienthi", 2048, NULL, 2, NULL);
    

    //done
    xTaskCreate(Buzzer_set_duty_task, NULL, 2048, NULL, 2, NULL);
    return;
}







/*____________Dinhnghia_______________________________*/
//khai bao ham ngat cho tung nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args)
{
        //deepsleep
        esp_deep_sleep_start();
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
    ds18b20_init(CONFIG_DQ_PIN); // khoi tao gpio lam bus 1 wire
    //cap nhat dia chi tu config vao mang dia chi temp sensor
    hexToUint8Array(CONFIG_18B_ADDR1, tempSensors[0]);
    hexToUint8Array(CONFIG_18B_ADDR2, tempSensors[1]);
	ds18b20_setResolution(tempSensors,2,10); //khai bao dia chi 2 cam bien, cai dat do phan giai 10bit
    // in ra dia chi 2 sensor len terminal
    printf("Address 1: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", 
        tempSensors[0][0],tempSensors[0][1],tempSensors[0][2],tempSensors[0][3],tempSensors[0][4],
        tempSensors[0][5],tempSensors[0][6],tempSensors[0][7]);
	printf("Address 2: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", 
        tempSensors[1][0],tempSensors[1][1],tempSensors[1][2],tempSensors[1][3],tempSensors[1][4],
        tempSensors[1][5],tempSensors[1][6],tempSensors[1][7]);
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
    // kiem tra neu chua bo tay ra khoi nut
    // tranh xung dot khi khai bao ngat
    while((button_get_level(CONFIG_POWER_PIN))==0) 
    {
        ESP_LOGI(TAG,"bo tay ra");
        vTaskDelay(pdMS_TO_TICKS(10));
        
    }
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

        printf("_____________________________________________________________________\n");
        printf("INA1: VBUS: %.04f V, IBUS: %.04f mA\n",
                bus_voltage1, current1 * 1000);
}

// do esp32 dung cau truc little endian VD: 0x123456 -> 0x56 0x34 0x12 khi luu vao flash
void hexToUint8Array(uint64_t hexValue, uint8_t array[8]) {
    for (int i = 0; i < 8; i++) {
        // Lấy giá trị của 1 byte từ giá trị hex và gán vào mảng
        array[7-i] = (uint8_t)(hexValue >> (i * 8));
    }
}

void _18B20_get_temp()
{
    ds18b20_requestTemperatures();// yeu cau cac cam bien cap nhat nhiet do
		float temp1 = ds18b20_getTempC((DeviceAddress *)tempSensors[0]); // lay do c tu PIN1
		float temp2 = ds18b20_getTempC((DeviceAddress *)tempSensors[1]); // ... PIN2
    //PRINT ON TERMINAL
        printf("Temperatures: (1) %0.1fC||(2) %0.1fC\n", temp1,temp2);
}

void task1(void *pvParameters){
    while(1)
    {
        if (task1_stat)
        {
            INA_get_current_Voltage();
            _18B20_get_temp();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task2(void *pvParameters){
    while(1)
    {
        if (task2_stat)
        {
            //so sanh nhiet do voi gia tri an toan


        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}