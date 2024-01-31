/// các chức năng thực hiện được: cảnh báo pin lắp ngược , pin không sạc , pin sạc nhanh , sạc chậm
// xử lý tự ngắt khi đầy pin

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
#define CUR_VAL_MIN 13 // 500 ohm >1A
#define CUR_VAL_1K 26 //== 1A
#define CUR_VAL_5K 128 // == 250mA
#define CUR_VAL_MAX 255 //10k 100mA

//Tag
static const char *TAG = "Main";


/*__________Khai bao bien______________________________________*/
// trang thai nut bam
static bool menu_stat = 0,select_stat = 0; // trang thai 0 la chua duoc nhan nut
// trang thai relay
static bool relay_power_stat = true, relay_1_stat = false, relay_2_stat = false; //trang thai relay
//gia tri MCP41010 cho tung module
static MCP_t mcp1, mcp2; 
static uint8_t potentiometer1 = METER_0;
static uint8_t potentiometer2 = METER_0;
//gia tri INA219 cho tung module
static ina219_t ina1,ina2;
static float bus_voltage1, current1; // điện áp và dòng điện đo đươc trên bus PIN1
static float bus_voltage2, current2;
//lcd
static char LCD_BUFFER[LCD_BUFFER_SIZE];
//18b20
//dia chi 18b20 tu menuconfig
DeviceAddress tempSensors[2];
static float temp1, temp2; // luu gia tri do C pin 1,2
// task status
//task1_đo đạc 2 pin
static bool task1_stat = true;
//task2_1_kiểm tra pin 1 trước khi sạc
static bool task2_1_stat = true;
//task2_2_kiểm tra pin 2 trước khi sạc
static bool task2_2_stat = true;
//task3_1_kiểm tra pin 1 trong khi sạc
static bool task3_1_stat = false;
//task3_2_kiểm tra pin 2 trong khi sạc
static bool task3_2_stat = false;
//task4_đổi trạng thái sạc
static bool task4_stat = true;
//task5_hiển thị 2 pin - mac dinh bat
static bool task5_stat = true;
//task6_1_hiển thị pin 1 - mac dinh tat
static bool task6_1_stat = false;
//task6_2_hiển thị pin 2 - mac dinh tat
static bool task6_2_stat = false;

// cac dieu kien can de danh gia pin
// nhiet do:
// 0: nhiet do tot
// 1: nhiet do cao (ko sac)
// 2: nhiet do thap (ko sac)
// dien ap:
// truoc khi sac:
// U = I * R // su dung dien ap tinh tu I
// 0 : > 1 ~ lap nguoc (ko sac)
// 1 : -1 -> 1  ~  khong pin (ko sac)
// 2 : -2.7 -> -1 ~ dien ap qua thap (ko sac)
// 3 : -4 -> -2.7 ~ lap dung pin + co the sac
// 4 : -4.3 -> -4 ~ pin day (ko sac)
// 5 : < -4,3 ~ lap dung nhung dien ap qua cao (lo sac)
// khi sac // su dung dien ap do duoc
// 6 : 2.7 -> 3 ~ sac cham
// 7 : 3 -> 4 ~ sac nhanh
// 8 : 4 -> 4.1 ~ sac cham
// 9 : > 4.1 -> ngat sac
static int pin1_temp = 0; // mac dinh nhiet do tot
static int pin1_vol = 1; // mac dinh 0 pin
static int pin2_temp = 0; // mac dinh
static int pin2_vol = 1; // mac dinh 0 pin

// charge mode
// mode =0 khong sac, = 1 bat dau sac, = 2 sac hoi phuc, = 3 sac dong cao, = 4 sac ket thuc
static int pin1_mode = 0; 
static int pin2_mode = 0; // mode ...

/*__________Khai bao ham_______________________________________*/
//khai bao ham ngat cho nut bam
static void IRAM_ATTR powerPin_interrupt_handler(void *args);
static void IRAM_ATTR menuPin_interrupt_handler(void *args);
static void IRAM_ATTR selectPin_interrupt_handler(void *args);

// KHOI TAO
// hexToUint8Array ho tro khoi tao dia chi 18B20
// do esp32 dung cau truc little endian VD: 0x123456 -> 0x56 0x34 0x12 khi luu vao flash
void hexToUint8Array(uint64_t hexValue, uint8_t array[8]);
//Ham khoi tao component
void Component_init();

// BUZZER ping
//bat tat coi
void Buzzer_set_duty_task(void *pvParameters);


/*______________task__________________________________*/
/*______task1______*/
// Log du lieu cam bien I,U,T 
//Cac ham lay du lieu cho task 1
//INA get current, voltage
void INA_get_current_Voltage();
//lay du lieu nhiet do tu 18b20
void _18B20_get_temp();
void task1(void *pvParameters); 

/*______task2______*/
// So sanh du lieu voi tieu chuan cua pin truoc khi sac da tao ra -> thay doi pin_mode
void Check_before_charge(int *pin_temp, int *pin_vol, int* pin_mode,
float current, float temp);
void task2_1(void *pvParameters); 
void task2_2(void *pvParameters); 

/*______task3______*/
// pin_mode -> dieu khien relay sac
void Check_while_charge(int *pin_temp, int *pin_vol, int* pin_mode,
float bus_voltage, float temp);
void task3_1(void *pvParameters);
void task3_2(void *pvParameters);

/*______task4______*/
// So sanh du lieu voi tieu chuan cua pin trong  khi sac da tao ra -> thay doi pin_mode
void set_charge_mode(MCP_t* mcp,uint8_t potentiometer ,int pin_mode, uint8_t _RELAY_PIN, bool* relay_stat,
bool* task2_stat, bool* task3_stat);
void task4(void *pvParematers);

/*______task5______*/
// hien thi chon 2 pin
void display_1(int pinnumber,int pin_mode, int row);
void task5(void *pvParematers);


/*______task6______*/
// chi tiet pin
// Ham ho tro hien thi task 6
// display
void display_1(int pinnumber,int pin_mode, int row);
void task6_1(void *pvParematers);
void task6_2(void *pvParematers);



/*____________APP_MAIN________________________________*/
void app_main(void)
{   
    esp_sleep_enable_ext0_wakeup(CONFIG_POWER_PIN, 0);
    // khởi tạo các ngoại vi
    Component_init();
    // create task
    xTaskCreate(task1, "doluong", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG,"1");
    xTaskCreate(task2_1, "sosanh1",2048,NULL,9,NULL);
    xTaskCreate(task2_2, "sosanh2",2048,NULL,9,NULL);
    ESP_LOGI(TAG,"2");
    xTaskCreate(task3_1, "mode1",2048,NULL,8,NULL);
    xTaskCreate(task3_2, "mode2",2048,NULL,8,NULL);
    ESP_LOGI(TAG,"3");
    xTaskCreate(task4, "charge", 2048, NULL, 10, NULL);
        ESP_LOGI(TAG,"4");
    xTaskCreate(task5, "display_all",2048,NULL,5,NULL);
        ESP_LOGI(TAG,"5");
    xTaskCreate(task6_1, "display_pin1",2048,NULL,5,NULL);
    xTaskCreate(task6_2, "display_pin2",2048,NULL,5,NULL);     
        ESP_LOGI(TAG,"6");  
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
        ESP_ERROR_CHECK(ina219_get_current(&ina1, &current1));
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&ina2, &bus_voltage2));
        ESP_ERROR_CHECK(ina219_get_current(&ina2, &current2));
        // in dong dien, dien ap cua ina sau khi lắp pin len terminal

    printf("_____________________________________________________________________\n");
        printf("INA1: VBUS: %.04f V, IBUS: %f A\n",
                bus_voltage1, current1);
        printf("INA2: VBUS: %.04f V, IBUS: %f A\n",
                bus_voltage2, current2);
                
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
	    temp1 = ds18b20_getTempC((DeviceAddress *)tempSensors[0]); // lay do c tu PIN1
		temp2 = ds18b20_getTempC((DeviceAddress *)tempSensors[1]); // ... PIN2
    //PRINT ON TERMINAL
        printf("Temperatures: (1) %0.1fC||(2) %0.1fC\n", temp1,temp2);
}

// 50ms lay thong so
void task1(void *pvParameters)
{ 
    while(1)
    {
        if (task1_stat)
        {
            INA_get_current_Voltage();
            _18B20_get_temp();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// chi can dong dien va nhiet do
void Check_before_charge(int* pin_temp, int *pin_vol, int* pin_mode,
float current, float temp)
{
    // check nhiet do
    if(temp >= 0 && temp <= 45) * pin_temp = 0;
    else if (temp > 45) * pin_temp = 1;
    else * pin_temp = 2;

    // check dien ap
    float bus_voltage = (float)current * (float)CONFIG_RES_TEST; // kiem tra dien ap khi pin xa ra dien tro test
    ESP_LOGI(TAG,"%f", bus_voltage);
    if(bus_voltage > 1) *pin_vol = 0; //lap nguoc
    else if(bus_voltage >=-1 && bus_voltage <=1) *pin_vol = 1; // khong pin
    else if(bus_voltage >-2.7 && bus_voltage <-1) *pin_vol = 2; // qua thap
    else if(bus_voltage >=-4 && bus_voltage <=-2.7) *pin_vol = 3; // ok
    else if(bus_voltage >=-4.3 && bus_voltage <-4) *pin_vol = 4; // full
    else *pin_vol = 5; // qua cao

            ESP_LOGI(TAG,"%d %d", * pin_temp, *pin_vol);
    // change pin_mode
    if(* pin_temp == 0 && *pin_vol == 3)
    {

        *pin_mode = 1; // bat dau sac voi dien tro cao nhat
    }
    else *pin_mode = 0; // khong du dieu kien sac

}

void task2_1(void *pvParameters)// 100ms kiem tra pin khi chua sac
{ 
    while(1)
    {
        if (task2_1_stat)
        {
            Check_before_charge(&pin1_temp,&pin1_vol, &pin1_mode, current1, temp1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void task2_2(void *pvParameters)// 100ms kiem tra pin khi chua sac
{ 
    while(1)
    {
        if (task2_2_stat)
        {
            Check_before_charge(&pin2_temp,&pin2_vol, &pin2_mode, current2, temp2);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Check_while_charge(int *pin_temp, int *pin_vol, int* pin_mode,
float bus_voltage, float temp)
{
    // check nhiet do
    if(temp >= 0 && temp <= 45) *pin_temp = 0;
    else if (temp > 45) *pin_temp = 1;
    else *pin_temp = 2;

    // check dien ap
    ESP_LOGE(TAG,"%f",bus_voltage);
    if(bus_voltage >= 2.7 && bus_voltage <= 3) *pin_vol = 6; // sac hoi phuc
    else if(bus_voltage >3 && bus_voltage <=4) *pin_vol = 7; // sac nhanh
    else if(bus_voltage > 4 && bus_voltage <= 4.1) *pin_vol = 8;  // sac ket thuc
    else *pin_vol = 9; // < 2.7 hoac > 4.1 ngat sac

    // change pin_mode
    if(*pin_temp == 0)
    {
        if(*pin_vol == 6)
        {
            *pin_mode = 2; // sac hoi phuc 
        }
        else if(*pin_vol == 7) *pin_mode = 3; // sac nhanh
        else if(*pin_vol == 8) *pin_mode = 4; // sac ket thuc
        else *pin_mode = 0; // khong du dieu kien sac ( sac xong hoac <2.7 v)
    }
    else *pin_mode = 0; // nhiet do khong dat
}

void task3_1(void *pvParameters)// 100ms kiem tra pin khi dang sac
{ 
    while(1)
    {
        if (task3_1_stat)
        {
            Check_while_charge(&pin1_temp,&pin1_vol, &pin1_mode, bus_voltage1, temp1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void task3_2(void *pvParameters)// 100ms kiem tra pin khi dang sac
{ 
    while(1)
    {
        if (task3_2_stat)
        {
            Check_while_charge(&pin2_temp,&pin2_vol, &pin2_mode, bus_voltage2, temp2);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void set_charge_mode(MCP_t* mcp,uint8_t potentiometer ,int pin_mode, uint8_t _RELAY_PIN, bool* relay_stat,
bool* task2_stat, bool* task3_stat)
{
    ESP_LOGI(TAG,"%d", pin_mode);
    // CHECK MODE AND CHANGE RELAY
    if ( pin_mode == 1) // bat dau sac
    {
        //set mcp len 10k
        MCP41010_setWiper(mcp, CUR_VAL_MAX, potentiometer);
        if(*relay_stat == false)
        {
        //set relay on
        relay_set_level(_RELAY_PIN, 1);
        *relay_stat = true; // bao relay da bat
        }
    }
    else if ( pin_mode == 2) // sac cham
    {
        //set mcp len 5k
        MCP41010_setWiper(mcp, CUR_VAL_5K, potentiometer);
        if(*relay_stat == false)
        {
        //set relay on
        relay_set_level(_RELAY_PIN, 1);
        *relay_stat = true; // bao relay da bat
        }
    }
    else if ( pin_mode == 3)
    {
        //set mcp len 1k
        MCP41010_setWiper(mcp, CUR_VAL_1K, potentiometer);
        if(*relay_stat == false)
        {
        //set relay on
        relay_set_level(_RELAY_PIN, 1);
        *relay_stat = true; // bao relay da bat
        }
    }
    else if ( pin_mode == 4)
    {
        //set mcp len 5k
        MCP41010_setWiper(mcp, CUR_VAL_5K, potentiometer);
        if(*relay_stat == false)
        {
        //set relay on
        relay_set_level(_RELAY_PIN, 1);
        *relay_stat = true; // bao relay da bat
        }
    }
    else // mode 0
    {
        if(*relay_stat == true)
        {
        //set mcp len 10k
        MCP41010_setWiper(mcp, CUR_VAL_MAX, potentiometer);
        //set relay off
        relay_set_level(_RELAY_PIN, 0);
        *relay_stat = false; // bao relay da tat
        }
    }

    // BLOCK AND OPEN TASK 
    // neu pin mode = 0 thi khong sac, block task 3 va quay lai task 2 de kiem tra pin truoc khi sac
    // neu pin mode = 1,2,3,4 thi block task 2, task 3 kiểm tra pin trong khi sac
    if ( pin_mode == 0 )
    {  
        *task2_stat = true;
        *task3_stat = false; 
    }
    else
        {
            *task2_stat = false;
            *task3_stat = true;
        }
}

void task4(void *pvParematers)// 10ms quyet dinh thay doi trang thai sac
{ 
    while(1)
    {
        if (task4_stat)
        {
            set_charge_mode(&mcp1,potentiometer1 , pin1_mode, CONFIG_RELAY_PIN_1, & relay_1_stat, 
            &task2_1_stat, &task3_1_stat);
            set_charge_mode(&mcp2,potentiometer2 , pin2_mode, CONFIG_RELAY_PIN_2, & relay_2_stat,
            &task2_2_stat, &task3_2_stat);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// hien thi man hinh 2 pin
void display_1(int pinnumber,int pin_mode, int row)
{
    if(pin_mode == 0) //khong sac
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d: Khong sac",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
    else if(pin_mode == 1)
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d: Bat dau",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
    else if(pin_mode == 2)
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d: Hoi phuc",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
    else if(pin_mode == 3)
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d: Sac nhanh",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
    else if(pin_mode == 4)
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d:Hoan thanh",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
    else
    {
        for (int i = 0; i < LCD_BUFFER_SIZE; i++) {
        LCD_BUFFER[i] = ' ';
        }
        snprintf(LCD_BUFFER, LCD_BUFFER_SIZE,"PIN%d:ERROR",pinnumber);
        LCD_setCursor(0,row); // cot truoc hang sau 
        LCD_writeStr(LCD_BUFFER);
    }
}
void task5(void *pvParematers)// 1000ms hien thi trang thai 2 pin
{ 
    while(1)
    {
        if (task5_stat)
        {
            LCD_clearScreen();
            display_1(1,pin1_mode,0);
            display_1(2,pin2_mode,1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void task6_1(void *pvParematers)// 1000ms hien thi trang thai pin 1
{ 
    while(1)
    {
        if (task6_1_stat)
        {
            
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task6_2(void *pvParematers)// 1000ms hien thi trang thai pin 2
{ 
    while(1)
    {
        if (task6_2_stat)
        {

        }       
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}