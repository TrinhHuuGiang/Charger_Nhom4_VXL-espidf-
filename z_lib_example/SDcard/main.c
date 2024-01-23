#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h" // tao task freertos
#include "freertos/task.h" //
#include "driver/gpio.h"// driver gpio
#include "sdkconfig.h" // cau hinh menuconfig
#include "esp_log.h" // log thong bao

#include "MicroSD.h"

static const char *TAG = "Main";
static const char file1[] = "/infpin.txt"; // toi da 8 ki tu cho file name, 3 ki tu cho mo rong
static char linetext[] = "file luu thong tin pin"; 

void app_main(void)
{
    MicroSD_bus_config(CONFIG_MOSI_PIN, CONFIG_MISO_PIN, CONFIG_SCK_PIN, 1);
    MicroSD_init(CONFIG_CS_PIN_3,1);
    MicroSD_create_file(file1);
    MicroSD_write_file(file1,linetext);
    MicroSD_read_file(file1);
    MicroSD_delete_file(file1);
    MicrtSD_unmount();
    ESP_LOGI(TAG, "Ketthuc");
}