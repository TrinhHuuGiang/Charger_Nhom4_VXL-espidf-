/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <esp_log.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "MicroSD.h"

static const char *TAG = "MicroSD";
const char mount_point[] = "/sdcard";
static sdmmc_card_t *card;
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();

static char* concat(const char *s1, const char *s2);

void MicroSD_bus_config(uint8_t PIN_NUM_MOSI, uint8_t PIN_NUM_MISO, uint8_t PIN_NUM_CLK,
spi_host_device_t HOST_ID)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing SPI peripheral");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    ret = spi_bus_initialize(HOST_ID, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    ESP_LOGI(TAG, "Success");
}

void MicroSD_init(uint8_t PIN_NUM_CS,spi_host_device_t HOST_ID)
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ESP_LOGI(TAG, "Initializing SD card");
    host.slot = HOST_ID;
    host.max_freq_khz = CONFIG_MAX_FREQUENCY;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}
void MicroSD_create_file(const char *file){

    const char *file_one = concat(mount_point,file);
    ESP_LOGI(TAG, "Kiem tra file %s",file_one);
    //kiem tra file ton tai
    struct stat st;
    if (stat(file_one, &st) == 0) {
        ESP_LOGE(TAG, "File da ton tai");
        return;
    }
    // tao file
    ESP_LOGI(TAG, "Dang tao va mo file %s", file_one);
    FILE *f = fopen(file_one, "w"); //tao file
    if (f == NULL) {
        ESP_LOGE(TAG, "Khong the mo file");
        return;
    }
    fclose(f);
    ESP_LOGI(TAG, "tao, mo va dong file thanh cong");
}
void MicroSD_write_file(const char *file, char *linetext)
{
    const char *file_one = concat(mount_point,file);
    //kiem tra file ton tai
    struct stat st;
    if (stat(file_one, &st) != 0) {
        ESP_LOGE(TAG, "File khong ton tai");
        return;
    }

    ESP_LOGI(TAG, "Opening file %s", file_one);
    FILE *f = fopen(file_one, "w"); // tao / mo de ghi
    if (f == NULL) {
        ESP_LOGE(TAG, "Khong the mo file");
        return;
    }

    ESP_LOGI(TAG, "Writing file %s", file_one);
    fprintf(f, "%s\n",linetext);
    fclose(f);


    ESP_LOGI(TAG, "mo, ghi va dong file thanh cong");
}

void MicroSD_read_file(const char *file)
{
    // Open renamed file for reading
    const char *file_one = concat(mount_point,file);
    ESP_LOGI(TAG, "Reading file %s", file_one);
    FILE *f = fopen(file_one, "r"); // mo file de doc
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    // Read a line from file
    char line[64];
    fgets(line, sizeof(line), f);
    ESP_LOGI(TAG, "Noi dung file: %s",line);
    fclose(f);
    ESP_LOGI(TAG, "Da dong file");
}
void MicroSD_delete_file(const char *file)
{
    //tao duong dan
    const char *file_one = concat(mount_point,file);
    ESP_LOGI(TAG, "Deleting file %s", file_one);
    // Check file da ton tai
    struct stat st;
    if (stat(file_one, &st) != 0) {
        ESP_LOGE(TAG, "Can't delete file %s", file_one);
        return;
    }
    // Delete it if it exists
    unlink(file_one);
    ESP_LOGI(TAG, "Deleted file %s", file_one);
}
void MicrtSD_unmount(){
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}
char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1) + strlen(s2) + 1); // +1 for the null-terminator
    // in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}