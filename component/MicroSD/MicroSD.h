#ifndef MICROSDCARD
#define MICROSDCARD
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
void MicroSD_bus_config(uint8_t PIN_NUM_MOSI, uint8_t PIN_NUM_MISO, uint8_t PIN_NUM_CLK,
spi_host_device_t HOST_ID); // HSPI mac dinh la 1, VSPI la 2
void MicroSD_init(uint8_t PIN_NUM_CS,spi_host_device_t HOST_ID);
void MicroSD_create_file(const char *file);
void MicroSD_write_file(const char *file, char *linetext);
void MicroSD_read_file(const char *file);
void MicroSD_delete_file(const char *file);
void MicrtSD_unmount();

#endif