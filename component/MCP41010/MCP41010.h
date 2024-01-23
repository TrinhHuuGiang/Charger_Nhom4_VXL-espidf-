#ifndef MCP41010
#define MCP41010

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define KOHMS_MCP41010 10

enum MCP_POTENTIOMETER {
	METER_0,
	METER_1,
	METER_BOTH
};

typedef struct {
	uint8_t _chipSelectPin;
	spi_device_handle_t _handle; // dia chi thiet bi trong bo nho
	float _totalKOhms;
} MCP_t;
// bus con fig uu tien dung cua microsd
esp_err_t MCP41010_bus_config(gpio_num_t MOSI_PIN, gpio_num_t SCK_PIN, spi_host_device_t HOST_ID);

void MCP41010_init(MCP_t * dev,float totalKOhms ,gpio_num_t CS_PIN, spi_host_device_t HOST_ID);
void MCP41010_setWiper(MCP_t * dev,  uint8_t value, uint8_t potentiometer);
float MCP41010_getK(MCP_t *dev,  uint8_t value);

#endif