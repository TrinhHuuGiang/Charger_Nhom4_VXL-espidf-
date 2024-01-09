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

void mcp41010_init(MCP_t * dev,float totalKOhms ,spi_host_device_t HOST_ID,
gpio_num_t MOSI_PIN, gpio_num_t SCK_PIN,gpio_num_t CS_PIN);
void MCP41010_setWiper(MCP_t * dev,  uint8_t value, uint8_t potentiometer);
float MCP41010_getK(MCP_t *dev,  uint8_t value);

#endif