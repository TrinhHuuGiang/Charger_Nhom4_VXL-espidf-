#include "MCP41010.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
static const int SPI_Frequency = SPI_MASTER_FREQ_20M;

esp_err_t MCP41010_bus_config(gpio_num_t MOSI_PIN, gpio_num_t SCK_PIN, spi_host_device_t HOST_ID)
{
		spi_bus_config_t buscfg = {
		.mosi_io_num = MOSI_PIN,
		.miso_io_num = -1,
		.sclk_io_num = SCK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};
	return spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO ); // khoi tao bus
}

void MCP41010_init(MCP_t * dev,float totalKOhms ,gpio_num_t CS_PIN, spi_host_device_t HOST_ID)
{
	//Khoi tao chan CS;
	gpio_reset_pin( CS_PIN );
	gpio_set_direction( CS_PIN, GPIO_MODE_OUTPUT );
	gpio_set_level( CS_PIN, 1 );

	//Cau hinh thiet bi
	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg)); // khoi tao vung nho chua du lieu devcfg
	devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.queue_size = 7;
	devcfg.mode = 0;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;
	devcfg.spics_io_num = CS_PIN;
	spi_device_handle_t handle;
	spi_bus_add_device( HOST_ID, &devcfg, &handle); // khoi tao ngoai vi

	dev->_chipSelectPin = CS_PIN;
	dev->_handle = handle;
	dev->_totalKOhms = totalKOhms;

}

void MCP41010_setWiper(MCP_t * dev,  uint8_t value, uint8_t potentiometer)
{
	uint8_t data[2];
	if (potentiometer == METER_0) {
		data[0] = 0x11; // Command executed on Potentiometer 0.
	} else if (potentiometer == METER_1) {
		data[0] = 0x12; // Command executed on Potentiometer 1.
	} else if (potentiometer == METER_BOTH) {
		data[0] = 0x13; // Command executed on both Potentiometers.
	}
	data[1] = value;
	//digitalWrite(_chipSelectPin, LOW);
	spi_transaction_t SPITransaction;

	// khoi tao vung nho void *memset(void *ptr, int value, size_t num);
	/*ptr: Con trỏ đến vùng nhớ cần thiết lập.
	value: Giá trị muốn gán cho mỗi byte trong vùng nhớ.
	num: Số lượng byte cần thiết lập.*/
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	spi_device_transmit( dev->_handle, &SPITransaction );
}

float MCP41010_getK(MCP_t *dev,  uint8_t value)
{
	return (dev->_totalKOhms/0xFF) * value;
}