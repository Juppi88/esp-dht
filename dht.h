#ifndef __DHT_H
#define __DHT_H

#include "os_type.h"

typedef enum {
	DHT_STATE_IDLE,
	DHT_STATE_STARTING,
	DHT_STATE_LOW,
	DHT_STATE_HIGH,
} dht_state_t;

typedef enum {
	DHT_FLAG_INTERRUPTS = 0x1,	// Attach interrupts automatically. This is useful if the temp sensor is the only source of interrupts.
} dht_flags_t;

// --------------------------------------------------------------------------------

typedef struct dht_t {
	uint8_t gpio_pin;			// GPIO pin the sensor is connected to
	uint8_t	flags;				// Flags this sensor was initialized with
	volatile uint8_t has_value;	// Set to 1 when the device has decoded a verified value from the sensor
	volatile uint8_t state;		// Current state of the sensor (see dht_state_t)
	int8_t buffer[5];			// Raw data from the sensor
	int8_t buffer_index;		// When reading the sensor, contains the index of the bit beind read

	uint32_t last_read;			// Timestamp when the sensor was last read
	uint32_t last_edge;			// Timestamp for the last edge change
	volatile uint16_t temperature_raw;	// Raw temperature value from the sensor
	volatile uint16_t humidity_raw;		// Raw humidity value from the sensor
} dht_t;

// --------------------------------------------------------------------------------

// Initialize the sensor on the given GPIO pin.
void ICACHE_FLASH_ATTR dht_initialize(dht_t *sensor, uint8_t gpio_pin, uint8_t flags);

// Wake up the sensor for a value read.
void ICACHE_FLASH_ATTR dht_read_sensor(dht_t *sensor);

// When 'has_value' field is set, these will return the temperature and humidity of the most recent successful read.
float ICACHE_FLASH_ATTR dht_get_temperature(dht_t *sensor);
float ICACHE_FLASH_ATTR dht_get_humidity(dht_t *sensor);

// Call the sensors interrupt handler manually if and only if a global handler is used outside this library.
// The argument is the dht_t struct which triggered the interrupt.
void dht_handle_interrupt(void *arg);

#endif
