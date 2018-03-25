#include "dht.h"
#include "ets_sys.h"
#include "gpio.h"
#include "osapi.h"

// Timer for the sensor reading process.
static os_timer_t timer;
static os_timer_t timeout_timer;

// --------------------------------------------------------------------------------

static void ICACHE_FLASH_ATTR dht_read_sensor_start(void *arg);
static void ICACHE_FLASH_ATTR dht_read_sensor_timeout(void *arg);
static void ICACHE_FLASH_ATTR dht_finish_read(dht_t *sensor);
static void ICACHE_FLASH_ATTR dht_validate_value(dht_t *sensor);

// --------------------------------------------------------------------------------

void ICACHE_FLASH_ATTR dht_initialize(dht_t *sensor, uint8_t gpio_pin, uint8_t flags)
{
	sensor->gpio_pin = gpio_pin;
	sensor->flags = flags;
	sensor->state = DHT_STATE_IDLE;
	sensor->last_read = 0;
	sensor->temperature_raw = 0;
	sensor->humidity_raw = 0;

	// Setup pin to high initially.
	gpio_output_set(1 << sensor->gpio_pin, 0, 1 << sensor->gpio_pin, 0);

	// Initialize timers for the read process.
	os_timer_setfn(&timer, (os_timer_func_t *)dht_read_sensor_start, sensor);
	os_timer_setfn(&timeout_timer, (os_timer_func_t *)dht_read_sensor_timeout, sensor);

	// GPIO interrupts must be enabled for this library to work.
	ETS_GPIO_INTR_ENABLE();
}

void ICACHE_FLASH_ATTR dht_read_sensor(dht_t *sensor)
{
	// The sensor is being read!
	if (sensor->state != DHT_STATE_IDLE) {
		return;
	}

	// DHT21 can be polled at a 0.5Hz rate.
	uint32_t time = system_get_time();
	if (time - sensor->last_read < 2000000) {
		return;
	}

	sensor->state = DHT_STATE_STARTING;
	sensor->last_read = time;

	// Set pin mode to output and send a start signal (write a low).
	gpio_output_set(0, 1 << sensor->gpio_pin, 1 << sensor->gpio_pin, 0);

	// Start a timer to continue the reading process after a period defined in the sensors specs (should be 800us, we're using 1ms which should also be fine).
	os_timer_disarm(&timer);
	os_timer_arm(&timer, 1, 0);
}

float ICACHE_FLASH_ATTR dht_get_temperature(dht_t *sensor)
{
	return sensor->temperature_raw * 0.1f;
}

float ICACHE_FLASH_ATTR dht_get_humidity(dht_t *sensor)
{
	return sensor->humidity_raw * 0.1f;
}

void dht_handle_interrupt(void *arg)
{
	dht_t *sensor = (dht_t *)arg;

	// Clear the interrupt status.
	uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	// Read the value of the pin.
	sensor->state = (GPIO_INPUT_GET(sensor->gpio_pin) ? DHT_STATE_HIGH : DHT_STATE_LOW);

	// When not reading the sensor, ignore the interrupt (interrupts should be disabled anyway).
	if (sensor->state == DHT_STATE_IDLE) {
		return;
	}

	uint32_t time = system_get_time();

	if (sensor->state == DHT_STATE_LOW) {

		if (sensor->buffer_index >= 0) {
			
			// Falling edge -> calculate the duration of the high edge and determine the bits value.
			uint32_t duration = time - sensor->last_edge;

			// Duration of a '1' bit is between 68 and 75 us (typically 70).
			if (duration >= 68 && duration <= 75) {
				
				// Set the bit.
				int8_t bit = 39 - sensor->buffer_index;
				int8_t index = bit / 8;

				bit = bit - index * 8;
				sensor->buffer[index] |= (1 << bit);
			}
		}

		// Has the sensor already sent all 40 (32+8) bits?
		if (++sensor->buffer_index > 39) {

			dht_finish_read(sensor);
			dht_validate_value(sensor);
		}
	}

	// Store edge change time for the next interrupt.
	sensor->last_edge = time;

	// Rearm the timeout timer when still reading the sensor.
	if (sensor->buffer_index >= 0 && sensor->state != DHT_STATE_IDLE) {

		os_timer_disarm(&timeout_timer);
		os_timer_arm(&timeout_timer, 1, 0);
	}
}

static void ICACHE_FLASH_ATTR dht_read_sensor_start(void *arg)
{
	dht_t *sensor = (dht_t *)arg;

	// Enable input mode and switch to receive mode (high).
	gpio_output_set(1 << sensor->gpio_pin, 0, 0, 1 << sensor->gpio_pin);
	
	// Reset the read buffer.
	int i;
	for (i = 0; i < 5; ++i) {
		sensor->buffer[i] = 0;
	}
	
	sensor->buffer_index = -2; // The sensor will send low, high, low when it is ready. We can ignore them.

	// Attach an interrupt handler unless the user handles interrupts manually.
	if (sensor->flags & DHT_FLAG_INTERRUPTS) {
		ETS_GPIO_INTR_ATTACH(dht_handle_interrupt, sensor);
	}

	// Enable interrupts for the sensor pin.
	gpio_pin_intr_state_set(GPIO_ID_PIN(sensor->gpio_pin), GPIO_PIN_INTR_ANYEDGE);
	
	// Arm read timeout timer.
	os_timer_arm(&timeout_timer, 1, 0);
}

static void ICACHE_FLASH_ATTR dht_read_sensor_timeout(void *arg)
{
	ETS_GPIO_INTR_DISABLE();

	// Not received a value for a while, reset the sensors state.
	dht_t *sensor = (dht_t *)arg;
	dht_finish_read(sensor);

	ETS_GPIO_INTR_ENABLE();
}

static void ICACHE_FLASH_ATTR dht_finish_read(dht_t *sensor)
{
	// Disable interrupts for the sensor pin.
	gpio_pin_intr_state_set(GPIO_ID_PIN(sensor->gpio_pin), GPIO_PIN_INTR_DISABLE);

	// Mark the reading process as finished.
	sensor->state = DHT_STATE_IDLE;
	
	// Disarm the timeout timer.
	os_timer_disarm(&timeout_timer);

	// Set pin back to high.
	gpio_output_set(1 << sensor->gpio_pin, 0, 1 << sensor->gpio_pin, 0);
}

static void ICACHE_FLASH_ATTR dht_validate_value(dht_t *sensor)
{
	// The sum of the data bytes must equal the parity byte.
	int8_t checksum = (int8_t)(sensor->buffer[4] + sensor->buffer[3] + sensor->buffer[2] + sensor->buffer[1]);

	if (checksum == sensor->buffer[0]) {

		// Result was read properly - store it.
		sensor->temperature_raw = (uint16_t)((sensor->buffer[1] & 0xFF) + 256 * (sensor->buffer[2] & 0xFF));
		sensor->humidity_raw = (uint16_t)((sensor->buffer[3] & 0xFF) + 256 * (sensor->buffer[4] & 0xFF));

		sensor->has_value = 1;
	}
}
