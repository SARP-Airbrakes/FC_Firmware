
#include <airbrakes.h>
#include <testing.h>

#include <sdk/spi.h>
#include <sdk/unique_pin.h>
#include <sdk/drivers/w25q16jv.h>

#include <usbd_cdc_if.h>
#include <main.h>

#include "cmsis_os2.h"
#include <cstdarg>

#define FLASH_CS_PIN_GPIO GPIOA
#define FLASH_CS_PIN_GPIO_PIN 1

struct airbrakes_state {

    airbrakes_state(sdk::spi &spi1) : flash_driver(spi1,
            sdk::unique_pin(FLASH_CS_PIN_GPIO, FLASH_CS_PIN_GPIO_PIN))
    {
    }

    sdk::w25q16jv flash_driver;
};

static inline airbrakes_state &cast_state(airbrakes_state_handle_t handle)
{
    return *((airbrakes_state *) handle);
}

extern "C" {

airbrakes_state_handle_t airbrakes_initialize()
{
    static sdk::spi spi1(&hspi1);
    static airbrakes_state state(spi1);

    return &state;
}

void airbrakes_flash_driver_update(airbrakes_state_handle_t handle)
{
    airbrakes_state &state = cast_state(handle);
    state.flash_driver.update();
}

static uint8_t rx_buf[256];
static uint8_t *rx_buf_end = rx_buf;

static void airbrakes_serial_receive_command(void)
{
    HAL_GPIO_TogglePin(BLACKPILL_LED_GPIO_Port, BLACKPILL_LED_Pin);
    airbrakes_print_prompt();
    rx_buf_end = rx_buf;

    if (::strcmp((const char *) rx_buf, "test") == 0) {
        testing_test_and_print();
    } else {
        airbrakes_serial_printf("Invalid command \"%s\"\n", rx_buf);
    }
}

void airbrakes_print_prompt(void)
{
    airbrakes_serial_print("\r\n\r\n> ");
}

void airbrakes_serial_receive(uint8_t *buf, uint32_t *len)
{
    if (len != nullptr) {
        int offset = 0;
        for (uint32_t i = 0; i < *len; i++) {
            if ((rx_buf_end - rx_buf) >= ((int) sizeof(rx_buf))) {
                airbrakes_serial_receive_command();
                return;
            }
            if (buf[i] == '\r' || buf[i] == '\n') {
                *rx_buf_end = 0;
                airbrakes_serial_receive_command();
                offset--;
                return;
            } else {
                *(rx_buf_end++) = buf[i];
            }
        }
        CDC_Transmit_FS(buf, *len + offset);
    }
}

void airbrakes_serial_print(const char *buf)
{
    int i = 0;
    const char *tmp;
    for (tmp = buf; *tmp; tmp++)
        i++;
    CDC_Transmit_FS((uint8_t *) buf, i);
}

void airbrakes_serial_printf(const char *format, ...)
{
    char buf[128];
    ::va_list vargs;
    va_start(vargs, format);
    vsnprintf(buf, sizeof(buf), format, vargs);
    va_end(vargs);
    airbrakes_serial_print(buf);
}

} // extern "C"

