
#include <airbrakes.h>

#include <sdk/spi.h>
#include <sdk/unique_pin.h>
#include <sdk/drivers/w25q16jv.h>

#include <usbd_cdc_if.h>
#include <main.h>

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
    CDC_Transmit_FS(rx_buf, (uint16_t) (rx_buf_end - rx_buf));

    static const uint8_t received_buf[] = "\r\n\r\n> ";
    CDC_Transmit_FS((uint8_t *) received_buf, sizeof(received_buf) - 1);
    rx_buf_end = rx_buf;
}

void airbrakes_serial_receive(uint8_t *buf, uint32_t *len)
{
    if (len != nullptr) {
        CDC_Transmit_FS(buf, *len);
        for (uint32_t i = 0; i < *len; i++) {
            if ((rx_buf_end - rx_buf) >= ((int) sizeof(rx_buf))) {
                airbrakes_serial_receive_command();
                return;
            }
            *(rx_buf_end++) = buf[i];
            if (buf[i] == '\n') {
                airbrakes_serial_receive_command();
                return;
            }
        }
    }
}

} // extern "C"

