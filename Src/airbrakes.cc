
#include <airbrakes.h>
#include <testing.h>

#include <sdk/spi.h>
#include <sdk/unique_pin.h>
#include <sdk/drivers/w25q16jv.h>

#include <usbd_cdc_if.h>
#include <main.h>

#include "cmsis_os2.h"
#include "sdk/i2c.h"
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

void airbrakes_i2c_interrupt(void *hdmatx)
{
    /* TODO: this is a error condition */
    if (hdmatx == nullptr) return;
    sdk::i2c_master *master = (sdk::i2c_master *) hdmatx;
    master->unblock_from_isr();
}

void airbrakes_cli_receive(uint8_t* Buf, uint32_t *Len)
{
    // send received packets to air brakes sdk
    airbrakes_serial_receive(Buf, Len); 

} // extern "C"

