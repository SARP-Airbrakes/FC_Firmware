
#include "sdk/shared_pin.h"
#include <airbrakes.h>

#include <sdk/spi.h>
#include <sdk/drivers/w25q16jv.h>
#include <main.h>

#define FLASH_CS_PIN_GPIO GPIOA
#define FLASH_CS_PIN_GPIO_PIN 1

struct airbrakes_state {

    airbrakes_state(sdk::spi &spi1) : flash_cs_pin(FLASH_CS_PIN_GPIO,
            FLASH_CS_PIN_GPIO_PIN), flash_driver(spi1, flash_cs_pin)
    {
    }

    sdk::shared_pin flash_cs_pin;
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

} // extern "C"

