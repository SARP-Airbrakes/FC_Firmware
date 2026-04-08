
#include <airbrakes.h>
#include <testing.h>

#include <sdk/i2c.h>
#include <sdk/spi.h>
#include <sdk/unique_pin.h>

#include <usbd_cdc_if.h>
#include <main.h>

#include <cstdarg>

#define FLASH_CS_PIN_GPIO GPIOA
#define FLASH_CS_PIN_GPIO_PIN 1

extern "C" {

airbrakes_state_handle_t state_handle;

void airbrakes_initialize()
{
    sdk::i2c_master i2c1(&hi2c1);

    static airbrakes_state state(std::move(i2c1));
    state_handle = &state;
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

