
#include <airbrakes.h>

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
    static sdk::i2c_master i2c1(&hi2c1);
    sdk::bmi088 imu(i2c1);
    sdk::bmp390 baro(i2c1);

    sdk::uart_buffered uart1(&huart1);
    static sdk::cdpa1616d gps(std::move(uart1));

    sdk::unique_pin cs_flash(CS_FLASH_GPIO_Port, CS_FLASH_Pin);
    static sdk::spi spi1(&hspi1);
    static sdk::w25q128jv flash(spi1, std::move(cs_flash));

    sdk::pwm in1(&htim2, sdk::pwm::tim_channel::CHANNEL_1);
    sdk::pwm in2(&htim2, sdk::pwm::tim_channel::CHANNEL_2);
    sdk::drv8701 drv(5.0e-2, std::move(in1), std::move(in2));

    sdk::unique_pin encoder1(ENCODER1_GPIO_Port, ENCODER1_Pin);
    sdk::unique_pin encoder2(ENCODER2_GPIO_Port, ENCODER2_Pin);
    sdk::quad_encoder encoder(3200, std::move(encoder1), std::move(encoder2));

    static sdk::motor_controller ctrl(1.8e-2f, 1.2e-3f, 5.0e-4f, std::move(drv), std::move(encoder));

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

