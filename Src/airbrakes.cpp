
#include <airbrakes.h>

#include <sdk/i2c.h>
#include <sdk/spi.h>
#include <sdk/unique_pin.h>

#include <usbd_cdc_if.h>
#include <main.h>

#include <cstdarg>

extern "C" {

airbrakes_state_handle_t state_handle;

void airbrakes_initialize()
{
    static sdk::i2c_master i2c1(&hi2c1);
    sdk::bmi088 imu(i2c1);
    sdk::bmp390 baro(i2c1);

    sdk::uart_buffered uart1(&huart1);
    sdk::cdpa1616d gps(std::move(uart1));

    sdk::unique_pin cs_flash(CS_FLASH_GPIO_Port, CS_FLASH_Pin);
    static sdk::spi spi1(&hspi1);
    sdk::w25q128jv flash(spi1, std::move(cs_flash));

    sdk::pwm in1(&htim2, sdk::pwm::tim_channel::CHANNEL_1);
    sdk::pwm in2(&htim2, sdk::pwm::tim_channel::CHANNEL_2);
    sdk::drv8701 drv(5.0e-2, std::move(in1), std::move(in2));

    sdk::unique_pin encoder1(ENCODER1_GPIO_Port, ENCODER1_Pin);
    sdk::unique_pin encoder2(ENCODER2_GPIO_Port, ENCODER2_Pin);
    sdk::quad_encoder encoder(3200, std::move(encoder1), std::move(encoder2));

    sdk::motor_controller ctrl(1.8e-2f, 1.2e-3f, 5.0e-4f, std::move(drv),
            std::move(encoder));

    static airbrakes_state state(std::move(imu), std::move(baro),
            std::move(gps), std::move(flash), std::move(ctrl));
    state_handle = &state;
}

void airbrakes_i2c_interrupt(void *hdmatx)
{
    /* TODO: this is a error condition */
    if (hdmatx == nullptr) return;
    sdk::i2c_master *master = (sdk::i2c_master *) hdmatx;
    master->unblock_from_isr();
}

void airbrakes_cli_receive(uint8_t* Buf, uint32_t Len)
{
}

void airbrakes_flash_pop_and_write(void)
{

}

// INTERRUPT CALLBACKS

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    state_handle->servo.encoder.read_and_update(GPIO_Pin);
}

static void i2c_unblock_from_isr(I2C_HandleTypeDef *hi2c)
{
    sdk::i2c_master *i2c = sdk::i2c_master::from_handle(hi2c);
    if (i2c != nullptr)
        i2c->unblock_from_isr();
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_unblock_from_isr(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    sdk::i2c_master *i2c = sdk::i2c_master::from_handle(hi2c);
    if (i2c != nullptr)
        i2c->unblock_from_isr();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    sdk::i2c_master *i2c = sdk::i2c_master::from_handle(hi2c);
    if (i2c != nullptr)
        i2c->error_from_isr();
}

static void spi_unblock_from_isr(SPI_HandleTypeDef *hspi)
{
    sdk::spi *spi = sdk::spi::from_handle(hspi);
    if (spi != nullptr)
        spi->unblock_from_isr();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_unblock_from_isr(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_unblock_from_isr(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_unblock_from_isr(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    sdk::spi *spi = sdk::spi::from_handle(hspi);
    if (spi != nullptr)
        spi->error_from_isr();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    sdk::uart_buffered *uart = sdk::uart_buffered::from_handle(huart);
    if (uart != nullptr)
        uart->transmit_complete_from_isr();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    sdk::uart_buffered *uart = sdk::uart_buffered::from_handle(huart);
    if (uart != nullptr)
        uart->receive_complete_from_isr();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    sdk::uart_buffered *uart = sdk::uart_buffered::from_handle(huart);
    if (uart != nullptr)
        uart->error_from_isr();
}
} // extern "C"

