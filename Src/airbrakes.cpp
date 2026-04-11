
#include <airbrakes.h>

#include <sdk/i2c.h>
#include <sdk/spi.h>
#include <sdk/unique_pin.h>

#include <cmsis_os.h>

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
    sdk::drv8701 drv(8.0e-2, std::move(in1), std::move(in2));

    sdk::unique_pin encoder1(ENCODER1_GPIO_Port, ENCODER1_Pin);
    sdk::unique_pin encoder2(ENCODER2_GPIO_Port, ENCODER2_Pin);
    sdk::quad_encoder encoder(3200, std::move(encoder1), std::move(encoder2));

    sdk::motor_controller ctrl(1.8e-2f, 1.2e-3f, 5.0e-4f, std::move(drv),
            std::move(encoder));

    static airbrakes_state state(std::move(imu), std::move(baro),
            std::move(gps), std::move(flash), std::move(ctrl));

    state_handle = &state;

}

void airbrakes_start(void)
{
    state_handle->servo.start();

    state_handle->imu.set_acc_config(
        sdk::bmi088::acc_range::RANGE_6G,
        sdk::bmi088::acc_bwp::OSR4,
        sdk::bmi088::acc_odr::ODR_100HZ
    );
    state_handle->imu.start();
    osDelay(50); /* required by datasheet */

    state_handle->baro.set_config(2);
    state_handle->baro.set_odr(sdk::bmp390::odr::ODR_100);
    state_handle->baro.set_osr(
        sdk::bmp390::osr::OSR_4,
        sdk::bmp390::osr::OSR_4
    );
    state_handle->baro.set_power(true, true, sdk::bmp390::pwr_mode::PWR_NORMAL);
    state_handle->baro.read_calibration_data();

    state_handle->init();
}

void airbrakes_motor_update(float time)
{
    if (state_handle != nullptr)
        state_handle->servo.update_motor(time);
}

void airbrakes_step(void)
{
    state_handle->step();
}

void airbrakes_flash_pop_and_write(void)
{
    if (state_handle != nullptr)
        state_handle->update_flash();
}

bool whatever = false;

void airbrakes_blink_leds(void)
{
    whatever = !whatever;
    GPIO_PinState pin_state = whatever ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (state_handle != nullptr) {
        switch (state_handle->current_state) {
        case airbrakes_state::state::IDLE_PAD:
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, pin_state);
            if (state_handle->time >= airbrakes_state::TIME_THRESHOLD) {
                HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, pin_state);
            }
            break;
        case airbrakes_state::state::IDLE_FLIGHT:
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, pin_state);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, pin_state);
            break;
        case airbrakes_state::state::ACTIVE_FLIGHT:
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, pin_state);
            break;
        case airbrakes_state::state::IDLE_RECOVERY:
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, pin_state);
            break;
        default:
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, pin_state);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, pin_state);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, pin_state);
            break;
        }
    }
}

void airbrakes_gps_update(void)
{
    if (state_handle != nullptr)
        state_handle->gps.update();
}

void airbrakes_imu_update(void)
{
    if (state_handle != nullptr)
        state_handle->imu.update();
}

void airbrakes_baro_update(void)
{
    if (state_handle != nullptr)
        state_handle->baro.update();
}

// INTERRUPT CALLBACKS

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (state_handle != nullptr)
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

// THE FOLLOWING THREE FUNCTIONS ARE REALLY SHITTY TEMPORARY FIXES
// IGNOORRRREEEE

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // state_handle->gps.uart.transmit_complete_from_isr();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // state_handle->gps.uart.receive_complete_from_isr();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        uint32_t error = huart->ErrorCode;

        if (error & HAL_UART_ERROR_FE) {
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);

            // uh oh
            state_handle->gps.uart.error_from_isr();
        }

        // uart buf full, ignore and keep going
        if (error & HAL_UART_ERROR_ORE) {
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
            __HAL_UART_FLUSH_DRREGISTER(huart);
        }

        if (error & HAL_UART_ERROR_NE) {
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
            HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        }
    }
}
} // extern "C"

