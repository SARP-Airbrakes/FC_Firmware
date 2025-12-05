
#include <testing.h>

#include <main.h>

testing_result_t sdk_w25q128jv_read_blocking(void)
{
    // TODO: use the HAL_SPI_Transmit and HAL_SPI_Receive (blocking versions) to
    // read data from the chip
    TESTING_ASSERT(false);
}

const testing_test_t sdk_w25q128jv_tests[] = {
    { "Read (blocking)", sdk_w25q128jv_read_blocking },
    { nullptr, nullptr }
};
