
#include "BNO055.h"

bno055::result bno055::connect(int sda, int sdl, char addr)
{
    /* TODO: initialize i2c hal wrapper */
    if (read_byte(bno055::CHIP_ID_ADDR) != bno055::CHIP_ID) {
        return bno055::result::UNCONNECTED;
    }
    return bno055::result::OK;
}

uint8_t bno055::read_byte(uint8_t address)
{
    /* TODO: read i2c using hal wrapper */
    return 0;
}

uint16_t bno055::read_short(uint8_t lsb, uint8_t msb)
{
    uint16_t out = 0;
    out |= read_byte(lsb);
    out |= ((uint16_t) read_byte(msb)) << 8;
    return out;
}
