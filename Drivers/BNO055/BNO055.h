
#ifndef DRIVER_BNO055_H_
#define DRIVER_BNO055_H_

#include <stdint.h>

class bno055 {
public:
    static constexpr uint8_t CHIP_ID_ADDR = 0x00;

    /** expected fixed-value of CHIP_ID (0x00) register */
    static constexpr uint8_t CHIP_ID = 0xA0;

public:
    enum class result {
        OK,
        UNCONNECTED,
    };

    struct vec4 {
        double x, y, z, w;
    };

public:

    bno055();
    result connect(int sda, int sdl, char addr);

    

private:

    /**
     * read an unsigned byte (8-bits) using the register address
     */
    uint8_t read_byte(uint8_t address);
    /**
     * read an unsigned short (16-bits) using the register addresses of the
     * least significant bits and the most significant bits
     */
    uint16_t read_short(uint8_t lsb, uint8_t msb);

};

#endif /* DRIVER_BNO055_H_ */
