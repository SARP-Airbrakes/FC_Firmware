
#ifndef DRIVER_BNO055_H_
#define DRIVER_BNO055_H_

#include <stdint.h>

/**
 * A class representing the driver for the BNO055 absolute orientation sensor.
 *
 * To use the driver, use the #connect method to initialize the connection with
 * the sensor.
 */
class bno055 {
public: // constants
    static constexpr uint8_t CHIP_ID_ADDR = 0x00; /* Chip identification code */
    static constexpr uint8_t PAGE_ID_ADDR = 0x07; /* Page Id */
    static constexpr uint8_t CALIB_STAT_ADDR = 0x35; /* Calibration status */
    static constexpr uint8_t SYS_STATUS_ADDR = 0x39; /* System status code */
    static constexpr uint8_t PWR_MODE_ADDR = 0x3E; /* Power mode */

    /** expected fixed-value of CHIP_ID (0x00) register (see 4.3.1). */
    static constexpr uint8_t CHIP_ID = 0xA0;

public: // types
    /**
     * Enum representing different error conditions with the driver.
     */
    enum class result {
        OK,
        UNCONNECTED,
    };

    /**
     * Enum representing the different sensors within the device.
     */
    enum class sensor {
        SYSTEM,
        GYROSCOPE,
        ACCELEROMETER,
        MAGNETOMETER,
    };

    /** Vector output type */
    struct vec4 {
        double x, y, z, w;
    };

    /**
     * Enum representing the different system status code values (see 4.3.58).
     */
    enum class sys_status_result : uint8_t { 
        SYSTEM_IDLE = 0, /* System Idle */
        SYSTEM_ERROR = 1, /* System Error */
    };

    /**
     * Enum representing the different operation modes (see 3.5).
     */
    enum class opr_mode {
        /* CONFIG MODE */
        CONFIGMODE = 0x00,

        /* Non-Fusion Mode */
        ACCONLY = 0x01,
        MAGONLY = 0x02,
        GYROONLY = 0x03,
        ACCMAG = 0x04,
        ACCGYRO = 0x05,
        MAGGYRO = 0x06,
        AMG = 0x07,

        /* Fusion Mode */
        IMU = 0x08,
        COMPASS = 0x09,
        M4G = 0x0a,
        NDOF_FMC_OFF = 0x0b,
        NDOF = 0x0c,
    };

    /**
     * Enum representing the different power modes (see 3.2).
     */
    enum class pwr_mode : uint8_t {
        NORMAL = 0x00,
        LOW_POWER = 0x01,
        SUSPEND = 0x02,
        INVALID = 0x03,
    };

public: // methods

    bno055();

    /**
     * Initializes a connection to the BNO055 using the given I2C pins and slave
     * address. For reference, the BNO055 uses a slave address of 0x29 by
     * default.
     *
     * Returns result::OK if the driver connected successfully. Otherwise,
     * returns result::UNCONNECTED.
     */
    result connect(int sda, int sdl, uint8_t addr);

    /**
     * Sets the current register map page of the device (see 4.3.8).
     */
    void set_page(int page);

    /**
     * Queries and checks the calibration status of one of the sensors of the
     * device (see 3.11.1 and 4.3.54).
     *
     * Returns true if the given sensor is fully calibrated. Otherwise, returns
     * false.
     */
    bool is_calibrated(sensor sensor);

    /**
     * Sets the power mode of the chip to the given power mode (see 3.2 and
     * 4.3.62).
     */
    void set_power_mode(pwr_mode mode);
    /**
     * Get the power mode of the chip (see 3.2 and 4.3.62).
     */
    pwr_mode get_power_mode();


    /**
     * Sets the power mode to suspend (see 3.2.3). All sensors are put into
     * sleep mode.
     */
    void suspend();

private:

    /**
     * Read an unsigned byte (8-bits) using the register address.
     */
    uint8_t read_byte(uint8_t address);

    /**
     * Read an unsigned short (16-bits) using the register addresses of the
     * least significant bits and the most significant bits.
     */
    uint16_t read_short(uint8_t lsb, uint8_t msb);

    /**
     * Write an unsigned byte (8-bits) of given value to given register address.
     */
    void write_byte(uint8_t address, uint8_t value);

    /**
     * Waits some given milliseconds.
     */
    void delay(int ms);

private:

    uint8_t addr;

};

#endif /* DRIVER_BNO055_H_ */
