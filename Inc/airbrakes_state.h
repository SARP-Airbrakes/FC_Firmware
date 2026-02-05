
#ifndef AIRBRAKES_STATE_H_
#define AIRBRAKES_STATE_H_

#include <sdk/i2c.h>
#include <sdk/drivers/bmi088.h>
#include <sdk/drivers/bmp390.h>

/**
 * This class represents the Airbrakes controller state.
 */
struct airbrakes_state {

    enum class state {
        OFF, /* v < mach 0.7 */
        DEPLOYING, /* v > mach 0.7 */
        DEPLOYED, /* motors are off */
    };

    /**
     * Initializes the airbrakes controller state.
     */
    airbrakes_state(sdk::i2c_master i2c1);

    /** Fat update */
    void update();

    /* updates internal driver states */
    void refresh_imu();
    void refresh_baro();

    sdk::bmi088 imu;
    sdk::bmp390 baro;
};

#endif // AIRBRAKES_STATE_H_
