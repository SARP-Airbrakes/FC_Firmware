
#ifndef AIRBRAKES_STATE_H_
#define AIRBRAKES_STATE_H_

#include <sdk/i2c.h>
#include <sdk/drivers/motor_controller.h>
#include <sdk/drivers/bmi088.h>
#include <sdk/drivers/bmp390.h>

/**
 * This class represents the Airbrakes controller state.
 */
struct airbrakes_state {

    /** 
     * States of a finite state machine representing the Airbrakes actuation
     * requirements.
     */
    enum class state {
        UNARMED, /* unarmed. needs human input */
        IDLE_PAD, /* idle, on pad */
        IDLE_FLIGHT, /* idle, during flight */
        ACTIVE_FLIGHT, /* active control, during flight */
        IDLE_RECOVERY, /* idle, after flight (recov.) */
    };

    /**
     * Initializes the airbrakes controller state.
     */
    airbrakes_state(sdk::i2c_master i2c1);

    /** Arms the airbrakes. */
    void arm();

    /** Finite state machine step */
    void step();

    /* updates internal driver states */
    void refresh_imu();
    void refresh_baro();

    state current_state;

    sdk::bmi088 imu;
    sdk::bmp390 baro;
};

#endif // AIRBRAKES_STATE_H_
