
#include <airbrakes_state.hpp>

airbrakes_state::airbrakes_state(sdk::i2c_master i2c1) : imu(i2c1), baro(i2c1)
{
}

void airbrakes_state::arm()
{
    if (current_state == state::UNARMED)
        current_state = state::IDLE_PAD;
}

void airbrakes_state::step()
{
    switch (current_state) {
    case state::UNARMED:
        /**
         * Nothing should happen here. We should wait until manual input.
         */
        break;
    case state::IDLE_PAD:
        /* 
         * TODO: There should be one state transition: IDLE_PAD -> IDLE_FLIGHT.
         * This transition should occur on launch - this could be as simple as
         * detecting a new acceleration (delta acceleration > 0?)
         */
        break;
    case state::IDLE_FLIGHT:
        /**
         * TODO: There should be one state transition: IDLE_FLIGHT ->
         * ACTIVE_FLIGHT. This transition should occur as soon as we detect that
         * the velocity drops below Mach 0.7.
         */
        break;
    case state::ACTIVE_FLIGHT:
        /**
         * TODO: There should be one state transition: ACTIVE_FLIGHT ->
         * IDLE_RECOVERY. This transition should occur in two situations:
         *  1. low enough altitude, incorrect orientation, etc. Unknown
         *  regulatory failure that requires immediate de-actuation; and
         *  2. the rocket, when at the base Cd, is found to be at or below the
         *  target apogee.
         */
        break;
    case state::IDLE_RECOVERY:
        /**
         * There should be no state transitions out of this state. This
         * state should remain until the flight controller is restarted. 
         */
        break;
    }
}

void airbrakes_state::refresh_imu()
{
    imu.update();
}

void airbrakes_state::refresh_baro()
{
    baro.update();
}
