
#include <airbrakes_state.hpp>

#include <sdk/timing.h>
#include <cd_controller.h>

airbrakes_state::airbrakes_state(bmi088 &&imu, bmp390 &&baro, cdpa1616d &&gps,
        w25q128jv &&flash, motor_controller &&servo) :
    imu(std::forward<bmi088>(imu)), baro(std::forward<bmp390>(baro)),
    gps(std::forward<cdpa1616d>(gps)), flash(std::forward<w25q128jv>(flash)),
    servo(std::forward<motor_controller>(servo))
{
    auto state = imu.copy_state();
    acceleration = state.acceleration_ms2;
    last_acceleration = state.acceleration_ms2;
}

std::optional<airbrakes_state::state> airbrakes_state::next() const
{
    switch (current_state) {
    case state::IDLE_PAD:
        /* 
         * TODO: There should be one state transition: IDLE_PAD -> IDLE_FLIGHT.
         * This transition should occur on launch - this could be as simple as
         * detecting a new acceleration (delta acceleration > 0?)
         */
        {
            vec3 diff = acceleration - last_acceleration;
            if (diff.length_sqr() > IDLE_FLIGHT_MIN_JERK)
                return state::IDLE_FLIGHT;
        }
        return std::nullopt;
    case state::IDLE_FLIGHT:
        /**
         * TODO: There should be one state transition: IDLE_FLIGHT ->
         * ACTIVE_FLIGHT. This transition should occur as soon as we detect that
         * the velocity drops below Mach 0.7.
         */
        if (velocity.length_sqr() < ACTIVE_FLIGHT_MAX_VELOCITY_SQR)
            return state::ACTIVE_FLIGHT;
        return std::nullopt;
    case state::ACTIVE_FLIGHT:
        /**
         * TODO: There should be one state transition: ACTIVE_FLIGHT ->
         * IDLE_RECOVERY. This transition should occur in two situations:
         *  1. low enough altitude, incorrect orientation, etc or unknown
         *  regulatory failure that requires immediate de-actuation; and
         *  2. the rocket, when at the base Cd, is found to be at or below the
         *  target apogee.
         */

        if (velocity.length_sqr() > ACTIVE_FLIGHT_MAX_VELOCITY_SQR || 
                altitude < IDLE_RECOVERY_MAX_ALTITUDE)
            return state::IDLE_RECOVERY;
        return std::nullopt;

    // Both of these states require manual input to exit.
    case state::UNARMED:
    case state::IDLE_RECOVERY:
        return std::nullopt;
    }
}


// TODO: implemenet flap deflection LUT
real get_flap_deflection(real cd) {
    return 0.0f;
}

void airbrakes_state::execute()
{
    // enforce closed state when we are not in active flight
    if (current_state != state::ACTIVE_FLIGHT) {
        servo.set_target_degrees(0);
    } else if (current_state == state::ACTIVE_FLIGHT) {
        real target_cd = cd_controller_solve(velocity.length_sqr(), altitude, target_altitude);
        real flap_deflection = get_flap_deflection(target_cd);
        servo.set_target_degrees(flap_deflection * MOTOR_DEGREE_PER_FLAP_DEGREE);
    }
}

void airbrakes_state::step()
{
    
    // Copy states and grab measurements from drivers.
    {
        auto imu_state = imu.copy_state();
        acceleration = imu_state.acceleration_ms2;
    }
    {
        time = get_tick_seconds();
        delta_time = time - last_time;

        // tick wrap-around
        if (delta_time < 0) {
            // TODO
        }

        // very simple integration for velocity
        // TODO: kalman filter for velocity and altitude estimation
        if (current_state == state::IDLE_FLIGHT || 
                current_state == state::ACTIVE_FLIGHT) {
            velocity += delta_time * acceleration;

            // really dumb integration tech for getting altitude
            altitude += 0.5f * delta_time * acceleration.length_sqr();
        }
    }

    // Switch the state if state transition is found.
    auto next_step = next();
    if (next_step.has_value())
        current_state = *next_step;

    // Execute the new state.
    execute();

    // Post-processing steps.
    {
        last_acceleration = acceleration;
        last_time = time;
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
