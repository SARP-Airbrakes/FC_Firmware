
#include <airbrakes_state.hpp>

#include <sdk/timing.h>
#include <cd_controller.h>

struct flight_packet {
    int packet_id;
    float time_s;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float altitude_m;
    float pressure_pascals;
    float temperature_c;
    float gps_altitude_m;
    int fix_status;
};

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
         * This transition should occur on launch.
         */
        {
            vec3 diff = acceleration - last_acceleration;
            if (diff.length_sqr() > IDLE_FLIGHT_MIN_JERK)
                return state::IDLE_FLIGHT;
        }
        return std::nullopt;
    case state::IDLE_FLIGHT:
        /**
         * This transition should occur as soon as we detect that the velocity
         * drops below Mach 0.7.
         */
        if (velocity.length_sqr() < ACTIVE_FLIGHT_MAX_VELOCITY_SQR)
            return state::ACTIVE_FLIGHT;
        return std::nullopt;
    case state::ACTIVE_FLIGHT:
        /**
         * This transition should occur when the rocket is at a low enough
         * altitude, incorrect orientation, etc or there was some unknown
         * regulatory failure that requires immediate de-actuation.
         */
        if (velocity.length_sqr() > ACTIVE_FLIGHT_MAX_VELOCITY_SQR || 
                altitude < IDLE_RECOVERY_MAX_ALTITUDE)
            return state::IDLE_RECOVERY;

        // TODO: Also should transition when the rocket, when at the base Cd, is
        // found to be at or below the target apogee using the ballistic model.
        return std::nullopt;

    // Both of these states require manual input to exit.
    case state::UNARMED:
    case state::IDLE_RECOVERY:
        return std::nullopt;
    }
}

static constexpr std::array<std::pair<int, float>, 10> FLAP_DEFLECTION_TO_CD = {{
    { 0, 0.661, },
    { 5, 0.661, },
    { 10, 0.764, },
    { 15, 0.8775, },
    { 20, 1.042, },
    { 25, 1.167, },
    { 30, 1.322, },
    { 35, 1.485, },
    { 40, 1.630, },
    { 45, 1.834, },
}};

// TODO: implemenet flap deflection LUT
real get_flap_deflection(real target_cd) {
    if (target_cd < FLAP_DEFLECTION_TO_CD.front().second)
        return 0;
    if (target_cd > FLAP_DEFLECTION_TO_CD.back().second)
        return 45;
    for (size_t i = 0; i < FLAP_DEFLECTION_TO_CD.size() - 1; i++) {
        const std::pair<int, float> &first_pair = FLAP_DEFLECTION_TO_CD[i];
        const std::pair<int, float> &second_pair = FLAP_DEFLECTION_TO_CD.at(i + 1);
        if (first_pair.second <= target_cd && second_pair.second >= target_cd) {
            return static_cast<float>(first_pair.first) +
                (static_cast<float>(second_pair.first) -
                 static_cast<float>(first_pair.first)) * ((target_cd -
                     first_pair.second) / (second_pair.second -
                         first_pair.second));
        }
    }
    return 0;
}

void airbrakes_state::execute()
{
    // Actual air-brakes control logic.
    if (current_state != state::ACTIVE_FLIGHT) {
        // Enforce closed state when we are not in active flight
        servo.set_target_degrees(0);
    } else if (current_state == state::ACTIVE_FLIGHT) {
        real target_cd = cd_controller_solve(velocity.length_sqr(), altitude, target_altitude);
        real flap_deflection = get_flap_deflection(target_cd);
        servo.set_target_degrees(flap_deflection * MOTOR_DEGREE_PER_FLAP_DEGREE);
    }

    // Flight logging logic.
    if (current_state != state::UNARMED) {
        real frequency = current_state == state::ACTIVE_FLIGHT ? 
            ACTIVE_LOGGING_FREQ : 
            IDLE_LOGGING_FREQ;
        if (time - last_log > 1.0f / frequency) {
            log();
            last_log = time;
        }
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

void airbrakes_state::log()
{
    // TODO
}
