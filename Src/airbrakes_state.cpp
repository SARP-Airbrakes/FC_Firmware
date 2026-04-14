
#include <airbrakes_state.hpp>
#include <main.h>

#include <sdk/timing.h>
#include <cd_controller.h>

#include <array>
#include <cmath>

airbrakes_state::airbrakes_state(bmi088 &&imu, bmp390 &&baro, cdpa1616d &&gps,
        w25q128jv &&flash, motor_controller &&servo) :
    imu(std::forward<bmi088>(imu)), baro(std::forward<bmp390>(baro)),
    gps(std::forward<cdpa1616d>(gps)), flash(std::forward<w25q128jv>(flash)),
    servo(std::forward<motor_controller>(servo)), flight_packet_queue(8)
{
}

void airbrakes_state::init()
{
    auto baro_state = baro.copy_state();
    auto imu_state = imu.copy_state();
    reference_altitude = baro_state.altitude_meters;
    baro_altitude = baro_state.altitude_meters;
    last_baro_altitude = baro_state.altitude_meters;
    acceleration = imu_state.acceleration_ms2;
    last_acceleration = imu_state.acceleration_ms2;
}

std::optional<airbrakes_state::state> airbrakes_state::next()
{
    switch (current_state) {
    case state::IDLE_PAD:
        /* 
         * This transition should occur on launch.
         */
        if (acceleration.magnitude_sqr() != 0 &&
                filtered_acceleration.magnitude_sqr() > IDLE_FLIGHT_MIN_ACCEL * IDLE_FLIGHT_MIN_ACCEL && 
                fused_velocity > 15.0f)
            return state::IDLE_FLIGHT;
        return std::nullopt;
    case state::IDLE_FLIGHT:
        /**
         * This transition should occur as soon as we detect that the velocity
         * drops below Mach 0.7.
         */
        /*
        if (velocity.magnitude_sqr() < ACTIVE_FLIGHT_MAX_VELOCITY_SQR)
            return state::ACTIVE_FLIGHT;
        */
        if (filtered_acceleration.z < -10.0f)
            return state::ACTIVE_FLIGHT;
        return std::nullopt;
    case state::ACTIVE_FLIGHT:
        /**
         * This transition should occur when the rocket is at a low enough
         * altitude, incorrect orientation, etc or there was some unknown
         * regulatory failure that requires immediate de-actuation.
         */
        if (fused_velocity < -1.0f) {
            if (state_time != 0 && time - state_time >= 0.5f) {
                return state::IDLE_RECOVERY;
            } else if (state_time == 0) {
                state_time = time;
            }
        } else {
            state_time = 0;
        }

        // TODO: Also should transition when the rocket, when at the base Cd, is
        // found to be at or below the target apogee using the ballistic model.
        return std::nullopt;

    // This state require manual input to exit.
    case state::IDLE_RECOVERY:
        return std::nullopt;
    }
    return std::nullopt;
}

void airbrakes_state::switch_state(state new_state)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

    last_log = 0.0f; // log as soon as we switch states

    switch (new_state) {
    default:
        break;
    };
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
        real target_cd = cd_controller_solve(fused_velocity, baro_altitude - reference_altitude, TARGET_ALTITUDE);
        real flap_deflection = get_flap_deflection(target_cd);
        flap_target_degrees = flap_deflection;
        float motor_degrees = 8280.0f * M_1_PI * std::asinf(flap_deflection / 45.0f);
        servo.set_target_degrees(-motor_degrees);
    }

    // THIS SHOULD ALSO CHECK FOR TIME ON THE PAD
    if (current_state == state::IDLE_PAD /* && time <= TIME_THRESHOLD */)
        return;

    // Flight logging logic.
    real frequency = (current_state == state::ACTIVE_FLIGHT || 
            current_state == state::IDLE_FLIGHT) ? 
        ACTIVE_LOGGING_FREQ : 
        IDLE_LOGGING_FREQ;
    if (current_state == state::IDLE_RECOVERY) {
        frequency = 0.25f;
    }
    if (time - last_log > 1.0f / frequency) {
        log();
        last_log = time;
    }
}

void airbrakes_state::step()
{
    // Copy states and grab measurements from drivers.
    {
        auto imu_state = imu.copy_state();
        auto baro_state = baro.copy_state();
        baro_altitude = baro_state.altitude_meters;
        angular_velocity = imu_state.angular_velocity_ds;
        if (reference_altitude == 0) {
            reference_altitude = baro_altitude;
        }
        pressure = baro_state.pressure_pascals;
        temperature = baro_state.temperature_celsius;

        if (isnanf(last_baro_altitude))
            last_baro_altitude = baro_altitude;

        acceleration = imu_state.acceleration_ms2;
        filtered_acceleration = acceleration;
        filtered_acceleration.z -= 9.81;
    }

    time = get_tick_seconds();
    delta_time = time - last_time;

    float dvtime = time - last_vtime;
    if (dvtime >= 0.50f) {
        baro_velocity = (baro_altitude - last_baro_altitude) / dvtime;
        last_vtime = time;
        last_baro_altitude = baro_altitude;
    }

    fused_velocity = 0.95f * baro_velocity + 0.05f * velocity.z;

    if (current_state == state::IDLE_FLIGHT || 
            current_state == state::ACTIVE_FLIGHT) {
        velocity += delta_time * filtered_acceleration;

        // really dumb integration tech for getting altitude
        acc_altitude += velocity.z * delta_time + 0.5f * delta_time * delta_time * filtered_acceleration.z;
    }
    last_time = time;

    // Switch the state if state transition is found.
    auto next_step = next();
    if (next_step.has_value()) { 
        switch_state(*next_step);
        current_state = *next_step;
    }

    // Execute the new state.
    execute();
}

void airbrakes_state::refresh_imu()
{
    imu.update();
}

void airbrakes_state::refresh_baro()
{
    baro.update();
}

union flight_packet_bytes {
    uint8_t bytes[sizeof(airbrakes_state::flight_packet)];
    airbrakes_state::flight_packet packet;
};

airbrakes_state::flight_packet airbrakes_state::read_packet(int address)
{
    flight_packet_bytes bytes;
    flash.read(256 * address, bytes.bytes, sizeof(bytes));
    return bytes.packet;
}

void airbrakes_state::update_flash()
{
    flight_packet_bytes bytes;   
    bytes.packet = flight_packet_queue.pop();

    bytes.packet.packet_id = packet_count;
    uint32_t address = 256 * (packet_count++);

    flash.write(address, bytes.bytes, sizeof(bytes.bytes));
}

void airbrakes_state::log()
{
    flight_packet packet;
    packet.packet_id = 0;
    packet.time_s = get_tick_seconds();
    packet.accel_x_mps2 = acceleration.x;
    packet.accel_y_mps2 = acceleration.y;
    packet.accel_z_mps2 = acceleration.z;

    packet.ang_vel_x_ds = angular_velocity.x;
    packet.ang_vel_y_ds = angular_velocity.y;
    packet.ang_vel_z_ds = angular_velocity.z;

    packet.acc_altitude_m = acc_altitude;
    packet.baro_altitude_m = baro_altitude;
    packet.reference_altitude_m = reference_altitude;
    packet.agl_altitude_m = baro_altitude - reference_altitude;

    packet.acc_velocity_mps = velocity.z;
    packet.baro_velocity_mps = baro_velocity;
    packet.fused_velocity_mps = fused_velocity;

    packet.pressure_pascals = pressure;
    packet.temperature_c = temperature;
    packet.gps_altitude_m = 0;
    packet.current_state = current_state;
    packet.motor_target_degrees = servo.target_degrees;
    packet.motor_actual_degrees = servo.encoder.get_degrees();
    packet.motor_commanded_power = servo.commanded_power;
    packet.flap_target_degrees = flap_target_degrees;
    packet.fix_status = 0;

    flight_packet_queue.push_back(packet);
}
