
#include <airbrakes_state.hpp>
#include <main.h>

#include <sdk/timing.h>
#include <cd_controller.h>

#include <array>
#include <cmath>

airbrakes_state::airbrakes_state(bmi088 &&imu, bmp390 &&baro, w25q128jv &&flash,
        motor_controller &&servo) :
    imu(std::forward<bmi088>(imu)), baro(std::forward<bmp390>(baro)),
    flash(std::forward<w25q128jv>(flash)),
    servo(std::forward<motor_controller>(servo)), flight_packet_queue(8)
{
    state_estimate = new filter();
}

void airbrakes_state::init()
{
    auto baro_state = baro.copy_state();
    reference_altitude_m = baro_state.altitude_meters;
    state_estimate->reinitialize(baro_state.altitude_meters);
}

void airbrakes_state::flight_packet::print_packet_header()
{
    printf("packet_id,"
           "time_s,"
           "accel_x_mps2,accel_y_mps2,accel_z_mps2,"
           "ang_vel_x_ds,ang_vel_y_ds,ang_vel_z_ds,"
           "baro_altitude_m,"
           "reference_altitude_m,"
           "agl_altitude_m,"
           "estimated_accel_x_mps2,estimated_accel_y_mps2,estimated_accel_z_mps2,"
           "estimated_altitude_m,"
           "estimated_upward_velocity_mps,"
           "pressure_pascals,"
           "temperature_c,"
           "current_state,"
           "motor_target_degrees,"
           "motor_actual_degrees,"
           "motor_commanded_power,"
           "flap_target_degrees\r\n");
}

void airbrakes_state::flight_packet::print_packet() const
{
    printf(
        "%d," /* packet id */
        "%.2f," /* time */
        "%.2f,%.2f,%.2f," /* acceleration */
        "%.2f,%.2f,%.2f," /* angular velocity */
        "%.2f," /* baro altitude */
        "%.2f," /* reference altitude */
        "%.2f," /* agl altitude */
        "%.2f,%.2f,%.2f," /* estimated acceleration */
        "%.2f," /* estimated altitude */
        "%.2f," /* estimated upward velocity */
        "%.2f," /* pressure */
        "%.2f," /* temperature */
        "%d," /* current state */
        "%.2f," /* motor target degrees */
        "%.2f," /* motor actual degrees */
        "%.2f," /* motor commanded power */
        "%.2f\r\n", /* flap target degrees */
        packet_id,
        time_s,
        accel_x_mps2,
        accel_y_mps2,
        accel_z_mps2,
        ang_vel_x_ds,
        ang_vel_y_ds,
        ang_vel_z_ds,
        baro_altitude_m,
        reference_altitude_m,
        agl_altitude_m,
        estimated_accel_x_mps2,
        estimated_accel_y_mps2,
        estimated_accel_z_mps2,
        estimated_altitude_m,
        estimated_upward_velocity_mps,
        pressure_pascals,
        temperature_c,
        (int) current_state,
        motor_target_degrees,
        motor_actual_degrees,
        motor_commanded_power,
        flap_target_degrees
    );
}

std::optional<airbrakes_state::state> airbrakes_state::next(const vec3 &filtered_acceleration_mps2)
{
    real vertical_velocity_mps = state_estimate->get_velocity();
    switch (current_state) {
    case state::IDLE_PAD:
        /* 
         * This transition should occur on launch.
         */
        if (filtered_acceleration_mps2.z > IDLE_FLIGHT_MIN_ACCEL && 
                vertical_velocity_mps > 15.0f)
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
        if (filtered_acceleration_mps2.z < -10.0f)
            return state::ACTIVE_FLIGHT;
        return std::nullopt;
    case state::ACTIVE_FLIGHT:
        /**
         * This transition should occur when the rocket is at a low enough
         * altitude, incorrect orientation, etc or there was some unknown
         * regulatory failure that requires immediate de-actuation.
         */
        if (vertical_velocity_mps < -1.0f) {
            return state::IDLE_RECOVERY;
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
    // Actual airbrakes control logic.
    if (current_state != state::ACTIVE_FLIGHT) {
        // Enforce closed state when we are not in active flight
        servo.set_target_degrees(0);
    } else if (current_state == state::ACTIVE_FLIGHT) {
        float upward_velocity_mps = state_estimate->get_velocity();
        float altitude_m = state_estimate->get_position();
        
        real target_cd = cd_controller_solve(
            upward_velocity_mps,
            altitude_m - reference_altitude_m,
            TARGET_ALTITUDE
        );

        real flap_deflection = get_flap_deflection(target_cd);
        real motor_degrees = 4361.0f - 652.0f * sqrtf(45.0f - flap_deflection);

        servo.set_target_degrees(-motor_degrees);

        packet.motor_actual_degrees = servo.encoder.get_degrees();
        packet.motor_target_degrees = motor_degrees;
        packet.motor_commanded_power = servo.commanded_power;
        packet.flap_target_degrees = flap_deflection;
    }

    // THIS SHOULD ALSO CHECK FOR TIME ON THE PAD
    if (false/* current_state == state::IDLE_PAD && time <= TIME_THRESHOLD */)
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
        if (force_log) {
            packet.print_packet();
        }
        log();
        last_log = time;
    }
}

void airbrakes_state::step()
{
    time = get_tick_seconds();
    real delta_time = time - last_time;

    if (delta_time < 1e-6)
        return;

    last_time = time;

    packet.time_s = time;

    // Copy states and grab measurements from drivers.
    auto imu_state = imu.copy_state();
    auto baro_state = baro.copy_state();

    packet.pressure_pascals = baro_state.pressure_pascals;
    packet.temperature_c = baro_state.temperature_celsius;

    packet.accel_x_mps2 = imu_state.acceleration_ms2.x;
    packet.accel_y_mps2 = imu_state.acceleration_ms2.y;
    packet.accel_z_mps2 = imu_state.acceleration_ms2.z;

    packet.ang_vel_x_ds = imu_state.angular_velocity_ds.x;
    packet.ang_vel_y_ds = imu_state.angular_velocity_ds.y;
    packet.ang_vel_z_ds = imu_state.angular_velocity_ds.z;

    packet.baro_altitude_m = baro_state.altitude_meters;
    packet.reference_altitude_m = reference_altitude_m;
    packet.agl_altitude_m = baro_state.altitude_meters - reference_altitude_m;

    auto raw_acceleration = filter::vec3(
        imu_state.acceleration_ms2.x,
        imu_state.acceleration_ms2.y,
        imu_state.acceleration_ms2.z
    );
    state_estimate->predict(delta_time, raw_acceleration);

    if (current_state == state::IDLE_PAD)
        state_estimate->correct_accelerometer(raw_acceleration);
    state_estimate->correct_barometer(baro_state.altitude_meters);

    auto eigen_filtered_acceleration_mps2 =
        state_estimate->get_filtered_acceleration(raw_acceleration);
    vec3 filtered_acceleration_mps2 = vec3 {
        eigen_filtered_acceleration_mps2(0),
        eigen_filtered_acceleration_mps2(1),
        eigen_filtered_acceleration_mps2(2)
    };

    packet.estimated_accel_x_mps2 = filtered_acceleration_mps2.x;
    packet.estimated_accel_y_mps2 = filtered_acceleration_mps2.y;
    packet.estimated_accel_z_mps2 = filtered_acceleration_mps2.z;

    packet.estimated_altitude_m = state_estimate->get_position();
    packet.estimated_upward_velocity_mps = state_estimate->get_velocity();

    // Switch the state if state transition is found.
    auto next_step = next(filtered_acceleration_mps2);
    if (next_step.has_value()) { 
        switch_state(*next_step);
        current_state = *next_step;
    }

    packet.current_state = current_state;

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
    // flight_packet_queue.push_back(packet);
    packet = flight_packet();
}
