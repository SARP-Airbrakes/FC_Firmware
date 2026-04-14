
#ifndef AIRBRAKES_STATE_H_
#define AIRBRAKES_STATE_H_

#include <optional>

#include <sdk/drivers/motor_controller.h>
#include <sdk/drivers/bmi088.h>
#include <sdk/drivers/bmp390.h>
#include <sdk/drivers/cdpa1616d.h>
#include <sdk/drivers/w25q128jv.h>

#include <cmath>

using namespace sdk;

/**
 * This class represents the Airbrakes controller state.
 */
struct airbrakes_state {
    
    // Minimum jerk for a transition from IDLE_PAD -> IDLE_FLIGHT
    static constexpr real IDLE_FLIGHT_MIN_ACCEL = 20.0f;
    // Maximum rocket velocity where we can actuate motor. Mach 0.7
    static constexpr real ACTIVE_FLIGHT_MAX_VELOCITY = 230.0f;
    static constexpr real ACTIVE_FLIGHT_MAX_VELOCITY_SQR =
        ACTIVE_FLIGHT_MAX_VELOCITY * ACTIVE_FLIGHT_MAX_VELOCITY;
    static constexpr real IDLE_RECOVERY_MAX_ALTITUDE = 1000.0f;
 
    static constexpr real TARGET_ALTITUDE = 1738.0f;
    static constexpr real TIME_THRESHOLD = 60.0f;

    // Frequency in which flight data is logged in idle modes, in Hz
    static constexpr real IDLE_LOGGING_FREQ = 2.0f;
    // Frequency in which flight data is logged in active mode, in Hz
    static constexpr real ACTIVE_LOGGING_FREQ = 20.0f;
    
    /** 
     * States of a finite state machine representing the Airbrakes actuation
     * requirements.
     */
    enum class state {
        IDLE_PAD, /* idle, on pad */
        IDLE_FLIGHT, /* idle, during flight */
        ACTIVE_FLIGHT, /* active control, during flight */
        IDLE_RECOVERY, /* idle, after flight (recov.) */
    };

    struct flight_packet {
        int packet_id; /* packet number */
        float time_s; /* time since flight computer boot */
        float accel_x_mps2; /* imu accel in x (board-relative) */
        float accel_y_mps2; /* imu accel in y (board-relative) */
        float accel_z_mps2; /* imu accel in z (board-relative) */

        float ang_vel_x_ds;
        float ang_vel_y_ds;
        float ang_vel_z_ds;

        float acc_altitude_m; /* altitude found by integrating accel */
        float baro_altitude_m; /* altitude found with pressure */
        float reference_altitude_m;
        float agl_altitude_m;

        float acc_velocity_mps;
        float baro_velocity_mps;
        float fused_velocity_mps;

        float pressure_pascals; /* pressure */
        float temperature_c; /* temperature */
        float gps_altitude_m; /* 0 */

        state current_state;
        float motor_target_degrees;
        float motor_actual_degrees;
        float motor_commanded_power;

        float flap_target_degrees;

        int fix_status;
    };

    /**
     * Initializes the airbrakes controller state. Assumes that the drivers are
     * already properly initialized when moved.
     */
    airbrakes_state(bmi088 &&imu, bmp390 &&baro, cdpa1616d &&gps, w25q128jv
            &&flash, motor_controller &&servo);

    /**
     * Calculates the next state in the finite state machine based on the rocket
     * state. If we aren't going to transition this step, returns nullopt.
     */
    std::optional<state> next();

    /** get some base values */
    void init();

    void switch_state(state new_state);

    /**
     * Executes the current state of the finite state machine.
     */
    void execute();

    /** Calls #next() to check for a new state, then calls #execute() */
    void step();

    /* updates internal driver states */
    void refresh_imu();
    void refresh_baro();
    void update_flash();

    flight_packet read_packet(int address);

    /** 
     * Commits the current rocket state to a new packet written to the flash.
     */
    void log();

    state current_state = state::IDLE_PAD;

    bmi088 imu;
    bmp390 baro;
    cdpa1616d gps;
    w25q128jv flash;
    motor_controller servo;

    sdk::queue<flight_packet> flight_packet_queue;

    int packet_count;

    real time;
    real last_time;
    real last_log;
    real delta_time;
    real acc_altitude;
    real pressure;
    real reference_altitude;
    real baro_altitude;
    real temperature;
    real last_baro_altitude = NAN;
    real baro_velocity;
    real target_altitude = 1738.0f;
    real flap_target_degrees;
    real fused_velocity;
    real state_time = 0;
    real last_vtime = 0;

    vec3 velocity;
    vec3 acceleration;
    vec3 angular_velocity;
    vec3 filtered_acceleration;
    vec3 last_acceleration;
};

#endif // AIRBRAKES_STATE_H_
