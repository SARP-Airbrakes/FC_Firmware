
#ifndef AIRBRAKES_STATE_H_
#define AIRBRAKES_STATE_H_

#include <optional>

#include <sdk/drivers/motor_controller.h>
#include <sdk/drivers/bmi088.h>
#include <sdk/drivers/bmp390.h>
#include <sdk/drivers/cdpa1616d.h>
#include <sdk/drivers/w25q128jv.h>

using namespace sdk;

/**
 * This class represents the Airbrakes controller state.
 */
struct airbrakes_state {
    
    // Minimum jerk for a transition from IDLE_PAD -> IDLE_FLIGHT
    static constexpr real IDLE_FLIGHT_MIN_JERK = 10.0f;
    // Maximum rocket velocity where we can actuate motor. Mach 0.7
    static constexpr real ACTIVE_FLIGHT_MAX_VELOCITY = 230.0f;
    static constexpr real ACTIVE_FLIGHT_MAX_VELOCITY_SQR =
        ACTIVE_FLIGHT_MAX_VELOCITY * ACTIVE_FLIGHT_MAX_VELOCITY;
    static constexpr real IDLE_RECOVERY_MAX_ALTITUDE = 1000.0f;
    // This value is the ratio between motor shaft turning over one (1) degree
    // of flap deflection.
    static constexpr real MOTOR_DEGREE_PER_FLAP_DEGREE = 0.0f;

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
     * Initializes the airbrakes controller state. Assumes that the drivers are
     * already properly initialized when moved.
     */
    airbrakes_state(bmi088 &&imu, bmp390 &&baro, cdpa1616d &&gps, w25q128jv
            &&flash, motor_controller &&servo);

    /**
     * Calculates the next state in the finite state machine based on the rocket
     * state. If we aren't going to transition this step, returns nullopt.
     */
    std::optional<state> next() const;

    /**
     * Executes the current state of the finite state machine.
     */
    void execute();

    /** Calls #next() to check for a new state, then calls #execute() */
    void step();

    /* updates internal driver states */
    void refresh_imu();
    void refresh_baro();

    state current_state;

    bmi088 imu;
    bmp390 baro;
    cdpa1616d gps;
    w25q128jv flash;
    motor_controller servo;

    real time;
    real last_time;
    real delta_time;
    real altitude;
    real target_altitude;

    vec3 velocity;
    vec3 acceleration;
    vec3 last_acceleration;
};

#endif // AIRBRAKES_STATE_H_
