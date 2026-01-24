
#ifndef AIRBRAKES_H_
#define AIRBRAKES_H_

#include <stdint.h>

typedef struct airbrakes_state *airbrakes_state_handle_t;

#ifdef __cplusplus

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

    void update();

    /* updates internal driver states */
    void refresh_imu();
    void refresh_baro();

    sdk::bmi088 imu;
    sdk::bmp390 baro;
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

extern airbrakes_state_handle_t state_handle;

void airbrakes_initialize(void);

void airbrakes_print_prompt(void);
void airbrakes_serial_receive(uint8_t *buf, uint32_t *len);

void airbrakes_serial_print(const char *buf);
void airbrakes_serial_printf(const char *format, ...);

void airbrakes_i2c_interrupt(void *hdmatx);

#ifdef __cplusplus
} // exterm "C"
#endif

#endif // AIRBRAKES_H_
