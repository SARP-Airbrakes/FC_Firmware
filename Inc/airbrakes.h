
#ifndef AIRBRAKES_H_
#define AIRBRAKES_H_

#include <stdint.h>

typedef void *airbrakes_state_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

airbrakes_state_handle_t airbrakes_initialize(void);
void airbrakes_selftest(void);
void airbrakes_flash_driver_update(airbrakes_state_handle_t handle);

void airbrakes_serial_receive(uint8_t *buf, uint32_t *len);

#ifdef __cplusplus
} // exterm "C"
#endif

#endif // AIRBRAKES_H_
