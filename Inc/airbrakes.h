
#ifndef AIRBRAKES_H_
#define AIRBRAKES_H_

typedef void *airbrakes_state_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

airbrakes_state_handle_t airbrakes_initialize(void);
void airbrakes_flash_driver_update(airbrakes_state_handle_t handle);

#ifdef __cplusplus
} // exterm "C"
#endif

#endif // AIRBRAKES_H_
