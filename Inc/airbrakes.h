
#ifndef AIRBRAKES_H_
#define AIRBRAKES_H_

#include <stdint.h>
#include <main.h>

#ifdef __cplusplus
#include <airbrakes_state.hpp>
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

typedef struct airbrakes_state *airbrakes_state_handle_t;
extern airbrakes_state_handle_t state_handle;

void airbrakes_initialize(void);

void airbrakes_print_prompt(void);
void airbrakes_cli_receive(uint8_t *buf, uint32_t *len);

void airbrakes_i2c_interrupt(void *hdmatx);

#ifdef __cplusplus
} // exterm "C"
#endif

#endif // AIRBRAKES_H_
