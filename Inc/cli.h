
#ifndef CLI_H_
#define CLI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void cli_init(void);
void cli_receive(uint8_t *buf, uint32_t len);
void cli_transmit_completed(void);
void cli_poll(void);
void cli_process_tx(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CLI_H_ */
