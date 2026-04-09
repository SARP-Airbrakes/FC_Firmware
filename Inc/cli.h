
#ifndef CLI_H_
#define CLI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void cli_receive(char *buf, uint32_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CLI_H_ */
