#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t gState; } UART_HandleTypeDef;
#define HAL_OK 0
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
