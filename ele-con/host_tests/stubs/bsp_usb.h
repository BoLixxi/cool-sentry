#ifndef HOST_BSP_USB_H
#define HOST_BSP_USB_H
#include <stdint.h>
#define USBD_STATE_CONFIGURED 3
#define USBD_OK 0
#define USBD_BUSY 1
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;
extern USBD_HandleTypeDef hUsbDeviceFS;
typedef struct { void (*tx_cbk)(uint16_t); void (*rx_cbk)(uint16_t); } USB_Init_Config_s;
uint8_t *USBInit(USB_Init_Config_s);
uint8_t CDC_Transmit_FS(uint8_t *,uint16_t);
#endif
