#include <assert.h>
#include <stdio.h>
#include "../modules/navigation/navigation.c"
static uint32_t tick,mask;
static uint8_t rxbuf[64],*pending;
static uint16_t pending_len;
static void (*rx_callback)(uint16_t);
static USBD_CDC_HandleTypeDef cdc;
USBD_HandleTypeDef hUsbDeviceFS={USBD_STATE_CONFIGURED,&cdc};
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask=1; }
void __set_PRIMASK(uint32_t m) { mask=m; }
uint8_t *USBInit(USB_Init_Config_s conf) { rx_callback=conf.rx_cbk; return rxbuf; }
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n)
{ assert(!cdc.TxState); pending=p; pending_len=n; cdc.TxState=1; return USBD_OK; }
static void feed(const uint8_t *p,size_t n) { memcpy(rxbuf,p,n); rx_callback(n); }
int main(void)
{
    NavInit(); NavCommand c;
    float q[4]={.70710678f,.70710678f,0,0},gyro[3]={1,2,3};
    NavSetImu(q,gyro);
    assert(fabsf(imu[2]-1.5707963f)<1e-5 && fabsf(imu[1])<1e-5 && imu[3]==3);
    float identity[4]={1,0,0,0}; NavSetImu(identity,gyro);
    assert(!NavGetCommand(&c) && c.vx==0);
    uint8_t p[49],payload[39]={0}; float speeds[3]={.25f,-.5f,1};
    memcpy(payload,speeds,12); memset(payload+12,0xff,27); /* Other commands must be ignored. */
    NavPack(p,1,0,payload,39); assert(p[3]==0x7f);
    feed(p,17); assert(!NavGetCommand(&c)); feed(p+17,32);
    assert(NavGetCommand(&c) && c.vx==.25f && c.wz==1);
    VisionCommand vision={1,1,.7f,-.2f}; tick=10; NavTask(1,&vision);
    assert(sentry_link_snapshot.nav_valid && sentry_link_snapshot.tracking);
    assert(sentry_link_snapshot.yaw_rad==.7f && sentry_link_snapshot.vx==.25f);
    uint8_t copy[64]; memcpy(copy,pending,pending_len);
    tick=20; NavTask(0,&vision); assert(!memcmp(copy,pending,pending_len));
    tick=100; assert(!NavGetCommand(&c) && c.vx==0);
    cdc.TxState=0; NavTask(0,&vision);
    assert(!sentry_link_snapshot.nav_valid && !sentry_link_snapshot.shoot_requested);
    assert(sentry_link_snapshot.yaw_rad==0);
    p[48]^=1; feed(p,49); assert(!NavGetCommand(&c)); p[48]^=1;
    feed(p,49); assert(NavGetCommand(&c));
    float invalid=NAN; memcpy(payload,&invalid,4); NavPack(p,1,0,payload,39); feed(p,49);
    assert(nav_invalid_commands==1);
    tick=200; assert(!NavGetCommand(&c));
    memcpy(payload,speeds,12); NavPack(p,1,0,payload,39);
    p[48]^=1; feed(p,49); p[48]^=1; feed(p,49); assert(NavGetCommand(&c));
    hUsbDeviceFS.dev_state=0; assert(!NavGetCommand(&c));
    /* HAL tick wraparound remains valid and preserves caller IRQ mask. */
    hUsbDeviceFS.dev_state=3; tick=0xfffffff0; feed(p,49); tick=20; mask=1;
    assert(NavGetCommand(&c) && mask==1); tick=100; assert(!NavGetCommand(&c));
    puts("PASS navigation module: dual-link snapshot, CRC, fragmentation, timeout, disconnect, DMA lifetime");
}
