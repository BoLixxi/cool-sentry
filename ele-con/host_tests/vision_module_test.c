#include <assert.h>
#include <stdio.h>
#include "../modules/master_machine/master_process.c"
static USARTInstance instance;
static uint32_t now, irq_mask;
static uint8_t busy;
static uint8_t *pending_dma;
static uint16_t pending_size;
uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask=1; }
void __set_PRIMASK(uint32_t mask) { irq_mask=mask; }
USARTInstance *USARTRegister(USART_Init_Config_s *c)
{
    instance.usart_handle=c->usart_handle; instance.module_callback=c->module_callback;
    return &instance;
}
uint8_t USARTIsReady(USARTInstance *p) { (void)p; return !busy; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h; pending_dma=p; pending_size=n; busy=1; return HAL_OK; }
static void feed(const uint8_t *p,uint16_t n)
{ memcpy(instance.recv_buff,p,n); instance.recv_len=n; instance.module_callback(); }
int main(void)
{
    UART_HandleTypeDef h={0}; VisionInit(&h);
    Vision_Recv_s c;
    assert(!VisionGetCommand(&c) && c.fire_mode==NO_FIRE);
    uint8_t frame[18]; float v[2]={0.25f,-0.5f}; VisionPack(frame,1,3,v,2);
    feed(frame,5); assert(!VisionGetCommand(&c));
    feed(frame+5,13); assert(VisionGetCommand(&c) && c.fire_mode==AUTO_FIRE);
    assert(c.yaw==v[0] && c.pitch==v[1]);
    now=100; assert(!VisionGetCommand(&c) && c.target_state==NO_TARGET && c.fire_mode==NO_FIRE);
    frame[17]^=1; feed(frame,18); assert(!VisionGetCommand(&c)); frame[17]^=1;
    feed(frame,18); assert(VisionGetCommand(&c));
    VisionPack(frame,1,2,v,2); feed(frame,18); /* shoot without control is invalid */
    now=200; assert(!VisionGetCommand(&c));
    float q[4]={1,0,0,0}; VisionSetQuaternion(q); VisionSend();
    assert(pending_size==26); uint8_t copy[26]; memcpy(copy,pending_dma,26);
    q[0]=0; q[1]=1; VisionSetQuaternion(q); now=220; VisionSend();
    assert(!memcmp(copy,pending_dma,26)); /* busy DMA buffer unchanged */
    busy=0; VisionSend(); assert(memcmp(copy,pending_dma,26));
    irq_mask=1; VisionGetCommand(&c); assert(irq_mask==1);
    puts("PASS firmware module: fragmentation, CRC, timeout, coherent copy, DMA buffer lifetime");
}
