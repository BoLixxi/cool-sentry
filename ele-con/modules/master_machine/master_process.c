/* Step 1: USART transport only, no motor or shooter actuation. */
#include "master_process.h"
#include "vision_wire.h"
#include "robot_def.h"
#include <string.h>
#if !defined(VISION_USE_UART) || defined(VISION_USE_VCP)
#error "Step 1 requires VISION_USE_UART only"
#endif
static USARTInstance *port;
static VisionParser parser;
static Vision_Recv_s latest;
static uint32_t last_received;
static uint8_t have_command;
static float quaternion[4]; /* xyzw */
static uint8_t have_attitude;
static uint8_t tx[VISION_ATTITUDE_SIZE]; /* valid until DMA completes */
static uint32_t last_sent;

static uint32_t Lock(void) { uint32_t mask=__get_PRIMASK(); __disable_irq(); return mask; }
static void Unlock(uint32_t mask) { __set_PRIMASK(mask); }
static void DecodeVision(void)
{
    uint8_t frame[VISION_COMMAND_SIZE];
    for (uint16_t i=0; i<port->recv_len; ++i) {
        if (!VisionFeed(&parser,port->recv_buff[i],1,VISION_COMMAND_SIZE,frame)) continue;
        VisionCommand cmd;
        if (!VisionDecodeCommand(frame,&cmd)) continue;
        latest.yaw=cmd.yaw_rad; latest.pitch=cmd.pitch_rad;
        latest.fire_mode=cmd.shoot ? AUTO_FIRE : (cmd.control ? AUTO_AIM : NO_FIRE);
        latest.target_state=cmd.control ? (cmd.shoot ? READY_TO_FIRE : TARGET_CONVERGING) : NO_TARGET;
        latest.target_type=NO_TARGET_NUM; /* target class not transmitted in step 1 */
        last_received=HAL_GetTick(); have_command=1;
    }
}
Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle)
{
    USART_Init_Config_s conf={.recv_buff_size=VISION_RECV_SIZE,.usart_handle=handle,.module_callback=DecodeVision};
    port=USARTRegister(&conf);
    return &latest; /* legacy API; control tasks must use VisionGetCommand */
}
uint8_t VisionGetCommand(Vision_Recv_s *out)
{
    uint32_t mask=Lock();
    uint8_t valid=have_command && (uint32_t)(HAL_GetTick()-last_received)<100u;
    if (valid) *out=latest;
    else memset(out,0,sizeof(*out));
    Unlock(mask); return valid;
}
void VisionSetQuaternion(const float wxyz[4])
{
    float norm=0;
    for(int i=0;i<4;++i) { if(!isfinite(wxyz[i])) return; norm+=wxyz[i]*wxyz[i]; }
    if(norm<0.64f || norm>1.44f) return;
    uint32_t mask=Lock();
    quaternion[0]=wxyz[1]; quaternion[1]=wxyz[2]; quaternion[2]=wxyz[3]; quaternion[3]=wxyz[0];
    have_attitude=1; Unlock(mask);
}
void VisionSend(void)
{
    uint32_t now=HAL_GetTick();
    if(!port || !have_attitude || (uint32_t)(now-last_sent)<10u || !USARTIsReady(port)) return;
    /* INS task owns attitude update + send. No buffer overwrite while DMA is busy. */
    VisionPack(tx,2,0,quaternion,4);
    if(HAL_UART_Transmit_DMA(port->usart_handle,tx,sizeof(tx))==HAL_OK) last_sent=now;
}
/* Legacy setters intentionally do not affect step-1 wire fields. */
void VisionSetAltitude(float yaw,float pitch,float roll) { (void)yaw; (void)pitch; (void)roll; }
void VisionSetFlag(Enemy_Color_e color,Work_Mode_e mode,Bullet_Speed_e speed) { (void)color; (void)mode; (void)speed; }
