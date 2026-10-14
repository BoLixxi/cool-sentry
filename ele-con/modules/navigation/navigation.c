#include "navigation.h"
#include "main.h"
#include "bsp_usb.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
static NavParser parser;
static uint8_t *rx;
static NavCommand latest;
static uint32_t last_rx,last_tx,imu_tick;
static uint8_t have_command,have_imu,tx_slot;
static float imu[6],motion[3];
static uint8_t tx[NAV_MAX_FRAME];
SentryLinkSnapshot sentry_link_snapshot;
volatile uint32_t nav_rx_frames,nav_tx_frames,nav_invalid_commands;
static uint32_t Lock(void) { uint32_t m=__get_PRIMASK(); __disable_irq(); return m; }
static void Unlock(uint32_t m) { __set_PRIMASK(m); }
static void Receive(uint16_t len)
{
    uint8_t frame[NAV_COMMAND_SIZE];
    for(uint32_t i=0;i<len;++i) {
        if(!NavFeed(&parser,rx[i],frame)) continue;
        NavCommand c;
        if(!NavDecode(frame,&c)) { ++nav_invalid_commands; continue; }
        latest=c; last_rx=HAL_GetTick(); have_command=1; ++nav_rx_frames;
    }
}
void NavInit(void)
{
    memset(&parser,0,sizeof(parser)); have_command=0;
    USB_Init_Config_s conf={.tx_cbk=NULL,.rx_cbk=Receive};
    rx=USBInit(conf);
}
uint8_t NavGetCommand(NavCommand *out)
{
    uint32_t m=Lock();
    uint8_t valid=have_command && (uint32_t)(HAL_GetTick()-last_rx)<100u &&
        hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED;
    if(valid) *out=latest; else memset(out,0,sizeof(*out));
    Unlock(m); return valid;
}
void NavSetImu(const float wxyz[4],const float gyro[3])
{
    /* Standard ROS ZYX Euler convention, derived from quaternion because the
       inherited INS Pitch/Roll labels differ from ROS rotation axes. */
    float norm=0;
    for(int i=0;i<4;++i) { if(!isfinite(wxyz[i])) return; norm+=wxyz[i]*wxyz[i]; }
    if(norm<0.64f || norm>1.44f) return;
    float scale=1.0f/sqrtf(norm);
    float w=wxyz[0]*scale,x=wxyz[1]*scale,y=wxyz[2]*scale,z=wxyz[3]*scale;
    float sin_pitch=2.0f*(w*y-z*x);
    sin_pitch=fmaxf(-1.0f,fminf(1.0f,sin_pitch));
    float yaw=atan2f(2.0f*(w*z+x*y),1.0f-2.0f*(y*y+z*z));
    float pitch=asinf(sin_pitch);
    float roll=atan2f(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y));
    float values[6]={yaw,pitch,roll,gyro[2],gyro[1],gyro[0]};
    for(int i=0;i<6;++i) if(!isfinite(values[i])) return;
    uint32_t m=Lock(); memcpy(imu,values,sizeof(imu)); have_imu=1; imu_tick=HAL_GetTick(); Unlock(m);
}
void NavSetMotionFeedback(float vx,float vy,float wz)
{
    float values[3]={vx,vy,wz};
    for(int i=0;i<3;++i) if(!isfinite(values[i])) return;
    uint32_t m=Lock(); memcpy(motion,values,sizeof(motion)); Unlock(m);
}
void NavTask(uint8_t vision_valid,const VisionCommand *vision)
{
    NavCommand c; uint8_t valid=NavGetCommand(&c);
    sentry_link_snapshot=(SentryLinkSnapshot){
        .vx=c.vx,.vy=c.vy,.wz=c.wz,.nav_valid=valid,.vision_valid=vision_valid,
        .yaw_rad=vision_valid?vision->yaw_rad:0,.pitch_rad=vision_valid?vision->pitch_rad:0,
        .tracking=vision_valid && vision->control,.shoot_requested=vision_valid && vision->shoot};
    uint32_t now=HAL_GetTick();
    if((uint32_t)(now-last_tx)<10u || hUsbDeviceFS.dev_state!=USBD_STATE_CONFIGURED || !hUsbDeviceFS.pClassData) return;
    USBD_CDC_HandleTypeDef *cdc=(USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if(cdc->TxState) return; /* Never overwrite a pending transfer buffer. */
    uint8_t payload[24],id; size_t count;
    uint32_t m=Lock();
    if(tx_slot==0 && have_imu && (uint32_t)(now-imu_tick)<100u) {
        id=2; count=24; memcpy(payload,imu,count);
    } else if(tx_slot==1) { id=0x1e; count=12; memcpy(payload,motion,count); }
    else {
        /* Bench diagnostic extension: speeds, vision angles, flags. Not odometry. */
        float values[5]={c.vx,c.vy,c.wz,sentry_link_snapshot.yaw_rad,sentry_link_snapshot.pitch_rad};
        memcpy(payload,values,20); payload[20]=valid; payload[21]=vision_valid;
        payload[22]=sentry_link_snapshot.tracking; payload[23]=sentry_link_snapshot.shoot_requested;
        id=0xe0; count=24;
    }
    Unlock(m);
    size_t n=NavPack(tx,id,now,payload,count);
    if(CDC_Transmit_FS(tx,(uint16_t)n)==USBD_OK) { ++nav_tx_frames; last_tx=now; tx_slot=(tx_slot+1)%3; }
}
