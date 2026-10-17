#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "nav_wire.h"
#include "vision_wire.h"
typedef struct {
    float vx,vy,wz,yaw_rad,pitch_rad;
    uint8_t nav_valid,vision_valid,tracking,shoot_requested;
} SentryLinkSnapshot;
/* Watch this symbol: communication snapshot only, not a motor command. */
extern SentryLinkSnapshot sentry_link_snapshot;
extern volatile uint32_t nav_rx_frames,nav_tx_frames,nav_invalid_commands;
void NavInit(void);
uint8_t NavGetCommand(NavCommand *out);
void NavSetImu(const float wxyz[4],const float gyro[3]);
/* Actual measured chassis velocity only. Default is zero on bare board. */
void NavSetMotionFeedback(float vx,float vy,float wz);
void NavTask(uint8_t vision_valid,const VisionCommand *vision);
#endif
