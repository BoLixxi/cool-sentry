#ifndef VISION_WIRE_H
#define VISION_WIRE_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

/* SeaSky envelope, fixed lengths for step 1. No packed struct on the wire. */
#define VISION_COMMAND_SIZE 18u
#define VISION_ATTITUDE_SIZE 26u
typedef struct { uint8_t bytes[26]; size_t used; } VisionParser;
typedef struct { uint8_t control, shoot; float yaw_rad, pitch_rad; } VisionCommand;
static inline uint8_t VisionCRC8(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) { c ^= *p++; for (int i=0; i<8; ++i) c = (c & 128) ? (uint8_t)((c<<1)^0x31) : (uint8_t)(c<<1); }
    return c;
}
static inline uint16_t VisionCRC16(const uint8_t *p, size_t n)
{
    uint16_t c = 0xffff;
    while (n--) { c ^= *p++; for (int i=0; i<8; ++i) c = (c & 1) ? (uint16_t)((c>>1)^0xa001) : (uint16_t)(c>>1); }
    return c;
}
static inline void VisionPack(uint8_t *p, uint8_t id, uint16_t flags, const float *v, size_t count)
{
    size_t n = count*4+10;
    p[0]=0xa5; p[1]=(uint8_t)(count*4+2); p[2]=0; p[3]=VisionCRC8(p,3);
    p[4]=id; p[5]=0; p[6]=(uint8_t)flags; p[7]=(uint8_t)(flags>>8);
    memcpy(p+8,v,count*4);
    uint16_t c=VisionCRC16(p,n-2); p[n-2]=(uint8_t)c; p[n-1]=(uint8_t)(c>>8);
}
/* Consume one byte. Reject unknown length/ID immediately and slide on bad CRC. */
static inline uint8_t VisionFeed(VisionParser *s, uint8_t b, uint8_t id, size_t n, uint8_t *out)
{
    s->bytes[s->used++]=b;
    while (s->used) {
        uint8_t *p=s->bytes;
        int bad=p[0]!=0xa5;
        if(s->used>=3) bad |= p[1]!=n-8 || p[2]!=0;
        if(s->used>=4) bad |= p[3]!=VisionCRC8(p,3);
        if(s->used>=6) bad |= p[4]!=id || p[5]!=0;
        if(!bad && s->used<n) return 0;
        if(!bad) {
            uint16_t c=VisionCRC16(p,n-2);
            if(p[n-2]==(uint8_t)c && p[n-1]==(uint8_t)(c>>8)) {
                memcpy(out,p,n); s->used=0; return 1;
            }
        }
        --s->used; memmove(p,p+1,s->used);
    }
    return 0;
}
static inline uint8_t VisionDecodeCommand(const uint8_t *p, VisionCommand *out)
{
    /* Caller has verified envelope. Flags 0,1,3 only; shoot requires control. */
    if(p[7]!=0 || (p[6]!=0 && p[6]!=1 && p[6]!=3)) return 0;
    float v[2]; memcpy(v,p+8,sizeof(v));
    if(!isfinite(v[0]) || !isfinite(v[1])) return 0;
    out->control=p[6]&1; out->shoot=(p[6]>>1)&1;
    out->yaw_rad=v[0]; out->pitch_rad=v[1]; return 1;
}
#endif
