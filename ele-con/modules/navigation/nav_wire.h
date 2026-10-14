#ifndef NAV_WIRE_H
#define NAV_WIRE_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
/* PolarBear USB protocol. CRC covers SOF/header and entire frame respectively. */
#define NAV_COMMAND_SIZE 49u
#define NAV_MAX_FRAME 64u
_Static_assert(sizeof(float)==4, "wire requires float32");
typedef struct { uint8_t bytes[NAV_MAX_FRAME]; size_t used; } NavParser;
typedef struct { float vx, vy, wz; uint32_t timestamp; } NavCommand;
static inline uint8_t NavCRC8(const uint8_t *p,size_t n)
{
    uint8_t c=0xff;
    while(n--) { c^=*p++; for(int i=0;i<8;++i) c=(c&1)?(uint8_t)((c>>1)^0x8c):(uint8_t)(c>>1); }
    return c;
}
static inline uint16_t NavCRC16(const uint8_t *p,size_t n)
{
    uint16_t c=0xffff;
    while(n--) { c^=*p++; for(int i=0;i<8;++i) c=(c&1)?(uint16_t)((c>>1)^0x8408):(uint16_t)(c>>1); }
    return c;
}
static inline size_t NavPack(uint8_t *p,uint8_t id,uint32_t tick,const void *data,size_t len)
{
    size_t n=len+10;
    p[0]=0x5a; p[1]=(uint8_t)(len+4); p[2]=id; p[3]=NavCRC8(p,3);
    memcpy(p+4,&tick,4); memcpy(p+8,data,len);
    uint16_t crc=NavCRC16(p,n-2); p[n-2]=(uint8_t)crc; p[n-1]=(uint8_t)(crc>>8);
    return n;
}
static inline uint8_t NavFeed(NavParser *s,uint8_t b,uint8_t *out)
{
    s->bytes[s->used++]=b;
    while(s->used) {
        uint8_t *p=s->bytes;
        int bad=p[0]!=0x5a;
        if(s->used>=2) bad|=p[1]!=43;
        if(s->used>=3) bad|=p[2]!=1;
        if(s->used>=4) bad|=p[3]!=NavCRC8(p,3);
        if(!bad && s->used<NAV_COMMAND_SIZE) return 0;
        if(!bad) {
            uint16_t crc=NavCRC16(p,47);
            if(p[47]==(uint8_t)crc && p[48]==(uint8_t)(crc>>8)) {
                memcpy(out,p,NAV_COMMAND_SIZE); s->used=0; return 1;
            }
        }
        --s->used; memmove(p,p+1,s->used);
    }
    return 0;
}
static inline uint8_t NavDecode(const uint8_t *p,NavCommand *out)
{
    float v[3]; memcpy(v,p+8,sizeof(v));
    for(int i=0;i<3;++i) if(!isfinite(v[i])) return 0;
    memcpy(&out->timestamp,p+4,4);
    out->vx=v[0]; out->vy=v[1]; out->wz=v[2];
    return 1; /* USB gimbal/shoot bytes deliberately do not enter vision control. */
}
#endif
