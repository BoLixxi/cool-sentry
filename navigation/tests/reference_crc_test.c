/* Build with the supplied chuanchaun CRC implementation as an independent oracle. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "nav_wire.h"
#include "CRC8_CRC16.h"
int main(void)
{
    unsigned char p[49],reference[49],payload[39];
    for(int n=0;n<256;++n) {
        for(int i=0;i<39;++i) payload[i]=(uint8_t)(n+i*7);
        NavPack(p,1,(uint32_t)n,payload,39); memcpy(reference,p,49);
        append_CRC8_check_sum(reference,4); append_CRC16_check_sum(reference,49);
        assert(!memcmp(p,reference,49));
    }
    puts("PASS 256 frames match supplied firmware CRC tables byte-for-byte");
}
