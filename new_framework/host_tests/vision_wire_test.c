#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../modules/master_machine/vision_wire.h"

int main(void)
{
    uint8_t packet[VISION_COMMAND_SIZE] = {0};
    float values[2] = {0.25f, -0.5f};
    VisionPack(packet, 1, 3, values, 2);
    VisionParser parser = {0};
    uint8_t decoded[VISION_COMMAND_SIZE] = {0};
    for (size_t i = 0; i < sizeof(packet); ++i)
        assert(VisionFeed(&parser, packet[i], 1, sizeof(packet), decoded) == (i + 1 == sizeof(packet)));
    VisionCommand command = {0};
    assert(VisionDecodeCommand(decoded, &command));
    assert(command.control == 1 && command.shoot == 1);
    assert(command.yaw_rad == values[0] && command.pitch_rad == values[1]);
    decoded[sizeof(decoded) - 1] ^= 1;
    parser.used = 0;
    for (size_t i = 0; i < sizeof(decoded); ++i)
        assert(VisionFeed(&parser, decoded[i], 1, sizeof(decoded), decoded) == 0);
    for (int burst = 0; burst < 100; ++burst) {
        assert(!VisionFeed(&parser, 0xa5, 1, sizeof(packet), decoded));
        assert(!VisionFeed(&parser, 0xff, 1, sizeof(packet), decoded));
        assert(!VisionFeed(&parser, 0xff, 1, sizeof(packet), decoded));
        for (size_t i=0; i<sizeof(packet); ++i)
            assert(VisionFeed(&parser,packet[i],1,sizeof(packet),decoded)==(i+1==sizeof(packet)));
    }
    const uint8_t sample[]="123456789";
    assert(VisionCRC16(sample,9)==0x4b37);
    assert(VisionCRC8(sample,9)==0xa2);
    puts("PASS vision SeaSky framing, CRC and resynchronization");
    return 0;
}
