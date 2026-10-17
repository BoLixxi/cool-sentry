"""Windows/Linux real-board step-1 probe. Default: receive only. Requires pyserial."""
import argparse
import math
import struct
import time


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x31 if value & 128 else 0)) & 255
    return value


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
    return value


def command(control):
    header = b"\xa5\x0a\x00"
    # Flags=1 requests tracking only; flags=0 cancels. Never request firing.
    packet = header + bytes([crc8(header)]) + struct.pack(
        "<HHff", 1, int(control), 0.25 if control else 0, -0.5 if control else 0
    )
    return packet + struct.pack("<H", crc16(packet))


class AttitudeParser:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, incoming):
        self.buffer.extend(incoming)
        results = []
        while self.buffer:
            if self.buffer[0] != 0xa5:
                del self.buffer[0]
                continue
            if len(self.buffer) < 4:
                break
            if self.buffer[1:3] != b"\x12\x00" or crc8(self.buffer[:3]) != self.buffer[3]:
                self.rejected += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < 26:
                break
            packet = bytes(self.buffer[:26])
            if packet[4:8] != b"\x02\x00\x00\x00" or crc16(packet[:-2]) != struct.unpack("<H", packet[-2:])[0]:
                self.rejected += 1
                del self.buffer[0]
                continue
            del self.buffer[:26]
            q = struct.unpack("<4f", packet[8:24])
            norm = math.sqrt(sum(x * x for x in q))
            if not all(math.isfinite(x) for x in q) or not 0.8 <= norm <= 1.2:
                self.rejected += 1
                continue
            results.append(q)
        return results


def self_test():
    assert crc8(b"123456789") == 0xa2
    assert crc16(b"123456789") == 0x4b37
    head = b"\xa5\x12\x00"
    packet = head + bytes([crc8(head)]) + b"\x02\x00\x00\x00" + struct.pack("<4f", 0, 0, 0, 1)
    packet += struct.pack("<H", crc16(packet))
    parser = AttitudeParser()
    assert not parser.feed(b"noise" + packet[:9])
    assert parser.feed(packet[9:]) == [(0, 0, 0, 1)]
    bad = packet[:-1] + bytes([packet[-1] ^ 1])
    assert parser.feed(bad + packet * 2) == [(0, 0, 0, 1)] * 2
    assert len(command(True)) == 18 and command(True)[6] == 1
    print("PASS: CRC, fragmented/bad/burst attitude frames, tracking-only command")


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", help="COM port, e.g. COM5")
    cli.add_argument("--baud", type=int, default=921600)
    cli.add_argument("--seconds", type=float, default=10)
    cli.add_argument("--send-test", action="store_true", help="Send tracking-only requests; isolate motors before use")
    cli.add_argument("--self-test", action="store_true")
    args = cli.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.port or args.seconds <= 0:
        cli.error("provide --port and positive --seconds")
    import serial
    parser = AttitudeParser()
    count = 0
    latest = None
    last_phase = None
    start = time.monotonic()
    report = start
    next_send = start
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.2) as port:
        try:
            while time.monotonic() - start < args.seconds:
                now = time.monotonic()
                elapsed = now - start
                # Repeat: 2s track, 1s silence (watchdog), 2s explicit cancellation.
                phase = "TRACK" if elapsed % 5 < 2 else ("SILENCE" if elapsed % 5 < 3 else "CANCEL")
                if args.send_test and phase != last_phase:
                    print("TX phase:", phase)
                    last_phase = phase
                if args.send_test and phase != "SILENCE" and now >= next_send:
                    data = command(phase == "TRACK")
                    if port.write(data) != len(data):
                        raise RuntimeError("incomplete serial write")
                    next_send = now + 0.02
                received = parser.feed(port.read(min(max(port.in_waiting, 1), 1024)))
                count += len(received)
                if received:
                    latest = received[-1]
                if now - report >= 1:
                    print(f"RX valid={count}, rejected={parser.rejected}, latest xyzw={latest}")
                    report = now
        finally:
            if args.send_test:
                port.write(command(False))
    if not count:
        raise SystemExit("FAIL: no valid attitude. Check board running, wiring, baud and protocol.")
    print(f"Received {count} valid frames. Verify actual command/timeout states in the board debugger.")


if __name__ == "__main__":
    main()
