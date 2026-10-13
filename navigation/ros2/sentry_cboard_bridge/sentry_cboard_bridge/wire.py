"""PolarBear 0x5A USB envelope; independent of ROS and serial libraries."""
import math
import struct


def crc8(data):
    c = 0xff
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0x8c if c & 1 else 0)
    return c


def crc16(data):
    c = 0xffff
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0x8408 if c & 1 else 0)
    return c


def pack(kind, tick, payload):
    header = bytes((0x5a, len(payload) + 4, kind))
    body = header + bytes((crc8(header),)) + struct.pack('<I', tick & 0xffffffff) + payload
    return body + struct.pack('<H', crc16(body))


def command(tick, vx, vy, wz):
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError('velocity must be finite')
    # Remaining fields (chassis/gimbal/shoot/tracking) are zero and ignored by MCU.
    return pack(1, tick, struct.pack('<3f', vx, vy, wz) + bytes(27))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        frames = []
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] != 0x5a:
                del self.buffer[0]
                continue
            if len(self.buffer) < 4:
                break
            length = self.buffer[1] + 6
            if length < 10 or length > 64 or crc8(self.buffer[:3]) != self.buffer[3]:
                self.errors += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc16(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
                self.errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:length]
            frames.append((frame[2], struct.unpack('<I', frame[4:8])[0], frame[8:-2]))
        return frames


class VelocityGate:
    def __init__(self, timeout=0.1, max_linear=2.5, max_angular=3.0):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, max_linear, max_angular)):
            raise ValueError('timeouts and limits must be positive and finite')
        self.timeout, self.max_linear, self.max_angular = timeout, max_linear, max_angular
        self.last = None
        self.velocity = (0.0, 0.0, 0.0)

    def update(self, values, now):
        if not all(math.isfinite(v) for v in values):
            self.last = None
            return False
        vx, vy, wz = values
        norm = math.hypot(vx, vy)
        scale = min(1.0, self.max_linear / norm) if norm else 1.0
        self.velocity = (vx * scale, vy * scale, max(-self.max_angular, min(self.max_angular, wz)))
        self.last = now
        return True

    def get(self, now):
        return self.velocity if self.last is not None and now - self.last < self.timeout else (0.0, 0.0, 0.0)

    def reset(self):
        self.last = None
