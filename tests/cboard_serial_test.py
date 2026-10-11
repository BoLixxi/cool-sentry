"""Linux PTY integration test; no camera or CBoard required."""
import os
import pty
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time


def run_case(probe, frame_size, seasky=False):
    master, slave = pty.openpty()
    process = None
    thread = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
            config.write(
                f"serial_port: {os.ttyname(slave)}\n"
                f"receive_frame_size: {frame_size}\n"
                f"serial_protocol: {'seasky_v1' if seasky else 'legacy19'}\n"
                "baudrate: 115200\nserial_timeout_ms: 300\n"
                "max_imu_age_ms: 100\n"
            )
            config.flush()
            process = subprocess.Popen(
                [probe, config.name], stdout=subprocess.PIPE, stderr=subprocess.STDOUT
            )
            def make_frame(values):
                data = b"\x55\xaa" + struct.pack("<4f", *values)
                if seasky:
                    return sea_frame(2, 0, struct.pack("<4f", *values))
                return data + bytes([sum(data) & 255]) if frame_size == 19 else data

            frame = make_frame((0, 0, 0, 1))

            def feed():
                time.sleep(0.05)
                for values in [(0, 0, 0, 0), (0, 0, 0, 10), (0, 0, 0, float("nan"))]:
                    os.write(master, b"noise" + make_frame(values))
                if frame_size == 19 or seasky:
                    # Plausible quaternion with an invalid checksum must not start the board.
                    bad = make_frame((1, 0, 0, 0))
                    bad = bad[:-1] + bytes([bad[-1] ^ 1])
                    for _ in range(10):
                        os.write(master, bad)
                        time.sleep(0.005)
                    assert not select.select([master], [], [], 0)[0], "bad checksum accepted"
                # Large bursts exercise bounded reads and multiple frames per read.
                os.write(master, frame * 100)
                for split in range(1, len(frame)):
                    os.write(master, frame[:split])
                    time.sleep(0.002)
                    os.write(master, frame[split:])
                    time.sleep(0.005)

            thread = threading.Thread(target=feed)
            thread.start()
            packet = b""
            deadline = time.monotonic() + 2
            while len(packet) < (18 if seasky else 16) and time.monotonic() < deadline:
                if select.select([master], [], [], 0.1)[0]:
                    packet += os.read(master, 1024)
            output = process.communicate(timeout=4)[0].decode()
            thread.join()
            expected = b"\xaa\x55\x01" + struct.pack("<hhff", 1, 0, 0.25, -0.5)
            expected += bytes([sum(expected[2:]) & 255])
            if seasky:
                expected = sea_frame(1, 1, struct.pack("<2f", 0.25, -0.5))
            assert packet == expected, (packet, expected)
            assert process.returncode == 0, output
            for marker in ["IMU_OK", "STALE_OK", "TIMEOUT_OK"]:
                assert marker in output, output
            print(f"PASS: {'SeaSky' if seasky else str(frame_size) + '-byte'} RX, fragments/bursts/noise, TX checksum, stale/timeout")
    finally:
        if thread is not None:
            thread.join()
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)


def sea_frame(command, flags, payload):
    head = b"\xa5" + struct.pack("<H", len(payload) + 2)
    c = 0
    for byte in head:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ (0x31 if c & 128 else 0)) & 255
    data = head + bytes([c]) + struct.pack("<HH", command, flags) + payload
    c = 0xffff
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = (c >> 1) ^ (0xa001 if c & 1 else 0)
    return data + struct.pack("<H", c)


if __name__ == "__main__":
    for size in [18, 19]:
        run_case(sys.argv[1] if len(sys.argv) > 1 else "./build_clean/cboard_serial_probe", size)

    run_case(sys.argv[1] if len(sys.argv) > 1 else "./build_clean/cboard_serial_probe", 19, True)
