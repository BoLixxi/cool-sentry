"""Verify step-1 command snapshots on a running board, without flashing."""
import ctypes as C
import importlib.util
import pathlib
import struct
import subprocess
import time
import serial

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('probe', ROOT / 'host_tests/windows_serial_probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
symbols = {}
nm = subprocess.check_output(['arm-none-eabi-nm', '-S', str(ROOT / 'build/basic_framework.elf')], text=True)
for line in nm.splitlines():
    fields = line.split()
    if len(fields) == 4:
        symbols[fields[3]] = (int(fields[0], 16), int(fields[1], 16))
assert symbols['vision_snapshot'][1] == 11
dll = C.CDLL(r'D:\ozone+jlink\JLink_V938\JLink_x64.dll')
dll.JLINKARM_Open.restype = C.c_char_p
dll.JLINKARM_ExecCommand.argtypes = [C.c_char_p, C.c_void_p, C.c_int]
dll.JLINKARM_ReadMemEx.argtypes = [C.c_uint32, C.c_uint32, C.c_void_p, C.c_uint32]
dll.JLINKARM_ReadMemEx.restype = C.c_int

def read(addr, size):
    buf = C.create_string_buffer(size)
    count = dll.JLINKARM_ReadMemEx(addr, size, buf, 0)
    if count != size:
        raise RuntimeError(f'Memory read failed: {addr:#x}, got {count}/{size}')
    return buf.raw

def snapshot():
    return struct.unpack('<BBBff', read(symbols['vision_snapshot'][0], 11))

def check(label, value, expected):
    print(label, dict(zip(('fire_mode', 'target_state', 'target_type', 'pitch', 'yaw'), value)), flush=True)
    assert value[:3] == expected[:3], (label, value, expected)
    assert all(abs(a-b) < 1e-5 for a,b in zip(value[3:], expected[3:])), (label, value, expected)
    print('PASS', label, flush=True)

try:
    error = dll.JLINKARM_Open()
    if error:
        raise RuntimeError(error.decode(errors='replace'))
    out = C.create_string_buffer(1024)
    status = dll.JLINKARM_ExecCommand(b'Device = STM32F407IG', out, len(out))
    if status < 0:
        raise RuntimeError(out.value)
    dll.JLINKARM_TIF_Select(1)
    dll.JLINKARM_SetSpeed(4000)
    if dll.JLINKARM_Connect() < 0:
        raise RuntimeError('J-Link connection failed; ensure Ozone has released the probe')
    binary = (ROOT / 'build/basic_framework.bin').read_bytes()
    for offset in (0, 0xe988, 0xec58):
        assert read(0x08000000 + offset, 32) == binary[offset:offset+32], 'Flash/ELF mismatch'
    print('PASS Flash matches current firmware at vector and vision functions', flush=True)
    if dll.JLINKARM_IsHalted():
        raise RuntimeError('CPU is halted; resume board execution before this test')
    with serial.Serial('COM7', 921600, timeout=0.01, write_timeout=0.2) as port:
        def send(control):
            frame = probe.command(control)
            assert port.write(frame) == len(frame)
        def track(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                send(True)
                time.sleep(0.02)
        try:
            track(0.5)
            check('TRACK', snapshot(), (2, 1, 0, -0.5, 0.25))
            time.sleep(0.16)
            check('SILENCE after 160ms', snapshot(), (0, 0, 0, 0., 0.))
            track(0.3)
            check('TRACK restored', snapshot(), (2, 1, 0, -0.5, 0.25))
            send(False)
            time.sleep(0.02)
            check('CANCEL after 20ms', snapshot(), (0, 0, 0, 0., 0.))
        finally:
            send(False)
    print('PASS real-board command decoding, timeout and cancellation; shoot never requested', flush=True)
finally:
    dll.JLINKARM_Close()
