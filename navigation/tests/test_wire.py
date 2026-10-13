import pathlib
import sys
import unittest
import struct
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'ros2/sentry_cboard_bridge'))
from sentry_cboard_bridge.wire import Parser, VelocityGate, command, pack, crc8, crc16

class WireTest(unittest.TestCase):
    def test_known_crc(self):
        self.assertEqual(crc8(bytes.fromhex('5a2b01')), 0x7f)
        self.assertEqual(crc8(b'123456789'), 0x0b)
        self.assertEqual(crc16(b'123456789'), 0x6f91)

    def test_fragment_and_resync(self):
        good=command(42, .25, -.5, 1)
        bad=bytearray(good); bad[-1]^=1
        parser=Parser(); results=[]
        for b in b'noise'+bytes(bad)+good+good:
            results.extend(parser.feed(bytes([b])))
        self.assertEqual(len(results),2)
        self.assertEqual(len(good),49)
        self.assertEqual(results[0][:2],(1,42))
        self.assertEqual(struct.unpack('<3f',results[0][2][:12]),(.25,-.5,1))
        self.assertEqual(results[0][2][12:], bytes(27))

    def test_stale_and_reconnect(self):
        gate=VelocityGate()
        gate.update((.25,-.5,1),10)
        self.assertNotEqual(gate.get(10.05),(0,0,0))
        self.assertEqual(gate.get(10.101),(0,0,0))
        gate.update((1,1,1),11); gate.reset()
        self.assertEqual(gate.get(11),(0,0,0))
        gate.update((float('nan'),0,0),12)
        self.assertEqual(gate.get(12),(0,0,0))

    def test_limit_and_feedback(self):
        gate=VelocityGate(); gate.update((3,4,10),1)
        self.assertEqual(gate.get(1),(1.5,2,3))
        frame=pack(0x1e,100,struct.pack('<3f',0,0,0))
        self.assertEqual(len(frame),22)
        self.assertEqual(Parser().feed(frame)[0][0],0x1e)

if __name__=='__main__': unittest.main()
