"""Bare-CBoard USB + vision USART verification. Receive-only unless --send-test."""
import argparse
import pathlib
import struct
import sys
import time
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation/ros2/sentry_cboard_bridge'))
sys.path.insert(0,str(ROOT/'ele-con/host_tests'))
from sentry_cboard_bridge.wire import Parser, command
from windows_serial_probe import AttitudeParser, command as vision_command


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--usb',required=True,help='/dev/rm_nav or COM port')
    cli.add_argument('--vision',required=True,help='/dev/rm_serial or COM port')
    cli.add_argument('--seconds',type=float,default=10)
    cli.add_argument('--send-test',action='store_true',help='Bare board / motors disconnected only; sends small nonzero speed, never fire')
    args=cli.parse_args()
    if args.seconds<=0: cli.error('--seconds must be positive')
    import serial
    counts={2:0,0x1e:0,0xe0:0}; attitude=0; verified=set(); parser=Parser(); qparser=AttitudeParser()
    duration=max(args.seconds,4) if args.send_test else args.seconds
    with serial.Serial(args.usb,115200,timeout=0,write_timeout=.1) as usb, serial.Serial(args.vision,921600,timeout=0,write_timeout=.1) as vision:
        start=time.monotonic(); next_send=start; last_print=start
        try:
            while time.monotonic()-start<duration:
                now=time.monotonic(); elapsed=now-start
                if args.send_test and now>=next_send:
                    # Both active 0-2s; USB silent 2-3s; both silent >=3s.
                    if elapsed<2: usb.write(command(int(elapsed*1000),.2,-.1,.3))
                    if elapsed<3: vision.write(vision_command(True))
                    next_send=now+.01
                attitude+=len(qparser.feed(vision.read(min(vision.in_waiting,4096))))
                for kind,tick,payload in parser.feed(usb.read(min(usb.in_waiting,4096))):
                    if kind in counts: counts[kind]+=1
                    if kind!=0xe0 or len(payload)!=24: continue
                    vals=struct.unpack('<5f4B',payload)
                    vx,vy,wz,yaw,pitch,nv,vv,tr,shoot=vals
                    if args.send_test:
                        if .3<elapsed<1.9 and nv and vv and tr and not shoot and abs(vx-.2)<1e-5 and abs(yaw-.25)<1e-5:
                            verified.add('BOTH_LINKS')
                        if 2.3<elapsed<2.9 and not nv and vx==vy==wz==0 and vv and tr and not shoot:
                            verified.add('NAV_TIMEOUT_ONLY')
                        if elapsed>3.3 and not nv and not vv and not tr and not shoot and vx==vy==wz==yaw==pitch==0:
                            verified.add('BOTH_TIMEOUT')
                    if now-last_print>1:
                        print(f't={tick} nav=({vx:.3f},{vy:.3f},{wz:.3f}) vision=({yaw:.3f},{pitch:.3f}) valid={nv}/{vv} tracking={tr} fire={shoot}')
                        last_print=now
                time.sleep(.002)
        finally:
            if args.send_test:
                usb.write(command(0,0,0,0)); vision.write(vision_command(False))
    print(f'USB frames={counts}, USART attitude={attitude}, rejected={parser.errors}/{qparser.rejected}')
    if not counts[0xe0] or not attitude: raise SystemExit('FAIL: missing link snapshot or USART attitude')
    if args.send_test:
        print('Verified:',sorted(verified))
        if verified!={'BOTH_LINKS','NAV_TIMEOUT_ONLY','BOTH_TIMEOUT'}: raise SystemExit('FAIL: dual-link/timeout verification incomplete')
    print('PASS')

if __name__=='__main__': main()
