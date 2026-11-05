#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2026, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDBUS Bridge hardware version tool

Write the hardware version to the user system data (usd) of the at32, shown as
"HW: vX.Y[.Z]" in the info string after reboot, "HW: v6" while never written.
The usd only erases as a whole, the app of the bridge writes the rest back; the
bl can not write it.

Args:
  --ver VER           # hardware version, default: 6.2
  --read-only | -r    # only show it
  --dev DEV           # specify serial port, default: ttyACM0
  --baud BAUD         # set baudrate, default: 0xcdcd (configuration mode)
  --target-addr ADDR  # default: 00:00:ff
  --help    | -h      # this help message
  --verbose | -v      # debug level: verbose
  --debug   | -d      # debug level: debug
  --info    | -i      # debug level: info

Command example:

$ ./hw_ver.py --ver 6.2

"""

HW_VER_ADDR = 0x1ffff848 # same as fw_bridge / fw_bl: usd Data2..4

import sys, os
import re
import struct

sys.path.append(os.path.join(os.path.dirname(__file__), './pycdnet'))

from cdnet.utils.log import *
from cdnet.utils.cd_args import CdArgs
from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import *

args = CdArgs()
local_mac = int(args.get("--local-mac", dft="0x00"), 0)
dev_str = args.get("--dev", dft="ttyACM0")
baud = int(args.get("--baud", dft="0xcdcd"), 0)
target_addr = args.get("--target-addr", dft="00:00:ff")
ver_str = args.get("--ver", dft="6.2")
read_only = args.get("--read-only", "-r") != None


if args.get("--help", "-h") != None:
    print(__doc__)
    exit()

m = re.fullmatch(r'v?(\d+)\.(\d+)(?:\.(\d+))?', ver_str)
ver = tuple(int(x or 0) for x in m.groups()) if m else None
if not ver or max(ver) > 255:
    print(f'invalid version: {ver_str}')
    exit(-1)


if args.get("--verbose", "-v") != None:
    logger_init(logging.VERBOSE)
elif args.get("--debug", "-d") != None:
    logger_init(logging.DEBUG)
elif args.get("--info", "-i") != None:
    logger_init(logging.INFO)


dev = CDBusSerial(dev_str, baud=baud)
CDNetIntf(dev, mac=local_mac)
sock = CDNetSocket(('', 0x40))


def info_read():
    sock.clear()
    sock.sendto(b'', (target_addr, 1))
    ret, _ = sock.recvfrom(timeout=1)
    return ret.decode() if ret else None

def flash_read(addr, len_):
    sock.clear()
    sock.sendto(b'\x00' + struct.pack("<IB", addr, len_), (target_addr, 8))
    ret, _ = sock.recvfrom(timeout=1)
    if not ret or ret[0] != 0 or len(ret) != len_ + 1:
        print(f'flash read error at: 0x{addr:08x}')
        exit(-1)
    return ret[1:]

def flash_erase(addr, len_):
    sock.clear()
    sock.sendto(b'\x2f' + struct.pack("<II", addr, len_), (target_addr, 8))
    ret, _ = sock.recvfrom(timeout=1)
    return ret != None and ret[0] == 0

def flash_write(addr, dat):
    sock.clear()
    sock.sendto(b'\x20' + struct.pack("<I", addr) + dat, (target_addr, 8))
    ret, _ = sock.recvfrom(timeout=1)
    return ret != None and ret[0] == 0

def ver_fmt(v):
    return f'v{v[0]}.{v[1]}.{v[2]}' if v[2] else f'v{v[0]}.{v[1]}'

def read_ver():
    """same rules as get_hw_ver() in the fw: each byte followed by its inverse"""
    dat = flash_read(HW_VER_ADDR, 6)
    pairs = [dat[i:i+2] for i in range(0, 6, 2)]
    if all(p == b'\xff\xff' for p in pairs):
        cur, note = None, 'blank, HW: v6'
    elif all(p[1] == p[0] ^ 0xff for p in pairs):
        cur = tuple(p[0] for p in pairs)
        note = ver_fmt(cur)
    else:
        cur, note = None, 'broken, HW: v6'
    print(f'  usd @{HW_VER_ADDR:08x}: {dat.hex(" ")}  {note}')
    return cur, dat != b'\xff' * 6


info = info_read()
if not info:
    print(f'no reply from {target_addr}')
    exit(-1)
print(f'info: {info}')

cur, used = read_ver()
if read_only:
    exit()
if cur == ver:
    print(f'already {ver_fmt(ver)}, nothing to write')
    exit()
if '(bl)' in info:
    print('the bl can not write it, start the app first')
    exit(-1)

if used and not flash_erase(HW_VER_ADDR, 6):
    print('erase failed')
    exit(-1)
ok = flash_write(HW_VER_ADDR, bytes(b for v in ver for b in (v, v ^ 0xff)))
cur, _ = read_ver()
if not ok or cur != ver:
    print('write failed')
    exit(-1)
print('done, reboot to show it in the info string')
