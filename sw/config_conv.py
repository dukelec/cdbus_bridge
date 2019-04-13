#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <duke@dukelec.com>

"""CDBUS Bridge config format converter

convert config format between cfg and binary:
  ./config_conv.py --to-bin --cfg xxx.cfg --bin xxx.bin
  ./config_conv.py --to-cfg --bin xxx.bin --cfg xxx.cfg

use default config:
  ./config_conv.py --to-cfg --cfg xxx.cfg
  ./config_conv.py --to-bin --bin xxx.bin
"""

import copy
import struct
import pprint
from unittest.mock import patch
from argparse import ArgumentParser

# https://stackoverflow.com/questions/51788397/can-i-avoid-a-sorted-dictionary-output-after-ive-used-pprint-pprint-in-python
def unsorted_pprint(*args, **kwargs):
    with patch('builtins.sorted', new=lambda l, **_: l):
        orig_pprint(*args, **kwargs)

pp = pprint.PrettyPrinter(width=80, compact=True)
orig_pprint = pprint.pprint
pp.pprint = unsorted_pprint


def_conf = {
    "magic_code": "cdcd",           # bcd, 2 bytes
    "bl_wait": 30,                  # uint8_t
    #"mode": 0,                     # enum (uint8_t), override by hw switch
    "ser_idx": 1,                   # enum (uint8_t)
    "rs485_net": 0,                 # uint8_t
    "rs485_mac": 0xdf,              # uint8_t
                                    # (pad 1 byte)
    "rs485_baudrate_low": 115200,   # uint32_t
    "rs485_baudrate_high": 115200,  # uint32_t
    "ttl_baudrate": 115200,         # uint32_t
    "rs232_baudrate": 115200,       # uint32_t
    "rpt_en": 1,                    # bool (uint8_t)
                                    # (pad 3 bytes)
    "rpt_dst": {
        "addr": "800000",           # bcd, 3 bytes
        "port": 20                  # uint16_t
    }
}


def conf_from_bytes(b):
    c = copy.deepcopy(def_conf)
    c['magic_code'] = b[0:2].hex()
    c['bl_wait'] = struct.unpack("<B", b[2:3])[0]
    c['ser_idx'] = struct.unpack("<B", b[4:5])[0]
    c['rs485_net'] = struct.unpack("<B", b[5:6])[0]
    c['rs485_mac'] = struct.unpack("<B", b[6:7])[0]
    c['rs485_baudrate_low'] = struct.unpack("<I", b[8:12])[0]
    c['rs485_baudrate_high'] = struct.unpack("<I", b[12:16])[0]
    c['ttl_baudrate'] = struct.unpack("<I", b[16:20])[0]
    c['rs232_baudrate'] = struct.unpack("<I", b[20:24])[0]
    c['rpt_en'] = struct.unpack("<B", b[24:25])[0]
    c['rpt_dst']['addr'] = b[28:31].hex()
    c['rpt_dst']['port'] = struct.unpack("<H", b[32:34])[0]
    return c

def conf_to_bytes(c):
    b = b''
    b += bytes.fromhex(c['magic_code'])
    b += struct.pack("<B", c['bl_wait'])
    b += b'\x00'
    b += struct.pack("<B", c['ser_idx'])
    b += struct.pack("<B", c['rs485_net'])
    b += struct.pack("<B", c['rs485_mac'])
    b += b'\x00'
    b += struct.pack("<I", c['rs485_baudrate_low'])
    b += struct.pack("<I", c['rs485_baudrate_high'])
    b += struct.pack("<I", c['ttl_baudrate'])
    b += struct.pack("<I", c['rs232_baudrate'])
    b += struct.pack("<B", c['rpt_en'])
    b += b'\x00' * 3
    b += bytes.fromhex(c['rpt_dst']['addr']) + b'\x00'
    b += struct.pack("<H", c['rpt_dst']['port'])
    b += b'\x00' * 2
    
    assert len(b) == 36
    return b


if __name__ == "__main__":
    parser = ArgumentParser(usage=__doc__)
    parser.add_argument('--to-cfg', action='store_true')
    parser.add_argument('--to-bin', action='store_true')
    parser.add_argument('--cfg', dest='cfg')
    parser.add_argument('--bin', dest='bin')
    args = parser.parse_args()

    if args.to_cfg:
        if args.bin:
            with open(args.bin, 'rb') as f:
                conf = conf_from_bytes(f.read())
        else:
            print('use default conf')
            conf = def_conf
        
        with open(args.cfg, 'w') as outfile:
            print('save to config', args.cfg)
            print()
            pp.pprint(conf)
            with patch('builtins.sorted', new=lambda l, **_: l):
                outfile.write(pp.pformat(conf))

    elif args.to_bin:
        if args.cfg:
            with open(args.cfg, mode='r') as f:
                conf = eval(f.read())
        else:
            print('use default conf')
            conf = def_conf
        
        with open(args.bin, 'wb') as f:
            print('save to binary', args.bin)
            print()
            pp.pprint(conf)
            f.write(conf_to_bytes(conf))

    else:
        print(__doc__)
