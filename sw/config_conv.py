#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <duke@dukelec.com>

"""CDBUS Bridge config format converter

convert config format between json and binary:
  ./config_conv.py --to-bin --json xxx.json --bin xxx.bin
  ./config_conv.py --to-json --bin xxx.bin --json xxx.json

use default config:
  ./config_conv.py --to-json --json xxx.json
  ./config_conv.py --to-bin --bin xxx.bin
"""

import copy
import struct
from argparse import ArgumentParser
from collections import OrderedDict
import json

def_conf = {
    "magic_code": 0xcdcd,           # uint16_t
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
        "addr": [0x80, 0x00, 0x00], # uint32_t
        "port": 20                  # uint16_t
    }
}


def conf_from_bytes(b):
    c = copy.deepcopy(def_conf)
    c['magic_code'] = struct.unpack("<H", b[0:2])
    c['bl_wait'] = struct.unpack("<B", b[2:3])
    c['ser_idx'] = struct.unpack("<B", b[4:5])
    c['rs485_net'] = struct.unpack("<B", b[5:6])
    c['rs485_mac'] = struct.unpack("<B", b[6:7])
    c['rs485_baudrate_low'] = struct.unpack("<I", b[8:12])
    c['rs485_baudrate_high'] = struct.unpack("<I", b[12:16])
    c['ttl_baudrate'] = struct.unpack("<I", b[16:20])
    c['rs232_baudrate'] = struct.unpack("<I", b[20:24])
    c['rpt_en'] = struct.unpack("<B", b[24:25])
    c['rpt_dst']['addr'] = list(b[28:31])
    c['rpt_dst']['port'] = struct.unpack("<H", b[32:34])
    return c

def conf_to_bytes(c):
    b = b''
    b += struct.pack("<H", c['magic_code'])
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
    b += bytes(c['rpt_dst']['addr']) + b'\x00'
    b += struct.pack("<H", c['rpt_dst']['port'])
    b += b'\x00' * 2
    
    assert len(b) == 36
    return b


if __name__ == "__main__":
    parser = ArgumentParser(usage=__doc__)
    parser.add_argument('--to-json', action='store_true')
    parser.add_argument('--to-bin', action='store_true')
    parser.add_argument('--json', dest='json')
    parser.add_argument('--bin', dest='bin')
    args = parser.parse_args()

    if args.to_json:
        try:
            with open(args.bin, 'rb') as f:
                conf = conf_from_bytes(f.read())
        except:
            print('use default conf')
            conf = def_conf
        
        with open(args.json, 'w') as outfile:
            print('save to config', args.json)
            print()
            print(json.dumps(conf, indent = 4))
            json.dump(conf, outfile, indent = 4)

    elif args.to_bin:
        try:
            with open(args.json, mode='r') as f:
                conf = json.loads(f.read(), object_pairs_hook=OrderedDict)
        except:
            print('use default conf')
            conf = def_conf
        
        with open(args.bin, 'wb') as f:
            print('save to binary', args.bin)
            print()
            print(json.dumps(conf, indent = 4))
            f.write(conf_to_bytes(conf))

    else:
        print(__doc__)
