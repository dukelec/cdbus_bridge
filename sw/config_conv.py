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

import struct
from argparse import ArgumentParser
from collections import OrderedDict
import json

def_conf = {
    "magic_code": 0xcdcd,
    "bl_wait": 30,
    "mode": 0,
    "ser_idx": 1,
    "rs485_addr": {"net": 0, "mac": 254},
    "rs485_baudrate_low": 115200,
    "rs485_baudrate_high": 115200,
    "ttl_baudrate": 115200,
    "rs232_baudrate": 115200,
    "rpt_en": 1,
    "rpt_pkt_level": 1,
    "rpt_seq": 1,
    "rpt_multi": 0,
    "rpt_mac": 0,
    "rpt_addr": {"net": 0, "mac": 0}
}


def conf_from_bytes(b):
    c = {}
    c['rs485_addr'] = {}
    c['rpt_addr'] = {}
    c['magic_code'], c['bl_wait'], c['mode'], c['ser_idx'],             \
        c['rs485_addr']['net'], c['rs485_addr']['mac'],                 \
        c['rs485_baudrate_low'], c['rs485_baudrate_high'],              \
        c['ttl_baudrate'], c['rs232_baudrate'],                         \
        c['rpt_en'], c['rpt_pkt_level'], c['rpt_seq'], c['rpt_multi'],  \
        c['rpt_mac'], c['rpt_addr']['net'], c['rpt_addr']['mac']        \
        = struct.unpack("<HBBBBBIIIIBBBBBBB", b)
    return c

def conf_to_bytes(c):
    return struct.pack("<HBBBBBIIIIBBBBBBB", c['magic_code'],           \
        c['bl_wait'], c['mode'], c['ser_idx'],                          \
        c['rs485_addr']['net'], c['rs485_addr']['mac'],                 \
        c['rs485_baudrate_low'], c['rs485_baudrate_high'],              \
        c['ttl_baudrate'], c['rs232_baudrate'],                         \
        c['rpt_en'], c['rpt_pkt_level'], c['rpt_seq'], c['rpt_multi'],  \
        c['rpt_mac'], c['rpt_addr']['net'], c['rpt_addr']['mac'])


if __name__ == "__main__":
    parser = ArgumentParser(usage=__doc__)
    parser.add_argument('--to-json', action='store_true')
    parser.add_argument('--to-bin', action='store_true')
    parser.add_argument('--json', dest='json')
    parser.add_argument('--bin', dest='bin')
    args = parser.parse_args()

    if args.to_json:
        conf = def_conf
        try:
            with open(args.bin, 'rb') as f:
                _conf = conf_from_bytes(f.read())
                conf.update(_conf)
        except:
            print('use default conf')
        
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
