#!/bin/sh

pyocd flash -t at32f405kcu7_4 build/*.hex --pack=$AT32F405_DFP_PACK_FILE
