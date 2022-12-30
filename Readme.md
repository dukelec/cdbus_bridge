## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge_v4.jpg">

The CDBUS Bridge is a USB virtual serial port to RS485 converter by default, and the RS485 packets are in CDBUS format.

 - The USB serial communication rate is not affected by the selected baud rate, and the specified baud rate is ignored when the USB serial port is opened.
 - RS485 baud rate needs to be configured using the CDBUS GUI tool, since the hardware supports dual baud rate mode, the baud rate specified when opening the USB serial port is not used.
 - RS485 baud rate can be temporarily set to 115200 single-rate mode via S2 of the left switch to facilitate quick connection to devices with default baud rate.
 - S1 of the left switch switches bootloader and app mode and off to app mode.
 - 2 RS485 ports internal straight-through, for easy wiring.
 - The right switch S1 is the pull-up enable on the A wire of RS485, S2 is the termination resistor enable between AB, S3 is the pull-down enable on the B wire, and S4 is the USB to external power enable switch.

<img alt="bridge_mode" src="doc/img/bridge_mode.svg">


## GUI Configuration

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

When configuring the Bridge as the target, do not select the CDBUS Bridge selection box, set the local MAC to 0xaa and the target address to 80:00:55.

<img src="doc/img/cdgui1.png">
<br><br>

After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.

If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.

<img src="doc/img/cdgui2.png">


## Download Source Code

```
git clone --recurse-submodules https://github.com/dukelec/cdbus_bridge.git
```

## Test

### Prepare
 - Linux: pip3 install pythoncrc pyserial
 - Mac: pip3 install readline pythoncrc pyserial
 - Windows: pip3 install pyreadline pythoncrc pyserial

Please refer scripts' `--help` message and the `Readme.md` under `sw/` folder, e.g.:

```
cd sw/cdbus_tools/
./cdbus_terminal.py --help
```

