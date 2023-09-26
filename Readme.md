## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge_v4.jpg">

The CDBUS Bridge is a USB virtual serial port to RS485 converter by default, and the RS485 packets are in CDBUS format.

 - The USB serial communication rate is not affected by the selected baud rate, and the specified baud rate is ignored when the USB serial port is opened.
 - RS485 baud rate needs to be configured using the CDBUS GUI tool, since the hardware supports dual baud rate mode, the baud rate specified when opening the USB serial port is not used.
 - RS485 baud rate can be temporarily set to 115200 single-rate mode via S2 of the left switch to facilitate quick connection to devices with default baud rate.
 - S1 of the left switch switches bootloader and app mode and off to app mode.
 - 2 RS485 ports internal straight-through, for easy wiring.
 - The right switch S1 is the pull-up enable on the A wire of RS485, S2 is the termination resistor enable between AB, S3 is the pull-down enable on the B wire, and S4 is the USB to external power enable switch.


## GUI Configuration

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

There are two ways to configure the Bridge:
1. Configuration in bootloader stage.
2. In the app stage, when you open the serial port, specify the baud rate as `52685` (`0xcdcd`) to enter the configuration mode.

The second way is convenient to check the status statistics after power on.  
When configuring the Bridge, the target address should be set to `80:00:fe`.


After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.

If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.

<img src="doc/img/cdgui2.png">


## Download Source Code

```
git clone --recurse-submodules https://github.com/dukelec/cdbus_bridge
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

