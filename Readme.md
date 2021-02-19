## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge.jpg">
<img alt="cdbus_bridge_pcb" src="doc/img/cdbus_bridge_pcb.jpg">


CDBUS Bridge has two types of communication ports, which are 3 serial ports and 2 RS485 ports:
 - Only one of the three serial ports will be in use at the same time. Inserting USB will switch over to the configuration-free USB serial port (the communication rate will not be affected by the selected baud rate);
 - The two RS485 ports are internally connected for easy wiring (or pull up, pull down, and termination resistors).

CDBUS Bridge has two modes (requires different firmware to be programmed, default is Bridge mode):

### Bridge Mode (Serial to RS485)

Switch Definition Changed: Forcing 115200 baud rate if the switch is in RAW when powering up.

<img alt="bridge_mode" src="doc/img/bridge_mode.svg">

### Raw Mode (Serial passthrough)

<img alt="raw_mode" src="doc/img/raw_mode.svg">


## GUI Configuration

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

When configuring the Bridge as the target, do not select the CDBUS Bridge selection box, set the local MAC to 0xaa and the target address to 80:00:55.

<img src="doc/img/cdgui1.png">
<br><br>

After modifying the configuration, write 1 to save_conf to save the configuration to flash.

If you need to restore the default configuration, change magic_code to another value and save it to flash. Then reapply power.

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

Note: The default is 3 seconds for bootloader mode on power-on. The status light will flash when it jumps to the main program. Please use the relevant script tool (except IAP) after this.

