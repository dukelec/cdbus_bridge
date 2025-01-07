## CDBUS Bridge

The `CDBUS Bridge HS` is a USB-to-RS485 (CDBUS) adapter. The USB port supports `High-Speed`, and the RS485 interface supports up to 50 Mbps.

<img alt="cdbus_bridge" src="doc/img/cdbridge_v6.1.jpg">  
<img alt="cdbus_bridge" src="doc/img/cdbridge_v6.1_case.jpg">

 - The RS485 baud rate must be configured using the CDBUS GUI tool, as the hardware supports dual baud rate mode. The baud rate specified when opening the USB serial port is ignored.
 - The two RS485 ports are internally straight-through, simplifying wiring.

Switchs Defination：
 - S1.1: Mode selection:  
         ON: Bootloader mode; OFF: Application mode.
 - S1.2: RS485 baud rate selection:  
         ON: 115200 bps; OFF: User-configured baud rate.
 - S2.1: Enable pull-up resistor.
 - S2.2: Enable termination resistor.
 - S2.3: Enable pull-down resistor.
 - S2.4: Enable 5V output.


## GUI Configuration

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

To enter configuration mode, open the serial port and set the baud rate to `52685` (`0xcdcd`).

When you open the serial port, specify the baud rate as `52685` (`0xcdcd`) to enter the configuration mode.

The target address should be set to `00:00:fe`.

After modifying the configuration, write 1 to `save_conf` to save the changes to flash.

To restore the default configuration, change the value of `magic_code` to a different value, save it to flash, and then power cycle the device.

<img src="doc/img/cdgui.png">


## Download Source Code

```
git clone https://github.com/dukelec/cdbus_bridge
```

For other hardware versions, please switch to the corresponding branch.

## Test

```
git clone --recurse-submodules https://github.com/dukelec/cdbus_tools
```

```
cd cdbus_tools/
./cdbus_terminal.py --help
```

