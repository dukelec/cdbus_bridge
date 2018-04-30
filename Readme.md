## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge.jpg">


CDBUS Bridge has two types of communication ports, which are 3 serial ports and 2 RS485 ports:
 - Only one of the three serial ports will be in use at the same time. Inserting USB will switch over to the configuration-free USB serial port (the communication rate will not be affected by the selected baud rate);
 - The two RS485 ports are internally connected for easy wiring (or pull up, pull down, and termination resistors).

The CDBUS Bridge has two modes, which are selected by the switch:

### Bridge Mode (Serial to RS485)

<img alt="bridge_mode" src="doc/img/bridge_mode.svg">

### Raw Mode (Serial passthrough)

<img alt="raw_mode" src="doc/img/raw_mode.svg">

