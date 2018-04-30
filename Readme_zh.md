## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge.jpg">


CDBUS Bridge 有兩類通訊端口，分別爲 3 個串口和 2 個 RS485 口：
 - 3 個串口同一時間只會有一個處於使用狀態，插入 USB 會優先切換使用免配置的 USB 串口（通訊速率不受所選波特率影響）；
 - 2 個 RS485 口內部直通，方便接線（或上拉、下拉及終端電阻）。

CDBUS Bridge 有兩個模式，通過開關切換：

### Bridge 模式（串口轉 RS485）

<img alt="bridge_mode" src="doc/img/bridge_mode.svg">

### Raw 模式（串口透傳）

<img alt="raw_mode" src="doc/img/raw_mode.svg">

