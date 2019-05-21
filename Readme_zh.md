## CDBUS Bridge

<img alt="cdbus_bridge" src="doc/img/cdbus_bridge.jpg">
<img alt="cdbus_bridge_pcb" src="doc/img/cdbus_bridge_pcb.jpg">


CDBUS Bridge 有兩類通訊端口，分別爲 3 個串口和 2 個 RS485 口：
 - 3 個串口同一時間只會有一個處於使用狀態，插入 USB 會優先切換使用免配置的 USB 串口（通訊速率不受所選波特率影響）；
 - 2 個 RS485 口內部直通，方便接線（或上拉、下拉及終端電阻）。

CDBUS Bridge 有兩個模式，通過開關切換：

### Bridge 模式（串口轉 RS485）

<img alt="bridge_mode" src="doc/img/bridge_mode.svg">

### Raw 模式（串口透傳）

<img alt="raw_mode" src="doc/img/raw_mode.svg">


## 下載

```
git clone --recurse-submodules https://github.com/dukelec/cdbus_bridge.git
```

## 安裝依賴包
 - Linux: pip3 install pycrc pyserial
 - Mac: pip3 install readline pycrc pyserial
 - Windows: pip3 install pyreadline pycrc pyserial

## 測試

請參考 `sw/` 目錄下的 `Readme.md` 以及各腳本的 `--help` 幫助，例如常用的：

```
cd sw/cdbus_tools/
./cdbus_terminal.py --help
```

註：上電默認 3 秒為 bootloader 模式，跳轉主程序時狀態燈會閃爍一下，請在這之後再使用相關腳本工具（IAP 除外）。

