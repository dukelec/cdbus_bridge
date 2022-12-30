TODO: please use CDBUS-GUI instead!


### Read config from device
```
cdbus_tools/cdbus_iap.py --direct --addr=0x0801f800 --size=36 --out-file conf.bin
```

### Convert to cfg
```
./config_conv.py --to-cfg --bin conf.bin --cfg conf.cfg
```

#### Or creat cfg by default config
```
./config_conv.py --to-cfg --cfg conf.cfg
```


### Convert cfg back to bin
```
./config_conv.py --to-bin --cfg conf.cfg --bin conf.bin
```

### Write back to device
```
cdbus_tools/cdbus_iap.py --direct --addr=0x0801f800 --in-file conf.bin
```
