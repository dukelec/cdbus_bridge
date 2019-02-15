### Read config from device
```
cdbus_tools/cdbus_iap.py --direct --local-mac=0xaa --target-addr="80:00:55" --addr=0x0801f800 --size=36 --out-file conf.bin
```

### Convert to json
```
./config_conv.py --to-json --bin conf.bin --json conf.json
```

#### Or creat json by default config
```
./config_conv.py --to-json --json conf.json
```


### Convert json back to bin
```
./config_conv.py --to-bin --json conf.json --bin conf.bin
```

### Write back to device
```
cdbus_tools/cdbus_iap.py --direct --local-mac=0xaa --target-addr="80:00:55" --addr=0x0801f800 --in-file conf.bin
```
