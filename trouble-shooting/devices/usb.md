# USB関連

## USBが認識されない



```shell
$ sudo dmesg -dw
[ 3481.122280 <  347.123169>] usb 1-5: new full-speed USB device number 35 using xhci_hcd
[ 3481.250293 <    0.128013>] usb 1-5: device descriptor read/64, error -71
[ 3481.486289 <    0.235996>] usb 1-5: device descriptor read/64, error -71
[ 3481.722274 <    0.235985>] usb 1-5: new full-speed USB device number 36 using xhci_hcd
[ 3481.850283 <    0.128009>] usb 1-5: device descriptor read/64, error -71
[ 3482.086292 <    0.236009>] usb 1-5: device descriptor read/64, error -71
[ 3482.194345 <    0.108053>] usb usb1-port5: attempt power cycle
[ 3482.610233 <    0.415888>] usb 1-5: new full-speed USB device number 37 using xhci_hcd
[ 3482.610415 <    0.000182>] usb 1-5: Device not responding to setup address.
[ 3482.818400 <    0.207985>] usb 1-5: Device not responding to setup address.
[ 3483.026258 <    0.207858>] usb 1-5: device not accepting address 37, error -71
[ 3483.154244 <    0.127986>] usb 1-5: new full-speed USB device number 38 using xhci_hcd
[ 3483.154417 <    0.000173>] usb 1-5: Device not responding to setup address.
[ 3483.362411 <    0.207994>] usb 1-5: Device not responding to setup address.
[ 3483.570222 <    0.207811>] usb 1-5: device not accepting address 38, error -71
[ 3483.570345 <    0.000123>] usb usb1-port5: unable to enumerate USB device
```

`device descriptor read/64, error -71`が問題そう。

- 参考
  - [Ubuntu(Linux)のUSB-Cポートがうまく動かないんだが...](device descriptor read/64, error -71)
  - [device descriptor read/64, error -71](device descriptor read/64, error -71)

上記に従い以下を実行。

```shell
sudo echo Y > /sys/module/usbcore/parameters/old_scheme_first
```

読み込めてそう

```shell
[ 3578.320022 <   94.749677>] usb 2-9: new SuperSpeed USB device number 3 using xhci_hcd
[ 3578.340860 <    0.020838>] usb 2-9: New USB device found, idVendor=0781, idProduct=558c, bcdDevice=10.12
[ 3578.340870 <    0.000010>] usb 2-9: New USB device strings: Mfr=2, Product=3, SerialNumber=1
[ 3578.340873 <    0.000003>] usb 2-9: Product: Extreme SSD
[ 3578.340876 <    0.000003>] usb 2-9: Manufacturer: SanDisk
[ 3578.340878 <    0.000002>] usb 2-9: SerialNumber: 313930393233343030303938
[ 3578.361854 <    0.020976>] usbcore: registered new interface driver usb-storage
[ 3578.366195 <    0.004341>] scsi host8: uas
[ 3578.366283 <    0.000088>] usbcore: registered new interface driver uas
[ 3578.366694 <    0.000411>] scsi 8:0:0:0: Direct-Access     SanDisk  Extreme SSD      1012 PQ: 0 ANSI: 6
[ 3578.368823 <    0.002129>] scsi 8:0:0:1: Enclosure         SanDisk  SES Device       1012 PQ: 0 ANSI: 6
[ 3578.370725 <    0.001902>] sd 8:0:0:0: Attached scsi generic sg1 type 0
[ 3578.370934 <    0.000209>] sd 8:0:0:0: [sdb] Spinning up disk...
[ 3578.370977 <    0.000043>] scsi 8:0:0:1: Attached scsi generic sg2 type 13
[ 3579.375801 <    1.004824>] ......ready
[ 3584.495918 <    5.120117>] sd 8:0:0:0: [sdb] 3907028992 512-byte logical blocks: (2.00 TB/1.82 TiB)
[ 3584.495923 <    0.000005>] sd 8:0:0:0: [sdb] 4096-byte physical blocks
[ 3584.496034 <    0.000111>] sd 8:0:0:0: [sdb] Write Protect is off
[ 3584.496038 <    0.000004>] sd 8:0:0:0: [sdb] Mode Sense: 67 00 10 08
[ 3584.496296 <    0.000258>] sd 8:0:0:0: [sdb] Write cache: disabled, read cache: enabled, supports DPO and FUA
[ 3584.496493 <    0.000197>] sd 8:0:0:0: [sdb] Preferred minimum I/O size 4096 bytes
[ 3584.496496 <    0.000003>] sd 8:0:0:0: [sdb] Optimal transfer size 33553920 bytes not a multiple of preferred minimum block size (4096 bytes)
[ 3584.545349 <    0.048853>]  sdb: sdb1
```
