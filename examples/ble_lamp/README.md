# BLE Lamp

This ESP-IDF project advertises a controllable BLE lamp named `NFC-BLE-LAMP`.
The NFC remote connects only after reading a supported Bluetooth LE OOB NDEF
record from a tag. Pressing the remote button toggles the lamp through the
custom GATT service (`0xFFF0`, command characteristic `0xFFF1`). Command values
`0`, `1`, and `2` mean off, on, and toggle.

Defaults target ESP32-S3. The remote uses GPIO9 as an active-low button input;
connect a momentary button between GPIO9 and GND. The lamp drives the onboard
WS2812 RGB LED on GPIO48 using RMT. Change the corresponding fallback GPIO
value in `remote_ble.c` or `ble_lamp.c` if the hardware differs.

Normal mode is read-only: blank or unsupported tags are ignored and never
written. To provision a tag, hold the remote's GPIO9 button for five seconds.
The status LED turns blue, and the remote scans for the lamp while waiting for
a blank NTAG. It writes a Bluetooth LE OOB NDEF record containing the lamp's
BLE address, name, service UUID, role, and pairing TK, then reads the record
back to verify it. After successful verification, the remote restarts into
normal read mode. Only NTAG213/215/216 tags are supported, and existing tag
contents are preserved. On later taps, the remote connects using the OOB
record; the tag UID is not used as a pairing or connection switch. The remote
status LED is green when authenticated BLE control is ready and red when tag
reading or pairing fails before control is ready. A short GPIO9 press toggles
the lamp while connected.

The two development firmware images share a fixed OOB TK, so a copied tag can
also pair with this development lamp. Use a per-lamp secret and protected tag
provisioning before deploying this design.

Build and flash the lamp from this directory:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM1 flash monitor
```

Build and flash the NFC remote from `examples/ntag_read`:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```
