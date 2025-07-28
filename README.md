
# BT_Sousce for Yoradio_mod

Based in ESP32 WROOM

## Connections
### I2S
~~~
WS      -   25
BLK     -   26
DIN     -   27
MUTE    -   23
~~~

__MUTE__ set LOW level when BT Sink connected

### UART
~~~
RX      -   5
TX      -   4
~~~

### ADC
~~~
INPUT   -   32

CHARGE_MODE_SETUP   -   12
CHARGE_MODE_CONTROL -   13
CHARGE_MODE_LED     -   14 
~~~

__INPUT__ connected to voltage divider (R1 = 47k, R2 = 120k)

__CHARGE_MODE_SETUP__ - charge detection (input):
LOW - hardware detection
HIGH - software detection

__CHARGE_MODE_CONTROL__ -   input for hardware detection

__CHARGE_MODE_LED__ - charge indication led (output).

## Flash

1. From VS-CODE + ESP-IDF
2. Flash Download Tools:
    ~~~
    0x01000     build/bootloader/bootloader.bin
    0x10000     build/bt_source.bin
    0x08000     build/partition_table/partition-table.bin
    ~~~