# slLIN


[![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

## What is this?

This is project slLIN. A LIN protocol inspired by Lawicel' serial line CAN protocol.

## How do I use it?

### Linux

Configure for master mode, 19200 baud

```
sudo slcand -o -F -b 4b00 /dev/ttyACM1
```

Configure for slave mode, 19200 baud

```
sudo slcand -l -F -b 4b00 /dev/ttyACM1
```

#### Configure Response

Configure response for ID 'b' using classic checksum. Note, the `-x` suppresses echoing the CAN frame

```
cangen slcan1 -x -e -L 4 -n 1 -D deadbeef -I 31000b
```

Configure response for ID '23' using enhanced checksum.

```
cangen slcan1 -x -e -L 8 -n 1 -D f00f00AA55AA55AA -I 320017
```


#### Send Master Requests

Send header for ID 'b'. Again, the `-x` suppresses the local echo of the RTR frame.

```
cangen slcan0  -x -R -I b -n 1
```

#### Send Master Frames

To send a master frame, first configure the frame data as you would for a slave, then send the request.

## Serial Line CAN (slcan) Protocol

The protocol for CAN is decribed [here](http://www.can232.com/docs/canusb_manual.pdf).

## Serial Line LIN (slLIN) Protocol

slLIN mostly follows Lawice's protocol with a few modifications for LIN:

| Command&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                                | Direction&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; |  Description                                   |
|----------------------------------------|----------------|------------------------------------------------|
| `L\r`                                  | host to device | Open device in slave mode |
| `O\r`                                  | host to device | Open device in master mode |
| `C\r`                                  | host to device | Close device |
| `TIIIIIIIILDD...\r`                       | both | See next section for CAN-ID decoding |
| `rIIIL\r`                              | host to device | Only valid in master mode. Send LIN header.    |
| `n\r`                                  | both           | Request device name                            |
| `Yn\r`</br>(where n is 0 or 1)         | host to device | Enable / disable periodic time stamp transmission.</br>If enabled, the device sends the current time stamp approx. once per second as `YTTTT\r`.</br>The format of the time stamp follows the Lawicel format, c.f. documentation of `Zn\r` command. |
| `shhhh\r`</br>(hex values)             | host to device | Set device baud rate, e.g  `4b00` sets 19200 Baud/s |
| `Sn\r`                                 | host to device | Set sleep timeout in seconds, i.e. `S0` sets 4 seconds, `S1` 5 seconds... |


## Frame Coding

### LIN frames

Length and data of the carrying CAN frame must be valid. A length of `0` implies an unanswered request.


| Start Bit | Length | Description |
|:----------|:-------|:------------|
| `0`       | `6`    | LIN-ID |
| `8`       | `8`    | CRC |

#### Device to Host Frames


| Bitmask      | Description |
|:-------------|:------------|
| `0x00010000` | bad sync field (not 0x55). This is typically the only flag |
| `0x00020000` | bad PID received, LIN-ID carries recovered ID |
| `0x00040000` | bit error in some fixed part of the frame e.g. start bit wasn't 0, stop bit wasn't 1, ... |
| `0x00080000` | more than 9 bytes on bus |
| `0x00100000` | request was not answered by this node |


##### Bus Error and State Frame

These are always from from device to host and do not carry any LIN frame data.

| Bitmask      | Description |
|:-------------|:------------|
| `0x10000000` | bus status bits are valid |
| `0x08000000` | bus error bits are valid |


| Start Bit | Length | Description |
|:----------|:-------|:------------|
| `0`       | `2`    | Bus State |
| `2`       | `2`    | Bus Error |

###### Bus State

| Value  | Description |
|:-------|:------------|
| `0x00` | bus is asleep |
| `0x01` | bus is awake |
| `0x02` | bus is in error state |

###### Bus Error

| Value  | Description |
|:-------|:------------|
| `0x00` | no error |
| `0x01` | bus shorted to GND |
| `0x02` | bus shorted to VBAT |


#### Host to Device Frames

| Bitmask      | Description |
|:-------------|:------------|
| `0x00100000` | Enables frame response if set, else disables. |
| `0x00200000` | store LIN frame data. The CAN frames length and data must be valid |


| Start Bit | Length | Description |
|-----------|--------|-------------|
| `16`      | `2`    | CRC computation |

##### CRC Computation

| Value  | Description |
|:-------|:------------|
| `0x00` | take CRC field as is |
| `0x01` | ignore CRC field, compute classic CRC |
| `0x02` | ignore CRC field, compute enhanced CRC |
