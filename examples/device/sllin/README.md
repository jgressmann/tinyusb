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

#### Configure slave to response to headers


Configure slave response for ID 'b' using classic checksum. Note, the `-x` suppresses echoing the CAN frame

```
cangen -I b slcan1 -L 4 -n 1 -D deadbeef -x
```

Configure slave response for ID 'b' using enhanced checksum.

```
cangen -I 4b slcan1 -L 4 -n 1 -D deadbeef -x
```


#### Send master requests

Send header for ID 'b'. Again, the `-x` suppresses the local echo of the RTR frame.

```
cangen -I b slcan0 -R  -L 4 -n 1 -x
```

If a slave responds to the header, a CAN frame will be echoed. If the request times out a RTR frame will be echoed.

## Serial Line CAN (slcan) Protocol

The protocol for CAN is decribed [here](http://www.can232.com/docs/canusb_manual.pdf).

## Serial Line LIN (slLIN) Protocol

slLIN mostly follows Lawice's protocol with a few modifications for LIN:

| Command&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                                | Direction&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; |  Description                                   |
|----------------------------------------|----------------|------------------------------------------------|
| `L\r`                                  | host to device | Open device in slave mode |
| `O\r`                                  | host to device | Open device in master mode |
| `C\r`                                  | host to device | Close device |
| `tIIILDD...\r`                         | host to device | 1. If the device operates in master mode, this sends a master data frame. </br>2. If the device operatates in slave mode and length is non-zero, this stores a data reponse.</br>3.  If the device operatates in slave mode and and length is zero, this clears a data reponse. |
| `tIIILDD...\r`                         | device to host | A complete LIN frame (header + data). |
| `rIIIL\r`                              | host to device | Only valid in master mode. Send LIN header.    |
| `rIIIL\r`                              | device to host | Header on bus but no slave responded. |
| `n\r`                                  | both           | Request device name                            |
| `Yn\r`</br>(where n is 0 or 1)         | host to device | Enable / disable periodic time stamp transmission.</br>If enabled, the device sends the current time stamp approx. once per second as `YTTTT\r`.</br>The format of the time stamp follows the Lawicel format, c.f. documentation of `Zn\r` command. |
| `shhhh\r`</br>(hex values)             | host to device | Set device baud rate, e.g  `4b00` sets 19200 Baud/s |
| `Sn\r`                                 | host to device | Set sleep timeout in seconds, i.e. `S0` sets 4 seconds, `S1` 5 seconds... |



slLIN used the unused bits in the CAN ID to transmit information on the frame.

| Bitmask | Description |
|---------|-------
| `0x040`  | Flag. Frame uses enhanced checksum |
| `0x080`  | Flag. Foreign frame, length & checksum are unverified |
| `0x100`  | Flag. Bus sleep |
| `0x200`  | Flag. Bus wake up |
| `0x400`  | Flag. Master data frame |

