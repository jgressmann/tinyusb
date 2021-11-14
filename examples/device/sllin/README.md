Configure for master mode, 19200 baud

```
sudo slcand -o -F -b 4b00 /dev/ttyACM1
```

Configure for slave mode, 19200 baud

```
sudo slcand -l -F -b 4b00 /dev/ttyACM1
```
