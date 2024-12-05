# pico Stepper-Control



## Run client

### Ubuntu

- Install **PySimpleGUI**

```
pip3 install PySimpleGUI
pip3 install pyserial
```

- Use ```sudo dmesg | tail``` to identify the name of the serial port (Typically **/dev/ttyACM0**).

- set permission for serial port
```
sudo chmod 666 /dev/ttyACM0
```

- run client program

```
python3 client.py /dev/ttyACM0
```

