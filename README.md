This is a simple C program which blinks an LED on the Arduino Uno, transmitting
a message via the serial link whenever it does so.

## Building and running screen -l /dev/ttyACM0 -s 9600      killall screen

### Dependencies

* make
* binutils-avr
* avr-libc
* avrdude
* gcc-avr

### Compiling

    make

The default `make` task builds an Intel HEX file, `led_blink.hex`, which can be
uploaded to the Arduino Uno.

### Uploading

    sudo make upload

Flashes `led_blink.hex` to the Arduino Uno, which is assumed to be connected
and available on `/dev/ttyACM0`.

### Watching

A status LED should be visibly blinking on the Arduino. However, this program
also sends a message via the USB serial connection every time the LED blinks.

One way to interact via the serial connection is with
[CuteCom](http://cutecom.sourceforge.net/).

1. Install CuteCom (`sudo apt-get install cutecom`)
2. Run CuteCom as root (`sudo cutecom`)
3. Set "Device" to `/dev/ttyACM0` "Baud rate" to `9600`
4. Click "Open Device"

You should be able to see output from the program running on the Arduino
being streamed to the display area (the string "Blink!" is printed each
time the LED blinks).

Note: Make sure you close the device in CuteCom before running
`sudo make upload`.

## References

* http://www.appelsiini.net/2011/simple-usart-with-avr-libc
* https://balau82.wordpress.com/2011/03/29/programming-arduino-uno-in-pure-c/
