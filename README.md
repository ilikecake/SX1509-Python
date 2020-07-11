# SX1509 Python

This a port of the [Sparkfun SX1509 Library](https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library/). The library was written in C for Arduino. I converted it to Python 3, and fixed up a few things that seemed broken.

![Picture of the Sparkfun SX1509 Expander](https://github.com/ilikecake/SX1509-Python/blob/master/images/13601-01.jpg?raw=true)

The SX1509 is a 16 port IO expander. The expander talks to the host CPU over I2C and has a few nice extra functions such as:
* Programmable pull up or pull down resistors
* LED dimming, blinking, and breathing
* Input debouncing
* Keypad scanning (not tested in my code)

More info on the hardware can be found on the [Sparkfun product page](https://www.sparkfun.com/products/13601) and their [Tutorial](https://learn.sparkfun.com/tutorials/sx1509-io-expander-breakout-hookup-guide#installing-the-sparkfun-sx1509-arduino-library).

## Prerequisites

This code was built and tested on a Raspberry Pi (B 3+). It should run okay on anything that can run Python code with a few modification. More on this in a bit.

The code uses the Adafruit Blinka libraries for I2C communication. To install these libraries on your Raspberry Pi, follow [Adafuit's excellent tutorial](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi). I was not super happy with doing it this way, but it is the was the easiest way I found that did not require me to write my own I2C code. This is not a dig at the Adafruit libraries, they seem quite good and well maintained. However, relying on them will make porting this code to a different CPUs more difficult in the future. I don't really plan on doing this, however, so I am not too bothered by it.

## Example

I include a sample python script (Example.py) to show the basic setup and usage of the library. This example sets up a SX1509 expander on the default address with a few connections:
* Leds on pins 4,5,6,12,13,14
* Pin 1 is a digital input attached to the VUSB line. This pin is used to sense when USB power is disconnected. (My Project has a backup battery)
* Pin 2 is a digital input attached to the LBO output of the battery charger ([Adafruit Powerbost 1000](https://www.adafruit.com/product/2465)). This pin is normally high, and goes low then the battery is about to die. I plan to use this to turn off the Pi gracefully if needed.
* Pin 3 is a digital out attached to the shutdown pin of a small PAM8403 speaker amp. Setting this pin low mutes the amp when not in use.
* Pin 10 is a digital input attached to a tactile switch. This pin is debounced. I intend to use this pin to gracefully shut down the Pi when pressed.

A note about the LEDs: I have attached them to the expander on the low side. This means that the LED cathode (- lead) is attached to the expander pin. The anode (+ lead) is attached to the 5V power supply. Make sure to include current limiting resistors on your LEDs, this is not a constant current LED driver.

## Porting

As I said above, this code should work with any CPU that can run python 3 with a few modification. In order to port this code to a different controller, you will need to update the functions at the bottom of the library that are hardware specific:
* _HW_Reset(self)
* _write_reg_8(self, reg, val)
* _write_reg_16(self, reg, val)
* _read_reg_8(self, reg)
* _read_reg_16(self, reg)

You will probably also need to do something about the Adafruit i2c_device library. This might be as simple as passing a null to the function to initialize the i2c_device. I have not put too much though into this yet...