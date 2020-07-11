import busio
import board
from adafruit_bus_device.i2c_device import I2CDevice
import time
import SX1509
import IO_Types

#IO expander pin definitions
PIN_LED_R_LEFT = 14
PIN_LED_G_LEFT = 13
PIN_LED_B_LEFT = 12
PIN_LED_R_RIGHT = 6
PIN_LED_G_RIGHT = 5
PIN_LED_B_RIGHT = 4

PIN_VUSB = 1
PIN_LBO = 2
PIN_AMP_SD = 3

PIN_RESET_BUTTON = 10

DEVICE_ADDRESS = 0x3E  # device address of SX1509

#Set up I2C
comm_port = busio.I2C(board.SCL, board.SDA)
device = I2CDevice(comm_port, DEVICE_ADDRESS)

#Initialize the expander
IOExpander = SX1509.SX1509(comm_port)
IOExpander.clock(oscDivider = 4)
IOExpander.debounceTime(32)

#Set up pins
IOExpander.pinMode(PIN_LED_R_LEFT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
IOExpander.pinMode(PIN_LED_G_LEFT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
IOExpander.pinMode(PIN_LED_B_LEFT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
IOExpander.pinMode(PIN_LED_R_RIGHT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
IOExpander.pinMode(PIN_LED_G_RIGHT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
IOExpander.pinMode(PIN_LED_B_RIGHT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)

IOExpander.pinMode(PIN_VUSB, IO_Types.PIN_TYPE_INPUT, True)
IOExpander.pinMode(PIN_LBO, IO_Types.PIN_TYPE_INPUT, True)
IOExpander.pinMode(PIN_AMP_SD, IO_Types.PIN_TYPE_OUTPUT)
IOExpander.pinMode(PIN_RESET_BUTTON, IO_Types.PIN_TYPE_INPUT_PULLDOWN)

IOExpander.digitalWrite(PIN_AMP_SD, 0)	#Amp muted by default
IOExpander.debouncePin(PIN_RESET_BUTTON)

IOExpander.enableInterrupt(PIN_RESET_BUTTON, IO_Types.INTERRUPT_STATE_RISING)
IOExpander.enableInterrupt(PIN_VUSB, IO_Types.INTERRUPT_STATE_FALLING)
IOExpander.enableInterrupt(PIN_LBO, IO_Types.INTERRUPT_STATE_FALLING)

print('IO Expander Initialized')

IOExpander.breathe(PIN_LED_R_RIGHT, 1000, 1000, 200, 200, 100, 0)
IOExpander.blink(PIN_LED_G_LEFT, 1000, 1000, 50, 0)

while 1:
 InterruptVals = IOExpander.interruptSource()
 if(InterruptVals & (1<<PIN_RESET_BUTTON)):
  print('Reset button pressed')
 elif(InterruptVals & (1<<PIN_VUSB)):
  print('USB Power Lost')
 elif(InterruptVals & (1<<PIN_LBO)):
  print('Low Battery')
 time.sleep(.2)


#PinVal = IOExpander.digitalRead(PIN_RESET_BUTTON)
#PinVal2 = PinVal

#while 1:
# PinVal = IOExpander.digitalRead(PIN_RESET_BUTTON)
# if (PinVal != PinVal2):
#  print('Val: ' + str(PinVal) + ', Val2: ' + str(PinVal2))
 # PinVal2 = PinVal

#print('Read: ' + str(IOExpander.digitalRead(PIN_RESET_BUTTON)))



#IOExpander.pinMode(PIN_LED_R_RIGHT, IO_Types.PIN_TYPE_ANALOG_OUTPUT)
#IOExpander.analogWrite(PIN_LED_R_RIGHT, 0)
#IOExpander.breathe(PIN_LED_R_RIGHT, 1000, 1000, 200, 200, 100, 0)
#IOExpander.blink(PIN_LED_G_RIGHT, 1000, 1000, 150, 0)

#blarg = IOExpander._read_reg_16(A_DEVICE_REGISTER)
#blarg = ReadReg(A_DEVICE_REGISTER)
#print("{0:b}".format(blarg))

#IOExpander._write_reg_16(A_DEVICE_REGISTER, 0xBFEF)
#IOExpander.pinMode(PinToSet, 0x01)
#IOExpander.digitalWrite(PinToSet, 0)

#IOExpander.pinMode(PIN_LED_R_RIGHT, 0x03)
#IOExpander.pinMode(PIN_LED_G_RIGHT, 0x03)
#IOExpander.pinMode(PIN_LED_B_RIGHT, 0x03)
#IOExpander.analogWrite(PIN_LED_R_RIGHT, 0)
#IOExpander.analogWrite(PIN_LED_G_RIGHT, 0)
#IOExpander.analogWrite(PIN_LED_B_RIGHT, 200)
#time.sleep(1)


#time.sleep(3)
#print('Amp Un-Muted')
#IOExpander.digitalWrite(PIN_AMP_SD, 1)
#time.sleep(3)
#print('Amp Muted')
#IOExpander.digitalWrite(PIN_AMP_SD, 0)

#IOExpander.analogWrite(PIN_LED_R_RIGHT, 0)
#IOExpander.analogWrite(PIN_LED_G_RIGHT, 0)
#IOExpander.analogWrite(PIN_LED_B_RIGHT, 100)
#time.sleep(1)

#IOExpander.analogWrite(PIN_LED_R_RIGHT, 10)
#IOExpander.analogWrite(PIN_LED_G_RIGHT, 10)
#IOExpander.analogWrite(PIN_LED_B_RIGHT, 10)
#time.sleep(1)


#WriteReg(A_DEVICE_REGISTER, 0xBF)
#blarg = IOExpander._read_reg_16(A_DEVICE_REGISTER)
#blarg = ReadReg(A_DEVICE_REGISTER)
#print("{0:b}".format(blarg))



#IOExpander.digitalWrite(PinToSet, 1)
#IOExpander.reset(0)
#IOExpander._write_reg_8(A_DEVICE_REGISTER, 0xFF)
#WriteReg(A_DEVICE_REGISTER, 0xFF)
#blarg = IOExpander._read_reg_16(A_DEVICE_REGISTER)
#blarg = ReadReg(A_DEVICE_REGISTER)
#print("{0:b}".format(blarg))