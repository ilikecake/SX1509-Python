# Copyright (c) 2020 Pat Satyshur
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Based on: (Thanks guys, I owe you a beer :))
#  SparkFun SX1509 I/O Expander Library
#  Jim Lindblom @ SparkFun Electronics
#  Original Creation Date: September 21, 2015
#  https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

#TODO: Move the registers to a new file?

from adafruit_bus_device import i2c_device
import IO_Types

_SX1509_RegInputDisableB = 0x00 #Input buffer disable register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegInputDisableA = 0x01 # Input buffer disable register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegLongSlewB = 0x02 # Output buffer long slew register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegLongSlewA = 0x03 # Output buffer long slew register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegLowDriveB = 0x04 # Output buffer low drive register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegLowDriveA = 0x05 # Output buffer low drive register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegPullUpB = 0x06 # Pull-up register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegPullUpA = 0x07 # Pull-up register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegPullDownB = 0x08 # Pull-down register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegPullDownA = 0x09 # Pull-down register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegOpenDrainB = 0x0A # Open drain register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegOpenDrainA = 0x0B # Open drain register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegPolarityB = 0x0C # Polarity register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegPolarityA = 0x0D # Polarity register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegDirB = 0x0E # Direction register - I/O[15-8] (Bank B) 1111 1111
_SX1509_RegDirA = 0x0F # Direction register - I/O[7-0] (Bank A) 1111 1111
_SX1509_RegDataB = 0x10 # Data register - I/O[15-8] (Bank B) 1111 1111*
_SX1509_RegDataA = 0x11 # Data register - I/O[7-0] (Bank A) 1111 1111*
_SX1509_RegInterruptMaskB = 0x12 # Interrupt mask register - I/O[15-8] (Bank B) 1111 1111
_SX1509_RegInterruptMaskA = 0x13 # Interrupt mask register - I/O[7-0] (Bank A) 1111 1111
_SX1509_RegSenseHighB = 0x14 # Sense register for I/O[15:12] 0000 0000
_SX1509_RegSenseLowB = 0x15 # Sense register for I/O[11:8] 0000 0000
_SX1509_RegSenseHighA = 0x16 # Sense register for I/O[7:4] 0000 0000
_SX1509_RegSenseLowA = 0x17 # Sense register for I/O[3:0] 0000 0000
_SX1509_RegInterruptSourceB = 0x18 # Interrupt source register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegInterruptSourceA = 0x19 # Interrupt source register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegEventStatusB = 0x1A # Event status register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegEventStatusA = 0x1B # Event status register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegLevelShifter1 = 0x1C # Level shifter register 0000 0000
_SX1509_RegLevelShifter2 = 0x1D # Level shifter register 0000 0000
_SX1509_RegClock = 0x1E # Clock management register 0000 0000
_SX1509_RegMisc = 0x1F # Miscellaneous device settings register 0000 0000
_SX1509_RegLEDDriverEnableB = 0x20 # LED driver enable register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegLEDDriverEnableA = 0x21 # LED driver enable register - I/O[7-0] (Bank A) 0000 0000
#Debounce and Keypad Engine
_SX1509_RegDebounceConfig = 0x22 # Debounce configuration register 0000 0000
_SX1509_RegDebounceEnableB = 0x23 # Debounce enable register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegDebounceEnableA = 0x24 # Debounce enable register - I/O[7-0] (Bank A) 0000 0000
_SX1509_RegKeyConfig1 = 0x25 # Key scan configuration register 0000 0000
_SX1509_RegKeyConfig2 = 0x26 # Key scan configuration register 0000 0000
_SX1509_RegKeyData1 = 0x27 # Key value (column) 1111 1111
_SX1509_RegKeyData2 = 0x28 # Key value (row) 1111 1111
#LED Driver (PWM, blinking, breathing)
#Register arrays for LED setup. Only pins 4-7 and 12-15 are capable of breathing. TRise and TFall for other pins are set to 0xFF.
# RegTOn: On time register. 
#   - 0:	 Static, state controlled by RegData 
#   - 1-15:  On time = 64 * RegTOnX * (255/ClkX)
#   - 16-31: On time = 512 * RegTOnX * (255/ClkX)
# RegIOn: On intensity register
# RegOff: Off time/intensity register. Bits 7:3 are off time, bits 2:0 are off intensity.
#   - Off time defined the same as on time above
#   - Off intensity = 4 x RegOff[2:0]
# RegTRise: Fade in time register
#   - 0: OFF
#   - 1-15: TRiseX = (RegIOnX-(4xRegOffX[2:0])) * RegTRiseX * (255/ClkX)
#   - 16-31 : TRiseX = 16 * (RegIOnX-(4xRegOffX[2:0])) * RegTRiseX * (255/ClkX)
# RegTFall: Fade out time register
#   - 0: OFF
#   - 1-15: TFallX = (RegIOnX-(4xRegOffX[2:0])) * RegTFallX * (255/ClkX)
#   - 16-31: TFallX = 16 * (RegIOnX-(4xRegOffX[2:0])) * RegTFallX * (255/ClkX)
#					0	  1		2	  3		4	  5		6	  7		8	  9		10	  11	12	  13	14	  15
_SX1509_RegTOn   = [0x29, 0x2C, 0x2F, 0x32, 0x35, 0x3A, 0x3F, 0x44, 0x49, 0x4C, 0x4F, 0x52, 0x55, 0x5A, 0x5F, 0x64]
_SX1509_RegIOn   = [0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45, 0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x60, 0x65]
_SX1509_RegOff   = [0x2B, 0x2E, 0x31, 0x34, 0x37, 0x3C, 0x41, 0x46, 0x4B, 0x4E, 0x51, 0x54, 0x57, 0x5C, 0x61, 0x66]
_SX1509_RegTRise = [0xFF, 0xFF, 0xFF, 0xFF, 0x38, 0x3D, 0x42, 0x47, 0xFF, 0xFF, 0xFF, 0xFF, 0x58, 0x5D, 0x62, 0x67]
_SX1509_RegTFall = [0xFF, 0xFF, 0xFF, 0xFF, 0x39, 0x3E, 0x43, 0x48, 0xFF, 0xFF, 0xFF, 0xFF, 0x59, 0x5E, 0x63, 0x68]


#Miscellaneous
_SX1509_RegHighInputB = 0x69 # High input enable register - I/O[15-8] (Bank B) 0000 0000
_SX1509_RegHighInputA = 0x6A # High input enable register - I/O[7-0] (Bank A) 0000 0000
#Software Reset
_SX1509_RegReset = 0x7D # Software reset register 0000 0000

class SX1509:

 def __init__(self, i2c, address=0x3E, newSetup=True, CPUSpeed=0):
  self.i2c_device = i2c_device.I2CDevice(i2c, address)
  
  if newSetup:
   #Reset the device and start fresh
   self.reset(0)
   self._CPU_CLK = 0.0	#Frequency of the SX1509 core clock. Will usually be 2Mhz unless an external clock source is used. If 0: LED, keypad, and debouncing are disabled.
   self._clkX = 0.0  	#Frequency of the LED Driver clock for all IOs. If 0: LEDs are disabled.
  else:
   #Keep the device doing what it was doing, do not reset.
   #Figure out what the clocks are doing.
   tempRegData = self._read_reg_8(_SX1509_RegClock)
   if ((tempRegData & 0x60) == 0x40):
    #Expander is set up with the internal 2MHz osc running
    self._CPU_CLK = 2000000.0
   elif(((tempRegData & 0x60) == 0x20) and CPUSpeed != 0):
    #Expander is setup with an external osc. Use the CPUSpeed input to set the internal CPU speed variable.
    #If CPUSpeed is not set, do not set up clock variables
    self._CPU_CLK = CPUSpeed
   else:
    self._CPU_CLK = 0.0
   
   #CPU Clock figured out. What is the LED clock up to?
   if(self._CPU_CLK != 0):
    tempRegData = self._read_reg_8(_SX1509_RegMisc)
    if((tempRegData & 0x70) == 0):
     #LED clock is off
     self._clkX = 0.0
    else:
     #LED Clock is on, figure out the divider and get the speed
     tempRegData = (tempRegData & 0x70) >> 4
     self._clkX = self._CPU_CLK / (1<<(tempRegData-1))
   else:
    #If the CPU clock is off, the LED clock is also off
    self._clkX = 0.0



 def reset(self, hardware):
  if hardware == 1:
   #Check if bit 2 of REG_MISC is set
   #if so nReset will not issue a POR, we'll need to clear that bit first
   regMisc = readByte(REG_MISC)
   if (regMisc & (1<<2)) == (1<<2):
    regMisc &= ~(1<<2)
    self._write_reg_8(_SX1509_RegMisc, regMisc)
   #Reset the SX1509, the pin is active low
   self._HW_Reset()
   print('This function is not implemented')
  else:
   #Software reset command sequence:
   self._write_reg_8(_SX1509_RegReset, 0x12)
   self._write_reg_8(_SX1509_RegReset, 0x34)

 #Changes
 # - Added HV_mode. Setting this on an input turns on HV mode for that pin. Setting it for an output has no effect
 # - Turns off LED driver on pins that are not set up as LEDs. This will probably not come up in production, but it caused problems in testing.
 def pinMode(self, pin, inOut, HV_Mode=False):
  #The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
  # 0: IO is configured as an output
  # 1: IO is configured as an input
  if ((inOut == IO_Types.PIN_TYPE_OUTPUT) or (inOut == IO_Types.PIN_TYPE_ANALOG_OUTPUT)):
   modeBit = 0
  else:
   modeBit = 1

  tempRegDir = self._read_reg_16(_SX1509_RegDirB)
  if modeBit == 1:	
   tempRegDir |= (1<<pin)
  else:
   tempRegDir &= ~(1<<pin)

  self._write_reg_16(_SX1509_RegDirB, tempRegDir)

  #Set pull up, pull down, or open drain as required. Also set HV mode.
  self.setInputMode(pin, inOut)
  self.setInputHV(pin, HV_Mode)

  if (inOut == IO_Types.PIN_TYPE_ANALOG_OUTPUT):
   self.ledDriverInit(pin)
  else:
   self.ledDriverDeinit(pin)

 def setInputHV(self, pin, SetHV = True):
  tempRegData = self._read_reg_16(_SX1509_RegHighInputB)
  if SetHV:
   tempRegData |= (1<<pin)
  else:
   tempRegData &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegHighInputB, tempRegData)

 def setInputMode(self, pin, InputMode):
  #Input modes are (should be) mutually exclusive. Turn them all off first, then set the one we want.
  tempRegData = self._read_reg_16(_SX1509_RegPullUpB)
  tempRegData &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegPullUpB, tempRegData)
  
  tempRegData = self._read_reg_16(_SX1509_RegPullDownB)
  tempRegData &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegPullDownB, tempRegData)  
  
  tempRegData = self._read_reg_16(_SX1509_RegOpenDrainB)
  tempRegData &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegOpenDrainB, tempRegData)
  
  if InputMode == IO_Types.PIN_TYPE_INPUT_PULLUP:
   tempRegData = self._read_reg_16(_SX1509_RegPullUpB)
   tempRegData |= (1<<pin)
   self._write_reg_16(_SX1509_RegPullUpB, tempRegData)
  elif InputMode == IO_Types.PIN_TYPE_INPUT_PULLDOWN:
   tempRegData = self._read_reg_16(_SX1509_RegPullDownB)
   tempRegData |= (1<<pin)
   self._write_reg_16(_SX1509_RegPullDownB, tempRegData)
  elif (InputMode == IO_Types.PIN_TYPE_INPUT_OPEN_DRAIN) or (InputMode == IO_Types.PIN_TYPE_ANALOG_OUTPUT):
   #LEDs should be open drain
   tempRegData = self._read_reg_16(_SX1509_RegOpenDrainB)
   tempRegData |= (1<<pin)
   self._write_reg_16(_SX1509_RegOpenDrainB, tempRegData)

 def digitalWrite(self, pin, highLow):
  tempRegDir = self._read_reg_16(_SX1509_RegDirB)

  if ((0xFFFF^tempRegDir)&(1<<pin)):	# If the pin is an output, write high/low
   tempRegData = self._read_reg_16(_SX1509_RegDataB)
   if (highLow == 1):
    tempRegData |= (1<<pin)
   else:
    tempRegData &= ~(1<<pin)
   self._write_reg_16(_SX1509_RegDataB, tempRegData)

  else:	# Otherwise the pin is an input, pull-up/down
   tempPullUp = self._read_reg_16(_SX1509_RegPullUpB)
   tempPullDown = self._read_reg_16(_SX1509_RegPullDownB)

   if (highLow == 1):	# if HIGH, do pull-up, disable pull-down
    tempPullUp |= (1<<pin)
    tempPullDown &= ~(1<<pin)
    self._write_reg_16(_SX1509_RegPullDownB, tempPullDown)
    self._write_reg_16(_SX1509_RegPullUpB, tempPullUp)
   else:	# If LOW do pull-down, disable pull-up
    tempPullDown |= (1<<pin)
    tempPullUp &= ~(1<<pin)
    self._write_reg_16(_SX1509_RegPullUpB, tempPullUp)
    self._write_reg_16(_SX1509_RegPullDownB, tempPullDown)

 def digitalRead(self, pin):
  tempRegDir = self._read_reg_16(_SX1509_RegDirB)

  if (tempRegDir & (1<<pin)): # If the pin is an input
   tempRegData = self._read_reg_16(_SX1509_RegDataB)
   if (tempRegData & (1<<pin)):
    return 1
  #Return 0 if pin is low or pin is output
  return 0

 #Changes:
 # - Correctly handles setting clock
 # - Correctly sets open drain mode on init
 # - Pins set as LEDs are initialized to high (off)
 #
 #TODO: What happens if LED is connected on high side? I don't have any LEDs connected this way.
 #      I might be able to have them set up either way and use the invert register to make them act the same way later.
 #      If I decide to use high side LEDs in the future, try this out.
 def ledDriverInit(self, pin, freq = 0, log = False):
  #Note: freq = 0 will not update the LED divider
  
  #Disable input buffer
  #Writing a 1 to the pin bit will disable that pins input buffer
  tempWord = self._read_reg_16(_SX1509_RegInputDisableB)
  tempWord |= (1<<pin)
  self._write_reg_16(_SX1509_RegInputDisableB, tempWord)

  #Setup clocks
  #If clock is off, turn it on. If no divider is set, use 1 (no divider).
  #If clock is already running, check to see if divider is set and valid. If so, update divider, otherwise leave the clock alone.
  if (self._CPU_CLK == 0):
   if (freq > 0) and (freq < 8):
    self.clock(oscDivider = freq)
   else:
    self.clock()
  elif (freq > 0) and (freq < 8):
   self.SetLEDClkDivider(freq)

  #Configure LED driver linear/log mode
  tempByte = self._read_reg_8(_SX1509_RegMisc)
  if (log):
   tempByte |= (1<<7)	# set logarithmic mode bank B
   tempByte |= (1<<3)	# set logarithmic mode bank A
  else:
   tempByte &= ~(1<<7)	# set linear mode bank B
   tempByte &= ~(1<<3)	# set linear mode bank A
  self._write_reg_8(_SX1509_RegMisc, tempByte)

  #Enable LED driver operation (REG_LED_DRIVER_ENABLE)
  tempWord = self._read_reg_16(_SX1509_RegLEDDriverEnableB)
  tempWord |= (1<<pin)
  self._write_reg_16(_SX1509_RegLEDDriverEnableB, tempWord)

  #Set digital state of the LED pin. This must be set to zero (on).
  tempWord = self._read_reg_16(_SX1509_RegDataB)
  tempWord &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegDataB, tempWord)
  
  #Set the intensity to zero so the LEDs are off after initialization
  self.analogWrite(pin, 0)

 #Turns off the LED driver and reenables the input buffer on a pin. You will still need to set direction, pull-up, open drain, etc...
 def ledDriverDeinit(self, pin):
  #Disable LED driver operation (REG_LED_DRIVER_ENABLE)
  tempWord = self._read_reg_16(_SX1509_RegLEDDriverEnableB)
  tempWord &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegLEDDriverEnableB, tempWord)
  
  #Enable input buffer
  #Writing a 1 to the pin bit will disable that pins input buffer
  tempWord = self._read_reg_16(_SX1509_RegInputDisableB)
  tempWord &= ~(1<<pin)
  self._write_reg_16(_SX1509_RegInputDisableB, tempWord)

 def analogWrite(self, pin, iOn):
  #Write the on intensity of pin
  #Linear mode: Ion = iOn
  #Log mode: Ion = f(iOn)
  self._write_reg_8(_SX1509_RegIOn[pin], iOn)

 def blink(self, pin, tOn, tOff, onIntensity, offIntensity):
  onReg = self.calculateLEDTRegister(tOn)
  offReg = self.calculateLEDTRegister(tOff)
  self.setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0)

 #Changes
 # - linear/log option removed. It was not used anyway.
 def breathe(self, pin, tOn, tOff, rise, fall, onInt, offInt):
  onReg = self.calculateLEDTRegister(tOn)
  offReg = self.calculateLEDTRegister(tOff)
  riseTime = self.calculateSlopeRegister(rise, onInt, offInt)
  fallTime = self.calculateSlopeRegister(fall, onInt, offInt)
  self.setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime)#, log)

 #Changes
 # - linear/log option removed. It was not used anyway.
 def setupBlink(self, pin, tOn, tOff, onIntensity, offIntensity, tRise, tFall):#, log):
  #Check if the clock is on. If not, no reason to try to turn on an LED.
  if (self._clkX == 0):
   return

  #Keep parameters within their limits:
  tOn &= 0x1F	# tOn should be a 5-bit value
  tOff &= 0x1F	# tOff should be a 5-bit value
  offIntensity &= 0x07
  #Write the time on
  self._write_reg_8(_SX1509_RegTOn[pin], tOn)

  #Write the time/intensity off register
  self._write_reg_8(_SX1509_RegOff[pin], ((tOff<<3) | offIntensity) )

  #Write the on intensity:
  self._write_reg_8(_SX1509_RegIOn[pin], onIntensity)

  #Prepare tRise and tFall
  tRise &= 0x1F		#tRise is a 5-bit value
  tFall &= 0x1F		#tFall is a 5-bit value

  #Write regTRise
  if (_SX1509_RegTRise[pin] != 0xFF):
   self._write_reg_8(_SX1509_RegTRise[pin], tRise)

  if (_SX1509_RegTFall[pin] != 0xFF):
   self._write_reg_8(_SX1509_RegTFall[pin], tFall)

 #UNTESTED
 def keypad(self, rows, columns, sleepTime, scanTime, debounceTime):
  #unsigned int tempWord;
  #unsigned char tempByte;

  #If clock hasn't been set up, set it to internal 2MHz
  if (self._CPU_CLK == 0):
   self.clock()

  #TODO: Should this use the other basic functions instead of fiddling with the registers itself?
  #Set regDir 0:7 outputs, 8:15 inputs:
  tempWord = self._read_reg_16(_SX1509_RegDirB)
  for i in range(rows):
   tempWord &= ~(1<<i)
  for i in range(8,(columns * 2)):
   tempWord |= (1<<i)
  self._write_reg_16(_SX1509_RegDirB, tempWord)

  #Set regOpenDrain on 0:7:
  tempByte = self._read_reg_8(_SX1509_RegOpenDrainA)
  for i in range(rows):
   tempByte |= (1<<i)
  self._write_reg_8(_SX1509_RegOpenDrainA, tempByte)

  #Set regPullUp on 8:15:
  tempByte = self._read_reg_8(_SX1509_RegPullUpB)
  for i in range(columns):
   tempByte |= (1<<i);
  self._write_reg_8(_SX1509_RegPullUpB, tempByte)

  #Debounce Time must be less than scan time
  #TODO: Do I need these checks?
  #debounceTime = constrain(debounceTime, 1, 64);
  #scanTime = constrain(scanTime, 1, 128);
  if (debounceTime >= scanTime):
   debounceTime = scanTime >> 1 	#Force debounceTime to be less than scanTime
  self.debounceKeypad(debounceTime, rows, columns)

  #Calculate scanTimeBits, based on scanTime
  scanTimeBits = 0
  for i in range(7,0,-1):
   if (scanTime & (1<<i)):
    scanTimeBits = i
    break

  #Calculate sleepTimeBits, based on sleepTime
  sleepTimeBits = 0
  if (sleepTime != 0):
   for i in range(7,0,-1):
    if (sleepTime & (1<<(i+6))): #TODO: This might not work, C code had a typecast here...
     sleepTimeBits = i
     break
   #If sleepTime was non-zero, but less than 128, 
   #assume we wanted to turn sleep on, set it to minimum:
   if (sleepTimeBits == 0):
    sleepTimeBits = 1

  #RegKeyConfig1 sets the auto sleep time and scan time per row
  sleepTimeBits = (sleepTimeBits & 0b111)<<4
  scanTimeBits &= 0b111	#Scan time is bits 2:0
  tempByte = sleepTime | scanTimeBits
  self._write_reg_8(_SX1509_RegKeyConfig1, tempByte)

  #RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
  rows = (rows - 1) & 0b111			#0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
  columns = (columns - 1) & 0b111	#0b000 = 1 column, ob111 = 8 columns, etc.
  self._write_reg_8(_SX1509_RegKeyConfig2, (rows << 3) | columns);

 #UNTESTED
 def readKeypad(self):
  return (0xFFFF ^ (self._read_reg_16(_SX1509_RegKeyData1)))

 #UNTESTED
 def getRow(self, keyData):
  rowData = (keyData & 0x00FF)
  
  for i in range(8):
   if (rowData & (1<<i)):
    return i
  return 0

 #UNTESTED
 def getCol(self, keyData):
  colData = ((keyData & 0xFF00) >> 8)
  
  for i in range(8):
   if (colData & (1<<i)):
    return i
  return 0

 #UNTESTED
 # This requires a hardware reset line. _HW_Reset function must be available to use this.
 # TODO: Add a check to see if hardware reset is available? Not sure how to do this.
 def sync(self):
  #First check if nReset functionality is set
  regMisc = self._read_reg_8(_SX1509_RegMisc)
  if ((regMisc & 0x04) == 0):
   #If bit 2 is zero, the NRESET pin is used as a reset pin.
   #Need to set this bit to one temporarially to use the sync function.
   self._write_reg_8(_SX1509_RegMisc, (regMisc | 0x04))

  self._HW_Reset()

  #Return nReset to whatever it was previously
  self._write_reg_8(_SX1509_RegMisc, regMisc)

 def debounceConfig(self, configValue):
  #Check if the clock is running. If not, set to the default of 2MHz.
  if (self._CPU_CLK == 0):
   self.clock()
 
  configValue &= 0b111	#The debounce setting must be a 3-bit value
  self._write_reg_8(_SX1509_RegDebounceConfig, configValue)

 #Set the debounce time for all inputs that are debounced.
 #'time' is the debounce time in milliseconds.
 #Actual debounce times depend on the CPU clock according to the equation.
 #  X*(2MHz)/F_CPU
 #Where X=.5, 1, 2, 4, 8, 16, 32, 64 depending on the bits of RegDebounceConfig
 #
 #Changes:
 # - Updated to handle other CPU speeds.
 def debounceTime(self, time):
  if (self._CPU_CLK == 0): # If clock hasn't been set up.
   self.clock() # Set clock to 2MHz.

  configValue = 0
  errorValue = 10000 #Set this to a number that will always be above the calculated error
  
  #Modify this to deal with other CPU speeds. Not sure if this is a good way to do this or not.
  #Step through all possible times and track the one with the minimum error from the requested value.
  for i in range(8):
   RealDebounceTime = 0.5*(1<<i)*(2000000.0/self._CPU_CLK)
   DebounceError = abs(time-RealDebounceTime)
   if(DebounceError < errorValue):
    configValue = i
    errorValue = DebounceError

  self.debounceConfig(configValue)

 def debouncePin(self, pin):
  debounceEnable = self._read_reg_16(_SX1509_RegDebounceEnableB)
  debounceEnable |= (1<<pin)
  self._write_reg_16(_SX1509_RegDebounceEnableB, debounceEnable)

 #UNTESTED
 def debounceKeypad(self, time, numRows, numCols):
  #Set up debounce time
  self.debounceTime(time)

  #Set up debounce pins:
  for i in range(numRows):
   self.debouncePin(i)
  for i in range((8 + numCols)):
   self.debouncePin(i)

 def enableInterrupt(self, pin, riseFall):
  # Set REG_INTERRUPT_MASK
  tempWord = self._read_reg_16(_SX1509_RegInterruptMaskB)
  tempWord &= ~(1<<pin)	# 0 = event on IO will trigger interrupt
  self._write_reg_16(_SX1509_RegInterruptMaskB, tempWord)

  if (riseFall > 3) or (riseFall < 0):
   #This should not happen. Setting sensivity to zero here will unmask the interrupt but not set any conditions to trigger the interrupt
   sensitivity = 0
  else:
   sensitivity = riseFall
  
  pinMask = (pin & 0x07) * 2

  #Need to select between two words. One for bank A, one for B.
  if (pin >= 8):
   senseRegister = _SX1509_RegSenseHighB
  else:
   senseRegister = _SX1509_RegSenseHighA

  tempWord = self._read_reg_16(senseRegister)
  tempWord &= ~(0b11<<pinMask)					#Mask out the bits we want to write
  tempWord |= (sensitivity<<pinMask)			#Add our new bits
  self._write_reg_16(senseRegister, tempWord)
  
 def disableInterrupt(self, pin):
  #Mask the interrput pin
  tempWord = self._read_reg_16(_SX1509_RegInterruptMaskB)
  tempWord |= (1<<pin)		#Setting a 1 here masks this pin from causing interrupts
  self._write_reg_16(_SX1509_RegInterruptMaskB, tempWord)

  pinMask = (pin & 0x07) * 2

  #Need to select between two words. One for bank A, one for B.
  if (pin >= 8):
   senseRegister = _SX1509_RegSenseHighB
  else:
   senseRegister = _SX1509_RegSenseHighA

  tempWord = self._read_reg_16(senseRegister)
  tempWord &= ~(0b11<<pinMask)					#Clear the sense bits for this pin.
  self._write_reg_16(senseRegister, tempWord)

 def interruptSource(self, clear=True):
  intSource = self._read_reg_16(_SX1509_RegInterruptSourceB)
  if (clear):
   self._write_reg_16(_SX1509_RegInterruptSourceB, 0xFFFF)	# Clear interrupts
  
  return intSource

 def checkInterrupt(self, pin):
  if (interruptSource(false) & (1<<pin)):
   return True
  return False

 #Changes:
 # - Added a CPUFreq variable to allow the use of external oscilators
 # - Added a _CPU_CLK internal variable to track the CPU clock speed
 def clock(self, oscSource=2, oscDivider=1, oscPinFunction=0, oscFreqOut=0, CPUFreq = 2000000.0):
  #Check the defined clock source and set the CPU clock variable
  if oscSource == 2:
   self._CPU_CLK = 2000000.0
  elif oscSource == 1:
   self._CPU_CLK = CPUFreq
  else:
   self._CPU_CLK = 0
  
  #RegClock constructed as follows:
  # 6:5 - Oscillator frequency souce
  #  00: off, 01: external input, 10: internal 2MHz, 1: reserved
  # 4 - OSCIO pin function
  #  0: input, 1 ouptut
  # 3:0 - Frequency of oscout pin
  #  0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
  oscSource = (oscSource & 0b11)<<5				#2-bit value, bits 6:5
  oscPinFunction = (oscPinFunction & 1)<<4		#1-bit value bit 4
  oscFreqOut = (oscFreqOut & 0b1111)			#4-bit value, bits 3:0
  regClock = oscSource | oscPinFunction | oscFreqOut
  self._write_reg_8(_SX1509_RegClock, regClock)
  
  self.SetLEDClkDivider(oscDivider)

 def calculateLEDTRegister(self, ms):
  #ms is time in milliseconds
  #int regOn1, regOn2;
  #float timeOn1, timeOn2;

  if (self._clkX == 0):
   return 0

  regOn1 = (ms / 1000.0) / (64.0 * 255.0 / self._clkX)
  regOn2 = round((regOn1/8))
  regOn1 = round(regOn1)
    
  #reg1 must be 1-15, reg2 must be 16-31
  if regOn1 > 15:
   regOn1 = 15
  elif regOn1 < 1:
   regOn1 = 1
   
  if regOn2 > 31:
   regOn2 = 31
  elif regOn2 < 16:
   regOn2 = 16

  #print(' reg1: '+str(regOn1)+' reg2: '+str(regOn2))

  timeOn1 = 64.0 * regOn1 * 255.0 / self._clkX * 1000.0
  timeOn2 = 512.0 * regOn2 * 255.0 / self._clkX * 1000.0

  if (abs(timeOn1 - ms) < abs(timeOn2 - ms)):
   return regOn1
  else:
   return regOn2

 def calculateSlopeRegister(self, ms, onIntensity, offIntensity):
  #ms is fade in/out time in milliseconds

  if (self._clkX == 0):
   return 0

  tFactor = (onIntensity - (4.0 * offIntensity)) * 255.0 / self._clkX
  timeS = ms / 1000.0

  regSlope1 = timeS / tFactor
  regSlope2 = regSlope1 / 16

  #Registers must be integers
  regSlope1 = round(regSlope1)
  regSlope2 = round(regSlope2)

  #regSlope1 must be 1-15, regSlope2 must be 16-31
  if regSlope1 > 15:
   regSlope1 = 15
  elif regSlope1 < 1:
   regSlope1 = 1
   
  if regSlope2 > 31:
   regSlope2 = 31
  elif regSlope2 < 16:
   regSlope2 = 16

  regTime1 = regSlope1 * tFactor * 1000.0
  regTime2 = 16 * regTime1

  if (abs(regTime1 - ms) < abs(regTime2 - ms)):
   return regSlope1
  else:
   return regSlope2

 #Set LED Clock divider
 # oscDivider must be between zero and seven.
 # Setting oscDivider to zero will turn off the LED clock.
 def SetLEDClkDivider(self, oscDivider):
  #Config RegMisc[6:4] with oscDivider
  #0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
  if(oscDivider == 0):
   self._clkX = 0
  else:
   self._clkX = self._CPU_CLK / (1<<(oscDivider - 1))	#Update private clock variable
  
  oscDivider = (oscDivider & 0b111)<<4					# 3-bit value, bits 6:4
  
  regMisc = self._read_reg_8(_SX1509_RegMisc)
  regMisc &= ~(0b111<<4)
  regMisc |= oscDivider
  self._write_reg_8(_SX1509_RegMisc, regMisc)

 #Hardware specific functions
 def _HW_Reset(self):
  #Code here should set the reset line low and hold it for a few 100ms.
  pass
 
 def _write_reg_8(self, reg, val):
  self.i2c_device.write(bytes([reg, val]))
  
 def _write_reg_16(self, reg, val):
  self.i2c_device.write(bytes([reg, ((val>>8)&(0xFF)), (val&0xFF)]))

 def _read_reg_8(self, reg):
  self.i2c_device.write(bytes([reg]))
  result = bytearray(1)
  self.i2c_device.readinto(result)
  return result[0]
  
 def _read_reg_16(self, reg):
  self.i2c_device.write(bytes([reg]))
  result = bytearray(2)
  self.i2c_device.readinto(result)
  return ((result[0]<<8) | result[1])
