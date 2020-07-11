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

#Various useful variables used in the SX1508 functions

#Pin types
PIN_TYPE_INPUT = 0x00
PIN_TYPE_OUTPUT = 0x01
PIN_TYPE_INPUT_PULLUP = 0x02
PIN_TYPE_ANALOG_OUTPUT = 0x03
PIN_TYPE_INPUT_PULLDOWN = 0x04
PIN_TYPE_INPUT_OPEN_DRAIN = 0x05

#Interrupt states
INTERRUPT_STATE_CHANGE = 0b11
INTERRUPT_STATE_FALLING = 0b10
INTERRUPT_STATE_RISING = 0b01
