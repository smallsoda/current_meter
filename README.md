# Current meter
USB current meter is designed to measure the current of low-power devices

#### Measurements
The meter has three operational ampifiers to extend current measurement range. Currents from 2 uA to 2 A can be measured. All OAs connected to the only one shunt resistor (1 Ohm).
- 1 amplifier gain: 391
- 2 amplifier gain: 40
- 3 amplifier gain: 1

#### Modes
The meter transfers current data via USB (virtual COM-port). Meter modes can be changed by push button.

Long press changes data format (***LED1***, *number of bulbs - number of LED1 blinks*):
- :bulb: - RAW data (*not implemented*)
- :bulb::bulb: - TEXT data (example: "10205\r\n")

Short press changes data transmission period (***LED2***, *number of bulbs - number of LED2 blinks*):
- :bulb: - period 1 ms
- :bulb::bulb: - period 10 ms (average value out of 10)
- :bulb::bulb::bulb: - period 100 ms (average value out of 100)

Meter mode settings are stored in EEPROM and saved after disconnection from USB

#### Device diagram
![Device diagram](https://github.com/smallsoda/temp/blob/master/pictures/meter.png?raw=true)

#### Connection diagram
The meter must be connected between the ground of the device whose current consumption is being measured and the ground of the power supply
![Connection diagram](https://github.com/smallsoda/temp/blob/master/pictures/connection.png?raw=true)

#### Current chart
Any software can be used to render current chart. For example, Serial Plotter Tool in Arduino IDE
![Current chart](https://github.com/smallsoda/temp/blob/master/pictures/chart.png?raw=true)
