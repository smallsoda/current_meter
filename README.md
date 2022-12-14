# Current meter
USB current meter is designed to measure the current of low-power devices

### Measurements
The meter has three operational ampifiers (OA) to extend current measurement range. Currents from 2 uA to 2 A can be measured. All OAs connected to the only one shunt resistor (1 Ohm).
- Amplifier 0 gain (OA0, microchip DA3): 1
- Amplifier 1 gain (OA1, microchip DA2): 40
- Amplifier 2 gain (OA2, microchip DA1): 391

### Modes
The meter transfers current data via USB (virtual COM-port). Meter modes can be changed by push button.

Long press changes **data format** (indicator: **LED1**):
- **TEXT** data
    - Number of LED blinks: :bulb:
    - Units: uA
    - Message example: `19037\r\n`
- **RAW JSON** data
    - Number of LED blinks: :bulb::bulb:
    - Units: ADC units (0 ... 4095)
    - Message example: `{"OA0": 23, "OA1": 944, "OA2": 2504}\r\n`
- **RAW SLIP** data
    - Number of LED blinks: :bulb::bulb::bulb:
    - Units: ADC units (0 ... 4095)
    - Message example: `C0 00 17 03 B0 09 C8 C0`
    - `uint16_t, big-endian`

Short press changes data **transmission period** (indicator: **LED2**):
- **1 ms** period
    - Number of LED blinks: :bulb:
    - Instant value in any mode
- **10 ms** period
    - Number of LED blinks: :bulb::bulb:
    - Average value out of 10 in *TEXT* mode, instant value in other modes
- **100 ms** period
    - Number of LED blinks: :bulb::bulb::bulb:
    - Average value out of 100 in *TEXT* mode, instant value in other modes

Meter mode settings are stored in EEPROM and saved after disconnection from USB

#### Device diagram
![Device diagram](https://github.com/smallsoda/current_meter/blob/master/pictures/meter.png?raw=true)

#### Connection diagram
The meter must be connected between the ground of the device whose current consumption is being measured and the ground of the power supply
![Connection diagram](https://github.com/smallsoda/current_meter/blob/master/pictures/connection.png?raw=true)

#### Current chart
Any software can be used to render current chart. For example, Serial Plotter Tool in Arduino IDE
![Current chart](https://github.com/smallsoda/current_meter/blob/master/pictures/chart.png?raw=true)
