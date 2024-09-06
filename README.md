# Heart Rate Sensor
CircuitPython library for the Piper Heart Rate (Pulse) Sensor module

The original Piper Heart Rate Sensor was module based on the Partron PPSI262 integrated photoplethysmography LED/Photodiode module.
Newer versions, and almost all sensors sold are based on an ATTINY uC acting as a i2c peripheral and controlling a set of LEDs and a sensitive phototransistor.  The latest version of this library is no longer compatible with the original PPSI262 sesnsor.

## Requirements
This library depends on the CircuitPython `busio` (i2c) and CircutPython `ulab` (numpy) libraries, and is connected via i2c.
