################################################################################
# The MIT License (MIT)
#
# Copyright (c) 2024 Piper Learning, Inc. and Matthew Matz
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
################################################################################

import busio
import board
import time
import ulab

__version__ = "1.1.0"
__repo__ = "https://github.com/buildwithpiper/circuitpython-heart-sensor-library.git"


#i2c address
HEART_SENSOR_ADDR = 0x5B

class piper_heart_sensor:
    # Initialize the sensor
    def __init__(self, i2c, smoothing=3, channel=0):

        self.i2c = i2c
        
        # Clock and sampling related settings
        self.smoothing = int(smoothing)                      # how much to low-pass filter the output (range from 1-4 to be useful)
        if (self.smoothing < 1):
            self.smoothing = 1
        self.heart_rate_measurement = None                   # Starting point

        # Specify which LED's data will be used
        self.channel = channel

        # Peak/zero-crossing detection for calculating heart rate from the data
        self.peak_detect = None
        self.last_peak_state = 1
        self.last_peak_mark = None

        # Build some FIFO arrays for averaging/smoothing the data from the sensor
        self.last_reading = [0] * (self.smoothing * 30)
        self.std_dev_readings = [0] * (self.smoothing * 10)

        # check sensor version
        __csr = bytearray(3)
        while not self.i2c.try_lock():
            pass        
        self.i2c.writeto_then_readfrom(HEART_SENSOR_ADDR, bytes([0x28]), __csr)
        __value = (int.from_bytes(__csr, 'big') & 0x1FFF80) >> 7  # mask is 0x1FFF80, shift 7 bits to get value
        self.i2c.unlock()
        if (__value != 84):
            raise Exception("This Heartrate Sensor (" + str(__value) + ") is no longer supported - please contact support: hi@playpiper.com")

        # read a number of samples to determine averages/deviation/etc.
        for _z in range(35):
            self.read_fifo()
            if (_z > 4):
                self.process_reading()

    # read raw sensor values
    def read_fifo(self):
        # read the sensor values from the registers
        __csr = bytearray(6)
        while not self.i2c.try_lock():
            pass        
        self.i2c.writeto_then_readfrom(HEART_SENSOR_ADDR, bytes([0xFF]), __csr)
        self.i2c.unlock()

        # separate the values from each LED/channel and store them
        __led_values = [
            int.from_bytes(__csr[0:3], 'big'), 
            int.from_bytes(__csr[3:6], 'big'), 
            int.from_bytes(__csr[6:9], 'big')
        ]
        self.last_reading.append(__led_values[self.channel])
        self.last_reading.pop(0)

        return __led_values

    # heart rate calculation
    def heart_beat_detect(self, current, upper, lower):
        # determine peak state
        if (current > upper): 
            self.peak_detect = 1
        elif (current < lower):
            self.peak_detect = -1
        else:
            self.peak_detect = 0

        # rising
        if (self.peak_detect == 1 and self.last_peak_state == -1):
            self.last_peak_state = 1

        # falling
        elif (self.peak_detect == -1 and self.last_peak_state == 1):
            self.last_peak_state = -1

            # calculate heart rate
            __mark = time.monotonic()
            if (self.last_peak_mark is not None):
                __tmp_hr = __mark - self.last_peak_mark
                __tmp_hr = int(60 * (1 / __tmp_hr))
                if (__tmp_hr <= 220 and __tmp_hr >= 35):
                    self.heart_rate_measurement = __tmp_hr
            self.last_peak_mark = __mark

    def process_reading(self):
        # use recent readings to detemine the standard deviation for heart beat detection
        __current_reading = self.last_reading[-1] + self.last_reading[-4] - self.last_reading[-2] - self.last_reading[-3]
        self.std_dev_readings.append(__current_reading)
        self.std_dev_readings.pop(0)
        return __current_reading

    # get smoothed values
    def read_sensor(self):    
        self.read_fifo()
        __current_reading = self.process_reading()
        __threshold = max(ulab.numpy.max(self.std_dev_readings), -ulab.numpy.min(self.std_dev_readings)) / 2
        self.heart_beat_detect(__current_reading, __threshold, -__threshold)
        return self.last_reading[-1]

    def set_channel(self, channel=0):
        # make sure channel is 0, 1, or 2
        self.channel = max(min(int(channel),2),0)

    # backward compatibility
    def start(self):
        pass

    # Return the measured heart rate
    @property
    def heart_rate(self):
        return self.heart_rate_measurement

    # Allows for use in context managers.
    def __enter__(self):
        return self

    # Automatically de-initialize after a context manager.
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.deinit()

    # De-initialize the sig pin.
    def deinit(self):
        self.i2c.deinit()