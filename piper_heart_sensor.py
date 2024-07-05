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


__version__ = "0.9.5"
__repo__ = "https://github.com/buildwithpiper/circuitpython-heart-sensor-library.git"


# ----------- Constants and Registers -----------
#i2c address
PPSI262_ADR = 0x5B

# The register map for this thing is insane, and fragged like crazy.  
# Every setting needs a mask, because these things are all over the place.
# The functions to set them read the full 24-bit value out, and then set only the
# relevant bits, then write it all back in
# And at this point, I only understand about half of what these all do.
FIFO_EN = 0
FIFO_EN_MASK = 0x40

ENABLE_ULP = 0
ENABLE_ULP_MASK = 0x20

RW_CONT = 0
RW_CONT_MASK = 0x10

SW_RESET = 0
SW_RESET_MASK = 8

TM_COUNT_RST = 0
TM_COUNT_RST_MASK = 2

SPI_REG_READ = 0
SPI_REG_READ_MASK = 1

LED2STC = 1
LED2STC_MASK = 0xFFFF

LED2ENDC = 2
LED2ENDC_MASK = 0xFFFF

LED1LEDSTC = 3
LED1LEDSTC_MASK = 0xFFFF

LED1LEDENDC = 4
LED1LEDENDC_MASK = 0xFFFF

ALED2STC = 5
ALED2STC_MASK = 0xFFFF

ALED2ENDC = 6
ALED2ENDC_MASK = 0xFFFF

LED3STC = 5
LED3STC_MASK = 0xFFFF

LED3ENDC = 6
LED3ENDC_MASK = 0xFFFF

LED1STC = 7
LED1STC_MASK = 0xFFFF

LED1ENDC = 8
LED1ENDC_MASK = 0xFFFF

LED2LEDSTC = 9
LED2LEDSTC_MASK = 0xFFFF

LED2LEDENDC = 0xA
LED2LEDENDC_MASK = 0xFFFF

ALED1STC = 0xB
ALED1STC_MASK = 0xFFFF

ALED1ENDC = 0xC
ALED1ENDC_MASK = 0xFFFF

LED2CONVST = 0xD
LED2CONVST_MASK = 0xFFFF

LED2CONVEND = 0xE
LED2CONVEND_MASK = 0xFFFF

LED3CONVST = 0xF
LED3CONVST_MASK = 0xFFFF

LED3CONVEND = 0x10
LED3CONVEND_MASK = 0xFFFF

ALED2CONVST = 0xF
ALED2CONVST_MASK = 0xFFFF

ALED2CONVEND = 0x10
ALED2CONVEND_MASK = 0xFFFF

LED1CONVST = 0x11
LED1CONVST_MASK = 0xFFFF

LED1CONVEND = 0x12
LED1CONVEND_MASK = 0xFFFF

ALED1CONVST = 0x13
ALED1CONVST_MASK = 0xFFFF

ALED1CONVEND = 0x14
ALED1CONVEND_MASK = 0xFFFF

PRPCOUNT = 0x1D
PRPCOUNT_MASK = 0xFFFF

TIMEREN = 0x1E
TIMEREN_MASK = 0x100

NUMAV = 0x1E
NUMAV_MASK = 0xF

TIA_GAIN_SEP3_MSB = 0x1F
TIA_GAIN_SEP3_MSB_MASK = 0x4000

TIA_CF_SEP3 = 0x1F
TIA_CF_SEP3_MASK = 0x3800

TIA_GAIN_SEP3_LSB = 0x1F
TIA_GAIN_SEP3_LSB_MASK = 0x700

TIA_GAIN_SEP2_MSB = 0x1F
TIA_GAIN_SEP2_MSB_MASK = 0x40

TIA_CF_SEP2 = 0x1F
TIA_CF_SEP2_MASK = 0x38

TIA_GAIN_SEP2_LSB = 0x1F
TIA_GAIN_SEP2_LSB_MASK = 7

ENSEPGAIN = 0x20
ENSEPGAIN_MASK = 0x8000

TIA_GAIN_SEP_MSB = 0x20
TIA_GAIN_SEP_MSB_MASK = 0x40

TIA_CF_SEP = 0x20
TIA_CF_SEP_MASK = 0x38

TIA_GAIN_SEP_LSB = 0x20
TIA_GAIN_SEP_LSB_MASK = 7

IFS_OFFDAC = 0x21
IFS_OFFDAC_MASK = 0x7000

FILTER_BW_0 = 0x21
FILTER_BW_0_MASK = 0x200

TIA_GAIN_MSB = 0x21
TIA_GAIN_MSB_MASK = 0x40

TIA_CF = 0x21
TIA_CF_MASK = 0x38

TIA_GAIN_LSB = 0x21
TIA_GAIN_LSB_MASK = 7

ILED3_LSB = 0x22
ILED3_LSB_MASK = 0xC00000

ILED2_LSB = 0x22
ILED2_LSB_MASK = 0x300000

ILED1_LSB = 0x22
ILED1_LSB_MASK = 0xC0000

ILED3_MSB = 0x22
ILED3_MSB_MASK = 0x3F000

ILED2_MSB = 0x22
ILED2_MSB_MASK = 0xFC0

ILED1_MSB = 0x22
ILED1_MSB_MASK = 0x3F

CONTROL_DYN_TX_0 = 0x23
CONTROL_DYN_TX_0_MASK = 0x100000

ILED_FS = 0x23
ILED_FS_MASK = 0x20000

ENSEPGAIN4 = 0x23
ENSEPGAIN4_MASK = 0x8000

CONTROL_DYN_BIAS = 0x23
CONTROL_DYN_BIAS_MASK = 0x4000

OSC_ENABLE = 0x23
OSC_ENABLE_MASK = 0x200

CONTROL_DYN_TIA = 0x23
CONTROL_DYN_TIA_MASK = 0x10

CONTROL_DYN_ADC = 0x23
CONTROL_DYN_ADC_MASK = 8

PDNRX = 0x23
PDNRX_MASK = 2

PDNAFE = 0x23
PDNAFE_MASK = 1

ILED4_MSB = 0x24
ILED4_MSB_MASK = 0x1F800

ILED4_LSB = 0x24
ILED4_LSB_MASK = 0x600

FIFO_TOGGLE = 0x28
FIFO_TOGGLE_MASK = 0x400000

DESIGN_ID = 0x28
DESIGN_ID_MASK = 0x1FFF80

SDOUT_TRISTATE_D = 0x29
SDOUT_TRISTATE_D_MASK = 0x80000

SDOUT_TRISTATE = 0x29
SDOUT_TRISTATE_MASK = 0x200

LED2VAL = 0x2A
LED2VAL_MASK = 0xFFFFFF

LED3VAL = 0x2B
LED3VAL_MASK = 0xFFFFFF

ALED2VAL = 0x2B
ALED2VAL = 0xFFFFFF

LED1VAL = 0x2C
LED1VAL_MASK = 0xFFFFFF

ALED1VAL = 0x2D
ALED1VAL_MASK = 0xFFFFFF

LED2_MINUS_ALED2VAL = 0x2E
LED2_MINUS_ALED2VAL_MASK = 0xFFFFFF

LED1_MINUS_ALED1VAL = 0x2F
LED1_MINUS_ALED1VAL_MASK = 0xFFFFFF

FILTER_BW_1 = 0x31
FILTER_BW_1_MASK = 0x800000

PD_DISCONNECT = 0x31
PD_DISCONNECT_MASK = 0x400

ENABLE_INPUT_SHORT = 0x31
ENABLE_INPUT_SHORT_MASK = 0x20

PROG_INT2_STC = 0x34
PROG_INT2_STC_MASK = 0xFFFF

PROG_INT2_ENDC = 0x35
PROG_INT2_ENDC_MASK = 0xFFFF

LED3LEDSTC = 0x36
LED3LEDSTC_MASK = 0xFFFF

LED3LEDENDC = 0x37
LED3LEDENDC_MASK = 0xFFFF

CLKDIV_TE = 0x39
CLKDIV_TE_MASK = 7

EARLY_OFFSET_DAC = 0x3A
EARLY_OFFSET_DAC_MASK = 0x100000

POL_OFFDAC_LED2 = 0x3A
POL_OFFDAC_LED2_MASK = 0x80000

I_OFFDAC_LED2_MID_A = 0x3A
I_OFFDAC_LED2_MID_A_MASK = 0x78000

POL_OFFDAC_AMB1 = 0x3A
POL_OFFDAC_AMB1_MASK = 0x4000

I_OFFDAC_LED2_MID_B = 0x3A
I_OFFDAC_LED2_MID_B_MASK = 0x3C00

POL_OFFDAC_LED1 = 0x3A
POL_OFFDAC_LED1_MASK = 0x200

I_OFFDAC_LED2_MID_C = 0x3A
I_OFFDAC_LED2_MID_C_MASK = 0x1E0

POL_OFFDAC_LED3 = 0x3A
POL_OFFDAC_LED3_MASK = 0x10

I_OFFDAC_LED3_MID = 0x3A
I_OFFDAC_LED3_MID_MASK = 0xF

POL_OFFDAC_AMB2 = 0x3A
POL_OFFDAC_AMB2_MASK = 0x10

I_OFFDAC_AMB2_MID = 0x3A
I_OFFDAC_AMB2_MID_MASK = 0xF

THR_DET_LOW_CODE = 0x3B
THR_DET_LOW_CODE_MASK = 0xFFFFFF

THR_DET_HIGH_CODE = 0x3C
THR_DET_HIGH_CODE_MASK = 0xFFFFFF

WM_MODE = 0x3D
WM_MODE_MASK = 0x80000

THR_DET_PHASE_SEL_BIT_7 = 0x3D
THR_DET_PHASE_SEL_BIT_7_MASK = 0x8000

THR_DET_PHASE_SEL_BIT_6 = 0x3D
THR_DET_PHASE_SEL_BIT_6_MASK = 0x4000

THR_DET_PHASE_SEL_BIT_5 = 0x3D
THR_DET_PHASE_SEL_BIT_5_MASK = 0x2000

THR_DET_PHASE_SEL_BIT_4 = 0x3D
THR_DET_PHASE_SEL_BIT_4_MASK = 0x1000

THR_DET_PHASE_SEL_BIT_3 = 0x3D
THR_DET_PHASE_SEL_BIT_3_MASK = 0x800

THR_DET_PHASE_SEL_BIT_2 = 0x3D
THR_DET_PHASE_SEL_BIT_2_MASK = 0x400

THR_DET_PHASE_SEL_BIT_1 = 0x3D
THR_DET_PHASE_SEL_BIT_1_MASK = 0x100

THR_DET_PHASE_SEL_BIT_8 = 0x3D
THR_DET_PHASE_SEL_BIT_8_MASK = 0x80

FIFO_EN_DEC = 0x3D
FIFO_EN_DEC_MASK = 0x40

DEC_EN = 0x3D
DEC_EN_MASK = 0x20

THR_DET_EN = 0x3D
THR_DET_EN_MASK = 0x10

DEC_FACTOR = 0x3D
DEC_FACTOR_MASK = 0xE

THR_DET_PHASE_SEL_BIT_0 = 0x3D
THR_DET_PHASE_SEL_BIT_0_MASK = 1

I_OFF_DAC_LED2_LSB_EXT = 0x3E
I_OFF_DAC_LED2_LSB_EXT_MASK = 0x1000

I_OFF_DAC_AMB_LSB_EXT = 0x3E
I_OFF_DAC_AMB_LSB_EXT_MASK = 0x800

I_OFF_DAC_LED1_LSB_EXT = 0x3E
I_OFF_DAC_LED1_LSB_EXT_MASK = 0x400

I_OFF_DAC_LED3_LSB_EXT = 0x3E
I_OFF_DAC_LED3_LSB_EXT_MASK = 0x100

I_OFF_DAC_LED2_MSB = 0x3E
I_OFF_DAC_LED2_MSB_MASK = 0x80

I_OFF_DAC_LED2_LSB = 0x3E
I_OFF_DAC_LED2_LSB_MASK = 0x40

I_OFF_DAC_AMB_MSB = 0x3E
I_OFF_DAC_AMB_MSB_MASK = 0x20

I_OFF_DAC_AMB_LSB = 0x3E
I_OFF_DAC_AMB_LSB_MASK = 0x10

I_OFF_DAC_LED1_MSB = 0x3E
I_OFF_DAC_LED1_MSB_MASK = 8

I_OFF_DAC_LED1_LSB = 0x3E
I_OFF_DAC_LED1_LSB_MASK = 4

I_OFF_DAC_LED3_MSB = 0x3E
I_OFF_DAC_LED3_MSB_MASK = 2

I_OFF_DAC_LED3_LSB = 0x3E
I_OFF_DAC_LED3_LSB_MASK = 1

AVG_LED2_MINUS_ALED2VAL = 0x3F
AVG_LED2_MINUS_ALED2VAL_MASK = 0xFFFFFF

AVG_LED1_MINUS_ALED1VAL = 0x40
AVG_LED1_MINUS_ALED1VAL_MASK = 0xFFFFFF

INT_MUX3 = 0x42
INT_MUX3_MASK = 0xC00000

INT_MUX2 = 0x42
INT_MUX2_MASK = 0x300000

FIFO_EARLY = 0x42
FIFO_EARLY_MASK = 0x7C000

REG_WM_FIFO = 0x42
REG_WM_FIFO_MASK = 0x3FC0

REG_FIFO_PERIOD = 0x42
REG_FIFO_PERIOD_MASK = 0x3FC0

INT_MUX1 = 0x42
INT_MUX1_MASK = 0x30

FIFO_PARTITION = 0x42
FIFO_PARTITION_MASK = 0xF

LED4LEDSTC = 0x43
LED4LEDSTC_MASK = 0xFFFF

LED4LEDENDC = 0x44
LED4LEDENDC_MASK = 0xFFFF

TG_PD1STC = 0x45
TG_PD1STC_MASK = 0xFFFF

TG_PD1ENDC = 0x46
TG_PD1ENDC_MASK = 0xFFFF

TG_PD2STC = 0x47
TG_PD2STC_MASK = 0xFFFF

TG_PD2ENDC = 0x48
TG_PD2ENDC_MASK = 0xFFFF

TG_PD3STC = 0x49
TG_PD3STC_MASK = 0xFFFF

TG_PD3ENDC = 0x4A
TG_PD3ENDC_MASK = 0xFFFF

EN_PROG_OUT1 = 0x4B
EN_PROG_OUT1_MASK = 0x100

CONTROL_DYN_VCM = 0x4B
CONTROL_DYN_VCM_MASK = 8

CONTROL_DYN_DLDO = 0x4B
CONTROL_DYN_DLDO_MASK = 4

CONTROL_DYN_ALDO = 0x4B
CONTROL_DYN_ALDO_MASK = 2

CONTROL_DYN_BG = 0x4B
CONTROL_DYN_BG_MASK = 1

TRIPLE_PD_ENABLE = 0x4E
TRIPLE_PD_ENABLE_MASK = 0x10

DUAL_PD_ENABLE = 0x4E
DUAL_PD_ENABLE_MASK = 8

SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP = 0x50
SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP_MASK = 0x20

CONTROL_DYN_TX_1 = 0x50
CONTROL_DYN_TX_1_MASK = 8

MASK_FIFO_RDY = 0x51
MASK_FIFO_RDY_MASK = 0x200

FORCE_FIFO_OFFSET = 0x51
FORCE_FIFO_OFFSET_MASK = 0x100

FIFO_OFFSET_TO_FORCE = 0x51
FIFO_OFFSET_TO_FORCE_MASK = 0xFF

DATA_RDY_STC = 0x52
DATA_RDY_STC_MASK = 0xFFFF

DATA_RDY_ENDC = 0x53
DATA_RDY_ENDC_MASK = 0xFFFF

MASK_PPG = 0x54
MASK_PPG_MASK = 0x7000

MASK1_PPG = 0x54
MASK1_PPG_MASK = 0x1C0

MASK2_PPG = 0x54
MASK2_PPG_MASK = 0x38

MASK3_PPG = 0x54
MASK3_PPG_MASK = 7

PROG_INT1_STC = 0x57
PROG_INT1_STC_MASK = 0xFFFF

PROG_INT1_ENDC = 0x58
PROG_INT1_ENDC_MASK = 0xFFFF

EN_AMB_LOOP = 0x60
EN_AMB_LOOP_MASK = 0x800

CHOOSE_AMB_PHASE = 0x60
CHOOSE_AMB_PHASE_MASK = 0x600

FREEZE_LOOP = 0x60
FREEZE_LOOP_MASK = 0x20

HYST_LOOP = 0x60
HYST_LOOP_MASK = 0x18

DYN_TIA_STC = 0x64
DYN_TIA_STC_MASK = 0xFFFF

DYN_TIA_ENDC = 0x65
DYN_TIA_ENDC_MASK = 0xFFFF

DYN_ADC_STC = 0x66
DYN_ADC_STC_MASK = 0xFFFF

DYN_ADC_ENDC = 0x67
DYN_ADC_ENDC_MASK = 0xFFFF

DYN_CLK_STC = 0x68
DYN_CLK_STC_MASK = 0xFFFF

DYN_CLK_ENDC = 0x69
DYN_CLK_ENDC_MASK = 0xFFFF

DEEP_SLEEP_STC = 0x6A
DEEP_SLEEP_STC_MASK = 0xFFFF

DEEP_SLEEP_ENDC = 0x6B
DEEP_SLEEP_ENDC_MASK = 0xFFFF

REG_POINTER_DIFF = 0x6D
REG_POINTER_DIFF_MASK = 0xFF

EN_DRV2_LED4 = 0x72
EN_DRV2_LED4_MASK = 0x80

EN_DRV2_LED3 = 0x72
EN_DRV2_LED3_MASK = 0x40

EN_DRV2_LED2 = 0x72
EN_DRV2_LED2_MASK = 0x20

EN_DRV2_LED1 = 0x72
EN_DRV2_LED1_MASK = 0x10

DIS_DRV1_LED4 = 0x72
DIS_DRV1_LED4_MASK = 8

DIS_DRV1_LED3 = 0x72
DIS_DRV1_LED3_MASK = 4

DIS_DRV1_LED2 = 0x72
DIS_DRV1_LED2_MASK = 2

DIS_DRV1_LED1 = 0x72
MASK_DIS_DRV1_LED1 = 1

# Internal oscillator frequency
INTERNAL_OSC_FREQ = 128000


# ----------- Methods -----------
# Get the whole 24-bit value of a register by it's address


class piper_heart_sensor:
    # Initialize the sensor
    def __init__(self, i2c, smoothing=3, pulse_rep_freq=250, samples_to_average=12, sample_depth=1, clock_divisor=1):

        self.i2c = i2c
        self.i2c.try_lock()
        
        # Clock and sampling related settings
        self.sample_depth = sample_depth                     # samples to retain in FIFO
        self.smoothing = int(smoothing)                      # how much to low-pass filter the output (range from 1-4 to be useful)
        if (self.smoothing < 1):
            self.smoothing = 1
        self.samples_to_average = int(samples_to_average)    # samples to avarage before storing in FIFO
        if (self.samples_to_average < 1):
            self.samples_to_average = 1
        self.clock_divisor = clock_divisor                   # Amount to divide the clock by
        self.pulse_rep_freq = pulse_rep_freq                 # Full sense/convert/store cycles per second
        self.heart_rate_measurement = None                   # Starting point

        # Global to keep track of the lat time the FIFO was read
        self.last_fifo_read = time.monotonic()

        # Determine the version of the sensor
        # Original version is the PPSI262 (True), the new version is Attiny-based (False)
        self.sensor_type = False   
        if (self.register_get(DESIGN_ID, DESIGN_ID_MASK) != 84):
            self.sensor_type = True

            # reset the device
            self.reset()
            time.sleep(0.01)

            # Divide the clock
            if (self.clock_divisor >= 16):
                self.register_set(CLKDIV_TE, CLKDIV_TE_MASK, 7)
                self.clock_divisor = 16
            elif (self.clock_divisor >= 8):
                self.register_set(CLKDIV_TE, CLKDIV_TE_MASK, 6)
                self.clock_divisor = 8
            elif (self.clock_divisor >= 4):
                self.register_set(CLKDIV_TE, CLKDIV_TE_MASK, 5)
                self.clock_divisor = 4
            elif (self.clock_divisor >= 2):
                self.register_set(CLKDIV_TE, CLKDIV_TE_MASK, 4)
                self.clock_divisor = 2
            else:
                self.register_set(CLKDIV_TE, CLKDIV_TE_MASK, 0)
                self.clock_divisor = 1

            _freq = INTERNAL_OSC_FREQ / self.clock_divisor
            _tick_period = 1000000 / _freq
            _prf_count = int(_freq / self.pulse_rep_freq) - 1


            self.register_set(NUMAV, NUMAV_MASK, int(self.samples_to_average - 1))  # Set the ADC output to be the average of [self.samples_to_average] samples

            # Setup LED current drivers and switching
            self.register_set(ILED_FS, ILED_FS_MASK, 1)             # Set the full-scale output of the LED current driver to 100mA (set to 0 for 50mA)
            self.register_set(EN_DRV2_LED1, EN_DRV2_LED1_MASK, 1)   # Enable the second current dirver for LED1 (Green)
            self.register_set(EN_DRV2_LED2, EN_DRV2_LED2_MASK, 1)   # Enable the second current dirver for LED1 (IR)

            # Green LED (LED1) - Should output 150mA according to datasheet
            self.register_set(ILED1_MSB, ILED1_MSB_MASK, 63)        # LED1's current to 100mA * 2 drivers = 200mA (high bits)
            self.register_set(ILED1_LSB, ILED1_LSB_MASK, 3)         # LED1's current to 100mA * 2 drivers = 200mA (low bits)

            # IR LED (LED2) - Should output 100 mA according to datasheet
            self.register_set(ILED2_MSB, ILED2_MSB_MASK, 63)        # LED2's current to 75mA * 2 drivers = 150mA (high bits)
            self.register_set(ILED2_LSB, ILED2_LSB_MASK, 3)         # LED2's current to 75mA * 2 drivers = 150mA (low bits)

            # Determine the sampling rate for the sensor (relative to the PRF?) - all 4 should be set to idential values
            self.register_set(MASK_PPG, MASK_PPG_MASK, 0b0)         # 3 bits wide - no idea what this does.
            self.register_set(MASK1_PPG, MASK1_PPG_MASK, 0b0)
            self.register_set(MASK2_PPG, MASK2_PPG_MASK, 0b0)
            self.register_set(MASK3_PPG, MASK3_PPG_MASK, 0b0)

            # Set the TIA (trans-impedence amplifier) gain - this is touchy.
            # Notes:  Too much capacitance seems to wipe out any useful signal.  
            # Too much resistance cranks the gain and saturates it, and too little doesn't lift the signal
            # above the noise floor
            self.register_set(TIA_GAIN_LSB, TIA_GAIN_LSB_MASK, 0b011)  # 3-bits (LSB) - resistor value
            self.register_set(TIA_GAIN_MSB, TIA_GAIN_MSB_MASK, 0)      # 1-bit (MSB)
            self.register_set(TIA_CF, TIA_CF_MASK, 0b000)              # 3-bits - capacitor value

            # Setup the FIFO
            # TODO: different FIFO configurations store different data - build functions/conditionals for setting this.
            self.register_set(FIFO_PARTITION, FIFO_PARTITION_MASK, 5)  # store (LED2—Ambient2), (LED1—Ambient1) ADC values
            self.register_set(REG_FIFO_PERIOD, REG_FIFO_PERIOD_MASK, self.sample_depth) # retain [self.sample_depth] sample cycles
            self.register_set(FIFO_EN, FIFO_EN_MASK, 1)                # turn on the FIFO

            # Do a bunch of calculations to set the timings for things based on the clock frequency, LED configuration, etc.
            # these are a bit generous - if you need a really high PRF - you could tighten some of these a bit
            # Constants here are microseconds.
            _active_phase_start = int(520 / _tick_period) + 1 # this one is very generous
            _led_on_width = int(70 / _tick_period) + 1
            _sampling_start_delay = int(30 / _tick_period) + 1
            _sampling_width = int(35 / _tick_period) + 1
            _convert_start_delay = _led_on_width + 2
            _convert_width = int((56.5 * self.samples_to_average + 72) / _tick_period) + 16 # and especially this one is too
            _phase_width = _convert_start_delay + _convert_width + 4
            _phase_0_start = 0 + _active_phase_start
            _phase_1_start = _phase_width * 1 + _active_phase_start
            _phase_2_start = _phase_width * 2 + _active_phase_start
            _phase_3_start = _phase_width * 3 + _active_phase_start

            #  TODO:
            #   If active_phase_end is greater than the PRF count, throw an error
            #   Set up deep sleep behavior options
            #   Set up external oscillator options

            _dont_activate = _prf_count + 16

            # Set the sensor to run a 500Hz PRF (Pulse Repetition Frequency) cycle
            self.register_set(PRPCOUNT, PRPCOUNT_MASK, _prf_count)

            self.register_set(ENABLE_ULP, ENABLE_ULP_MASK, 1)

            self.register_set(LED2LEDSTC, LED2LEDSTC_MASK, _phase_0_start)
            self.register_set(LED2LEDENDC, LED2LEDENDC_MASK, _phase_0_start + _led_on_width)
            self.register_set(LED2STC, LED2STC_MASK, _phase_0_start + _sampling_start_delay)
            self.register_set(LED2ENDC, LED2ENDC_MASK, _phase_0_start + _sampling_start_delay + _sampling_width)
            self.register_set(LED2CONVST, LED2CONVST_MASK, _phase_0_start + _convert_start_delay)
            self.register_set(LED2CONVEND, LED2CONVEND_MASK, _phase_0_start + _convert_start_delay + _convert_width)

            self.register_set(LED3LEDSTC, LED3LEDSTC_MASK, _dont_activate)      # Don't turn on
            self.register_set(LED3LEDENDC, LED3LEDENDC_MASK, _dont_activate)    # Don't turn on
            self.register_set(ALED2STC, ALED2STC_MASK, _phase_1_start + _sampling_start_delay)
            self.register_set(ALED2ENDC, ALED2ENDC_MASK, _phase_1_start + _sampling_start_delay + _sampling_width)
            self.register_set(ALED2CONVST, ALED2CONVST_MASK, _phase_1_start + _convert_start_delay)
            self.register_set(ALED2CONVEND, ALED2CONVEND_MASK, _phase_1_start + _convert_start_delay + _convert_width)

            self.register_set(LED1LEDSTC, LED1LEDSTC_MASK, _phase_2_start)
            self.register_set(LED1LEDENDC, LED1LEDENDC_MASK, _phase_2_start + _led_on_width)
            self.register_set(LED1STC, LED1STC_MASK, _phase_2_start + _sampling_start_delay)
            self.register_set(LED1ENDC, LED1ENDC_MASK, _phase_2_start + _sampling_start_delay + _sampling_width)
            self.register_set(LED1CONVST, LED1CONVST_MASK, _phase_2_start + _convert_start_delay)
            self.register_set(LED1CONVEND, LED1CONVEND_MASK, _phase_2_start + _convert_start_delay + _convert_width)

            self.register_set(LED4LEDSTC, LED4LEDSTC_MASK, _dont_activate)      # Don't turn on
            self.register_set(LED4LEDENDC, LED4LEDENDC_MASK, _dont_activate)    # Don't turn on
            self.register_set(ALED1STC, ALED1STC_MASK, _phase_3_start + _sampling_start_delay)
            self.register_set(ALED1ENDC, ALED1ENDC_MASK, _phase_3_start + _sampling_start_delay + _sampling_width)
            self.register_set(ALED1CONVST, ALED1CONVST_MASK, _phase_3_start + _convert_start_delay)
            self.register_set(ALED1CONVEND, ALED1CONVEND_MASK, _phase_3_start + _convert_start_delay + _convert_width)


            self.register_set(DATA_RDY_STC, DATA_RDY_STC_MASK, _phase_3_start + _convert_start_delay + _convert_width + 6)
            self.register_set(DATA_RDY_ENDC, DATA_RDY_ENDC_MASK, _phase_3_start + _convert_start_delay + _convert_width + 7)

            # Active phase start and end timings
            self.register_set(DYN_TIA_STC, DYN_TIA_STC_MASK, 0)
            self.register_set(DYN_TIA_ENDC, DYN_TIA_ENDC_MASK, _dont_activate)
            self.register_set(DYN_ADC_STC, DYN_ADC_STC_MASK, 0)
            self.register_set(DYN_ADC_ENDC, DYN_ADC_ENDC_MASK, _dont_activate)
            self.register_set(DYN_CLK_STC, DYN_CLK_STC_MASK, 0)
            self.register_set(DYN_CLK_ENDC, DYN_CLK_ENDC_MASK, _dont_activate)

            # Deep sleep behavior
            # Not used - we aren't in a battery-powered application where power is a concern.
            #self.register_set(CONTROL_DYN_TIA, CONTROL_DYN_TIA_MASK, 0)
            #self.register_set(CONTROL_DYN_ADC, CONTROL_DYN_ADC_MASK, 0)
            #self.register_set(CONTROL_DYN_ALDO, CONTROL_DYN_ALDO_MASK, 0)
            #self.register_set(CONTROL_DYN_DLDO, CONTROL_DYN_DLDO_MASK, 0)
            #self.register_set(SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP, SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP_MASK, 1)

            # Deep sleep phase start and end timings
            self.register_set(DEEP_SLEEP_STC, DEEP_SLEEP_STC_MASK, _dont_activate)    # Don't go into deep sleep
            self.register_set(DEEP_SLEEP_ENDC, DEEP_SLEEP_ENDC_MASK, _dont_activate)  # Don't go into deep sleep

            # Enable the internal oscillator
            self.register_set(OSC_ENABLE, OSC_ENABLE_MASK, 1)    # Turn on the internal oscillator

        else:
            self.sample_depth = 1

        self.i2c.unlock()

    def readcsr(self, addr):
        _csr = bytearray(3)
        self.i2c.writeto_then_readfrom(PPSI262_ADR, bytes([addr]), _csr)
        _value = int.from_bytes(_csr, 'big')
        return _value

    # Set the whole 24-bit value of a register by it's address
    def writecsr(self, addr, value):
        _csr = ((addr << 24) + value).to_bytes(4, 'big')
        self.i2c.writeto(PPSI262_ADR, _csr)

    # Returns the index, counting from 0, of the least significant set bit in `x`.
    def ffs(self, x):
        return (x&-x).bit_length()-1

    # Invert the bits in a 24-bit (or otherwise specified) number
    def bit_not(self, n, numbits=24):
        return (1 << numbits) - 1 - n

    # Get the value of a specific register by it's name and mask
    def register_get(self, addr, mask, shift=True):
        _csr = bytearray(3)
        self.i2c.writeto_then_readfrom(PPSI262_ADR, bytes([addr]), _csr)
        _value = (int.from_bytes(_csr, 'big') & mask)
        if (shift == True):
            _value = _value >> self.ffs(mask)
        return _value

    # Set the value of a specific register by it's name and mask
    def register_set(self, addr, mask, value):
        _csr = self.register_get(addr, 0xFFFFFF, False)
        _csr = _csr & self.bit_not(mask)
        value = _csr | (value << self.ffs(mask))
        _csr = ((addr << 24) | value).to_bytes(4, 'big')
        self.i2c.writeto(PPSI262_ADR, _csr)

    # read the FIFO
    def read_fifo(self):
        _time_now = time.monotonic()
        while (self.last_fifo_read > _time_now):
            _time_now = time.monotonic()

        # Wait for the (sample count * cycle period) before trying to read from the FIFO again
        self.last_fifo_read = _time_now + self.samples_to_average/self.pulse_rep_freq

        _led2_values = []
        _led1_values = []

        _csr = bytearray(6 * self.sample_depth)
        self.i2c.writeto_then_readfrom(PPSI262_ADR, bytes([0xFF]), _csr)

        # TODO: There are different configurations for how the data is stored in the FIFO - create different conditions for them
        for i in range(self.sample_depth):
            j = i * 6
            _led2 = int.from_bytes(_csr[j:(j+3)], 'big')
            _led1 = int.from_bytes(_csr[(j+3):(j+6)], 'big')

            #The values from the Attiny version of the sensor will be smaller, so multiply them by 8
            #if (self.sensor_type == False):
            #    _led2 = _led2 << 3
            #    _led1 = _led1 << 3

            _led2_values.append(_led2)
            _led1_values.append(_led1)

        return [_led1_values, _led2_values]

    # heart rate calculation
    def heart_beat_detect(self, current, upper, lower):
        if (current > upper): 
            self.peak_detect = 1
        elif (current < lower):
            self.peak_detect = -1
        else:
            self.peak_detect = 0

        if (self.peak_detect == 1 and self.last_peak_state == -1):
            # rising
            self.last_peak_state = 1
        elif (self.peak_detect == -1 and self.last_peak_state == 1):
            # falling
            self.last_peak_state = -1
            _mark = time.monotonic()
            if (self.last_peak_mark is not None):
                _tmp_hr = _mark - self.last_peak_mark
                _tmp_hr = int(60 * (1 / _tmp_hr))
                if (_tmp_hr <= 220 and _tmp_hr >= 35):
                    self.heart_rate_measurement = _tmp_hr
            self.last_peak_mark = _mark

    # get smoothed values
    def read_sensor(self):    
        if (self.sensor_type == True):
            for _z in range(2):  # take 2 samples - helps with smoothing/averaging
                _sensor_output = self.read_fifo()[0][0]
                self.last_reading.append(_sensor_output)
                self.last_reading.pop(0)

                self.reading_average = (self.reading_average * self.smoothing * 10 + _sensor_output) / (self.smoothing * 10 + 1)
                
                self.std_dev_readings.append(_sensor_output - self.reading_average)
                self.std_dev_readings.pop(0)

                _current_reading = ulab.numpy.mean(self.last_reading) - self.reading_average
                    
                self.heart_beat_detect(_current_reading, ulab.numpy.max(self.std_dev_readings) * 0.1, ulab.numpy.min(self.std_dev_readings) * 0.1)

            return int(_current_reading / 10)
        
        else:
            _sensor_output = self.read_fifo()[0][0]
            self.last_reading.append(_sensor_output)
            self.last_reading.pop(0)

            _current_reading = self.last_reading[-1] + self.last_reading[-4] - self.last_reading[-2] - self.last_reading[-3]
            self.std_dev_readings.append(_current_reading)
            self.std_dev_readings.pop(0)

            _threshold = max(ulab.numpy.max(self.std_dev_readings), -ulab.numpy.min(self.std_dev_readings)) / 2

            self.heart_beat_detect(_current_reading, _threshold, -_threshold)

            return _sensor_output


    # Return the measured heart rate
    @property
    def heart_rate(self):
        return self.heart_rate_measurement

    # Preform a software reset
    def reset(self):
        if (self.sensor_type == True):
            self.register_set(SW_RESET, SW_RESET_MASK, 1)

    # Start the sensor
    def start(self):
        self.i2c.try_lock()

        # Peak/zero-crossing detection for calculating heart rate from the data
        self.peak_detect = None
        self.last_peak_state = 1
        self.last_peak_mark = None

        # Build some FIFO arrays for averaging/smoothing the data from the sensor
        self.last_reading = [0] * (self.smoothing * 30)
        self.std_dev_readings = [0] * (self.smoothing * 10)
        self.reading_average = 0

        # start the timer to begin operation
        if (self.sensor_type == True):
            self.register_set(TIMEREN, TIMEREN_MASK, 1)

        for _z in range(35):
            _sensor_output = self.read_fifo()[0][0]

            self.reading_average = self.reading_average + _sensor_output / 30
            self.last_reading.append(_sensor_output)
            self.last_reading.pop(0)

            if (self.sensor_type == False and _z > 4):
                self.std_dev_readings.append(self.last_reading[-1] + self.last_reading[-4] - self.last_reading[-2] - self.last_reading[-3])
                self.std_dev_readings.pop(0)

            # TODO: This would be a good place to do some twiddling of the TIA gain and DC offset registers
            # Auto-calibrate?

    # Stop the sensor
    def stop(self):
        if (self.sensor_type == True):
            self.register_set(TIMEREN, TIMEREN_MASK, 0)    # stop the timer to end operation
        self.i2c.unlock()

    # Allows for use in context managers.
    def __enter__(self):
        return self

    # Automatically de-initialize after a context manager.
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.deinit()

    # De-initialize the sig pin.
    def deinit(self):
        self.i2c.deinit()
