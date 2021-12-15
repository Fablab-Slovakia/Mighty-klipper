# Support for SPI based ADS11182 ADC temperature sensors
#
# Copyright (C) 2021  Ricardo Alcantara <ricardo@vulcanolabs.xyz>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus
import ads1118helper

ADS11182_REPORT_TIME = .3

ENABLE_PULL_UP = (1<<3)
START_SINGLE_SHOT = ((1<<15) | (1<<8))

CHANNEL_1_CFG = ( 0x0C62 | ENABLE_PULL_UP )
CHANNEL_2_CFG = ( 0x3C62 | ENABLE_PULL_UP )
INTERNAL_T = ( 0x0072 | ENABLE_PULL_UP )

TEMP_TABLE = [
    0,
    10000,
    20000,
    30000,
    40000,
    50000,
    60000,
    79000,
    98000,
    116000,
    134000,
    139000,
    155000,
    172000,
    193000,
    212000,
    231000,
    250000,
    269000,
    288000,
    307000,
    326000,
    345000,
    364000,
    383000,
    402000,
    421000,
    440000,
    459000,
    478000,
    497000,
    516000,
    535000,
    554000,
    573000,
    592000,
    611000,
    630000,
    649000,
    668000,
    687000,
    706000,
    725000,
    744000,
    763000,
    782000,
    801000,
    820000,
    839000,
    858000,
    877000,
    896000,
    915000,
    934000,
    953000,
    972000,
    991000,
    1010000,
    1029000,
    1048000,
    1067000,
    1086000,
    1105000,
    1124000,
    1143000,
    1200000
]

VOLT_TABLE = [
    0,
    397,
    798,
    1203,
    1612,
    2023,
    2436,
    3225,
    4013,
    4756,
    5491,
    5694,
    6339,
    7021,
    7859,
    8619,
    9383,
    10153,
    10930,
    11712,
    12499,
    13290,
    14084,
    14881,
    15680,
    16482,
    17285,
    18091,
    18898,
    19707,
    20516,
    21326,
    22137,
    22947,
    23757,
    24565,
    25373,
    26179,
    26983,
    27784,
    28584,
    29380,
    30174,
    30964,
    31752,
    32536,
    33316,
    34093,
    34867,
    35637,
    36403,
    37166,
    37925,
    38680,
    39432,
    40180,
    40924,
    41665,
    42402,
    43134,
    43863,
    44588,
    45308,
    46024,
    46735,
    48838
]

class ADS11182:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.spi = bus.MCU_SPI_from_config(
            config, 1, pin_option="sensor_pin", default_speed=4000000)
        self.mcu = self.spi.get_mcu()
        self.temp = self.min_temp = self.max_temp = 0.0
        self.channel = config.getint('ads11182_channel')
        self.sample_timer = self.reactor.register_timer(self._sample_ads11182)
        self.printer.add_object("ads11182 " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_ads11182()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return ADS11182_REPORT_TIME

    def _init_ads11182(self):
        # Check and report the chip ID but ignore errors since many
        # chips don't have it
        try:
            self.spi.spi_send([(INTERNAL_T | START_SINGLE_SHOT) >> 8, (INTERNAL_T | START_SINGLE_SHOT) & 0xff])
            logging.info("ads11182: initialized")
        except:
            pass

    def _sample_ads11182(self, eventtime):
        try:
            self.temp  = self.read_temp(self.channel)
            #logging.info("temp: %f", self.temp)
        except Exception:
            logging.exception("ads11182: Error reading data")
            self.temp = 0.0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "ADS11182 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + ADS11182_REPORT_TIME

    def read_temp(self, channel):
        #logging.info("reading internal temp")
        self.spi.spi_send([(INTERNAL_T | START_SINGLE_SHOT) >> 8, (INTERNAL_T | START_SINGLE_SHOT) & 0xff])
        self.reactor.pause(self.reactor.monotonic() + .06)
        raw_it_res = 0
        if channel == 1:
            raw_it_res = self.spi.spi_transfer([(CHANNEL_1_CFG | START_SINGLE_SHOT) >> 8, (CHANNEL_1_CFG | START_SINGLE_SHOT) & 0xff])
        if channel == 2:
            raw_it_res = self.spi.spi_transfer([(CHANNEL_2_CFG | START_SINGLE_SHOT) >> 8, (CHANNEL_2_CFG | START_SINGLE_SHOT) & 0xff])
        raw_it = bytearray(raw_it_res['response'])
        raw_it = (raw_it[0] << 8) | raw_it[1]
        #logging.info("raw internal temp: %s", hex(raw_it))
        #logging.info("reading tc")
        self.reactor.pause(self.reactor.monotonic() + .06)
        raw_tc_res = self.spi.spi_transfer([(INTERNAL_T | START_SINGLE_SHOT) >> 8, (INTERNAL_T | START_SINGLE_SHOT) & 0xff])
        raw_tc = bytearray(raw_tc_res['response'])
        raw_tc = (raw_tc[0] << 8) | raw_tc[1]
        #logging.info("raw tc: %s", hex(raw_tc))
        tc_uv = self._raw_to_uv(raw_tc)
        #logging.info("tc uv: %f", tc_uv)
        it_c = self._raw_to_c(raw_it)
        #logging.info("it deg c: %f", it_c)

        if channel == 1:
            temp= ads1118helper.temp1
        if channel == 2:
            temp= ads1118helper.temp2

        return temp

    def _raw_to_uv(self, raw):
        # negative unsupported
        if raw & 0x8000:
            return 0
        # (raw*256000)/32768
        current_uv = (raw*1000)>>7
        # open thermocouple ? 41276uV => 1000
        if current_uv >= 41276:
            return 0 # show ambient only
        return current_uv

    def _raw_to_c(self, raw):
        # 14-bit result that is left-justified
        raw >>= 2
        # negative unsupported
        if raw & 0x2000:
          return 25000
        # LSB: 0.03125
        return ( raw*3125 ) / 100

    # Returns temperature as a function of the ambient temperature
    # and measured thermocouple voltage.
    # Currently only positive ambient temperature is supported
    def _tc_cj_comp(self, measured_uv, ambient):
        # Convert ambient temp to microvolts
        # and add them to the thermocouple measured microvolts 
        microvolts = measured_uv + self.interpolateVoltage(ambient, self.searchTemp(ambient))

        # look up microvolts in The Table and interpolate
        return self.interpolateTemperature(microvolts, self.searchMicrovolts(microvolts))


    def interpolate(self, val, rangeStart, rangeEnd, valStart, valEnd):
        valDiff = valEnd - valStart
        if valDiff == 0:
            return 0
        return rangeStart + (rangeEnd - rangeStart) * (val - valStart) / valDiff

    def interpolateVoltage(self, temp, i):
        return self.interpolate(temp, VOLT_TABLE[i - 1], VOLT_TABLE[i], TEMP_TABLE[i - 1], TEMP_TABLE[i])

    def interpolateTemperature(self, microvolts, i):
        return self.interpolate(microvolts, TEMP_TABLE[i - 1], TEMP_TABLE[i], VOLT_TABLE[i - 1], VOLT_TABLE[i])

    def linearSearch(self, t, val):
        for i in range(0, 65):
            if t[i] > val:
                return i
        return 65;

    # Returns the index of the first point whose temperature
    # value is greater than argument
    def searchTemp(self, temp):
        return self.linearSearch(TEMP_TABLE, temp)

    # Returns the index of the first point whose microvolts
    # value is greater than argument
    def searchMicrovolts(self, microvolts):
        return self.linearSearch(VOLT_TABLE, microvolts)

    def get_status(self, eventtime):
        return {
            'temperature': self.temp,
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("ADS11182", ADS11182)
