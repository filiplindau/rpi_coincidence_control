"""
Created: 2018-10-12

Control of the photocathode gun coincidence trigger generator FPGA
using a raspberry pi.

@author Filip Lindau
"""

import RPi.GPIO as gpio
import time
import numpy as np
import threading
import logging

root = logging.getLogger()
while len(root.handlers):
    root.removeHandler(root.handlers[0])

f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
root.addHandler(fh)
root.setLevel(logging.DEBUG)


class RPiCoincidenceController(object):
    def __init__(self, data_pins=None, mode_pins=[32, 22, 18, 16], strobe_pin=12, strobe_time=0.005):
        self.pin_dict = dict()
        if data_pins is None:
            self.pin_dict["d0"] = 29
            self.pin_dict["d1"] = 31
            self.pin_dict["d2"] = 33
            self.pin_dict["d3"] = 35
            self.pin_dict["d4"] = 37
            self.pin_dict["d5"] = 40
            self.pin_dict["d6"] = 38
            self.pin_dict["d7"] = 36
        else:
            self.pin_dict["d0"] = int(data_pins[0])
            self.pin_dict["d1"] = int(data_pins[1])
            self.pin_dict["d2"] = int(data_pins[2])
            self.pin_dict["d3"] = int(data_pins[3])
            self.pin_dict["d4"] = int(data_pins[4])
            self.pin_dict["d5"] = int(data_pins[5])
            self.pin_dict["d6"] = int(data_pins[6])
            self.pin_dict["d7"] = int(data_pins[7])

        root.info("Using data pins: {0}".format(self.pin_dict))
        self.pin_dict["mode0"] = mode_pins[0]
        self.pin_dict["mode1"] = mode_pins[1]
        self.pin_dict["mode2"] = mode_pins[2]
        self.pin_dict["mode3"] = mode_pins[3]
        self.pin_dict["strobe"] = strobe_pin

        self.pin_list = list()
        self.pin_list.append(self.pin_dict["d0"])
        self.pin_list.append(self.pin_dict["d1"])
        self.pin_list.append(self.pin_dict["d2"])
        self.pin_list.append(self.pin_dict["d3"])
        self.pin_list.append(self.pin_dict["d4"])
        self.pin_list.append(self.pin_dict["d5"])
        self.pin_list.append(self.pin_dict["d6"])
        self.pin_list.append(self.pin_dict["d7"])
        self.pin_list.append(self.pin_dict["mode0"])
        self.pin_list.append(self.pin_dict["mode1"])
        self.pin_list.append(self.pin_dict["mode2"])
        self.pin_list.append(self.pin_dict["mode3"])

        self.mode_dict = dict()
        self.mode_dict["inc_phase"] = 1
        self.mode_dict["dec_phase"] = 3
        self.mode_dict["quadrature"] = 4
        self.mode_dict["pause_trig"] = 5
        self.mode_dict["bucket"] = 6
        self.mode_dict["window"] = 7
        self.mode_dict["laser_trig_source"] = 8
        self.mode_dict["ring_rf_source"] = 9

        self.strobe_time = strobe_time
        self.laser_freq = 2998.5e6 / 39
        self.quad_time = 1 / self.laser_freq / 4

        # Generate array of possible delays
        self.delay_bin = np.array([16, 77, 140, 166, 231, 292, 343, 424])
        self.delay_array = list()
        for d in np.arange(256):
            self.delay_array.append((np.unpackbits(np.uint8(d))[::-1] * self.delay_bin).sum())
        self.delay_array = np.array(self.delay_array)

        self.init_gpio()

        self.attr_lock = threading.Lock()

        self.target = 0
        self.window = 0
        self.laser_trig = 0
        self.offset = 0
        self.set_bucket(self.target)
        self.set_window(self.window)
        self.set_offset(self.offset)

    def init_gpio(self):
        root.info("Initializing pins to OUTPUT")
        gpio.setmode(gpio.BOARD)
        for key, value in self.pin_dict.items():
            gpio.setup(value, gpio.OUT)

    def _write_byte(self, data, mode):
        root.debug("Writing byte {0}, mode {1}".format(data, mode))
        # data_shift = data
        # output_list = list()
        # for bit in range(8):
        #     output_list.append(data_shift & 1)
        #     data_shift >>= 1

        data_list = list(np.unpackbits(np.uint8(data))[::-1].astype(int))
        mode_list = list(np.unpackbits(np.uint8(mode))[::-1].astype(int))[0:4]
        output_list = data_list + mode_list
        root.debug("Output: {0}".format(output_list))
        root.debug("Pins: {0}".format(self.pin_list))

        gpio.output(self.pin_list, output_list)
        time.sleep(self.strobe_time)
        gpio.output(self.pin_dict["strobe"], 1)
        time.sleep(self.strobe_time)
        gpio.output(self.pin_dict["strobe"], 0)

    def set_bucket(self, target):
        root.info("Selecting bucket {0}".format(target))
        with self.attr_lock:
            self._write_byte(target, self.mode_dict["bucket"])
            self.target = target

    def get_bucket(self):
        with self.attr_lock:
            target = self.target
        return target

    def write_window_raw(self, delay_bin):
        root.info("Writing delay raw number {0}".format(delay_bin))
        with self.attr_lock:
            self._write_byte(delay_bin, self.mode_dict["delay"])

    def set_window(self, delay):
        """
        Set time delay that the coincidence window is open. Only some values are possible,
        this will set the closest valid value by combining these:
        [16, 77, 140, 166, 231, 292, 343, 424] ps
        :param delay: Delay window in ps
        :return:
        """
        ind = np.abs(self.delay_array-delay).argmin()
        delay_min = self.delay_array[ind]
        root.info("Wanted delay {0} ps. Closest possible: {1} ps".format(delay, delay_min))
        with self.attr_lock:
            self._write_byte(ind, self.mode_dict["window"])
            self.window = delay_min

    def get_window(self):
        with self.attr_lock:
            window = self.window
        return window

    def set_laser_trig(self, source="MRF"):
        """
        Set the source of the laser 20 Hz trig. It can be either MRF for normal triggering
        or COINCIDENCE for coincidence triggering.

        :param source: String MRF or COINCIDENCE
        :return:
        """
        with self.attr_lock:
            if "coin" in source.lower():
                root.info("Using COINCIDENCE triggering")
                self.laser_trig = 1
            else:
                # MRF triggering
                root.info("Using MRF triggering")
                self.laser_trig = 0
            self._write_byte(self.laser_trig, self.mode_dict["laser_trig_source"])

    def get_laser_trig(self):
        with self.attr_lock:
            trig = self.laser_trig
        if trig == 0:
            return "MRF"
        else:
            return "COINCIDENCE"

    def set_offset(self, offset):
        root.info("Setting offset {0}".format(offset))
        with self.attr_lock:
            quad_offset = np.uint8(offset // self.quad_time)
            phase_offset = np.uint8((offset - quad_offset * self.quad_time) * self.laser_freq)
            self._write_byte(phase_offset, self.mode_dict["phase_offset"])
            self._write_byte(quad_offset, self.mode_dict["quadrature"])
            self.offset = quad_offset * self.quad_time + phase_offset / self.laser_freq

    def get_offset(self):
        with self.attr_lock:
            offset = self.offset
        return offset


if __name__ == "__main__":
    rpc = RPiCoincidenceController(strobe_time=0.1)
