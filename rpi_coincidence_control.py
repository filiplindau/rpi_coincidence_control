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
    def __init__(self, data_pins=None, mode_pin=32, strobe_pin=22, strobe_time=0.005):
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
            self.pin_dict["d0"] = data_pins[0]
            self.pin_dict["d1"] = data_pins[1]
            self.pin_dict["d2"] = data_pins[2]
            self.pin_dict["d3"] = data_pins[3]
            self.pin_dict["d4"] = data_pins[4]
            self.pin_dict["d5"] = data_pins[5]
            self.pin_dict["d6"] = data_pins[6]
            self.pin_dict["d7"] = data_pins[7]

        root.info("Using data pins: {0}".format(self.pin_dict))
        self.pin_dict["mode"] = mode_pin
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
        self.pin_list.append(self.pin_dict["mode"])

        self.strobe_time = strobe_time

        # Generate array of possible delays
        self.delay_bin = np.array([16, 77, 140, 166, 231, 292, 343, 424])
        self.delay_array = list()
        for d in np.arange(256):
            self.delay_array.append((np.unpackbits(np.uint8(d))[::-1] * self.delay_bin).sum())
        self.delay_array = np.array(self.delay_array)

        self.attr_lock = threading.Lock()

        self.target = 0
        self.delay = 0
        self.write_bucket(self.target)
        self.write_delay(self.delay)

    def init_gpio(self):
        root.info("Initializing pins to OUTPUT")
        gpio.setmode(gpio.board)
        for key, value in self.pin_dict.items():
            gpio.setup(value, gpio.OUT)

    def _write_byte(self, data, mode):
        root.debug("Writing byte {0}, mode {1}".format(data, mode))
        # data_shift = data
        # output_list = list()
        # for bit in range(8):
        #     output_list.append(data_shift & 1)
        #     data_shift >>= 1

        output_list = list(np.unpackbits(np.uint8(data))[::-1])
        output_list.append(mode)
        root.debug("Output: {0}".format(output_list))

        gpio.output(self.pin_list, output_list)
        time.sleep(self.strobe_time)
        gpio.output(self.pin_dict["strobe"], 1)
        time.sleep(self.strobe_time)
        gpio.output(self.pin_dict["strobe"], 0)

    def write_bucket(self, target):
        root.info("Selecting bucket {0}".format(target))
        with self.attr_lock:
            self._write_byte(target, 0)
            self.target = target

    def get_bucket(self):
        with self.attr_lock:
            target = self.target
        return target

    def write_delay_raw(self, delay_bin):
        root.info("Writing delay raw number {0}".format(delay_bin))
        with self.attr_lock:
            self._write_byte(delay_bin, 1)

    def write_delay(self, delay):
        ind = np.abs(self.delay_array-delay).argmin()
        delay_min = self.delay_array[ind]
        root.info("Wanted delay {0} ps. Closest possible: {1} ps".format(delay, delay_min))
        with self.attr_lock:
            self._write_byte(ind, 1)
            self.delay = delay

    def get_delay(self):
        with self.attr_lock:
            delay = self.delay
        return delay


if __name__ == "__main__":
    rpc = RPiCoincidenceController(strobe_time=0.1)
