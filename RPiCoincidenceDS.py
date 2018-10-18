"""
Created: 2018-10-15

Device server for control of the photocathode gun coincidence trigger generator FPGA
using a raspberry pi.

@author Filip Lindau
"""

import time
import PyTango as pt
from PyTango.server import Device, DeviceMeta
from PyTango.server import attribute
from PyTango.server import device_property
from rpi_coincidence_control import RPiCoincidenceController


class CoincidenceDS(Device):
    __metaclass__ = DeviceMeta

    bucket = attribute(label="Inject bucket",
                       dtype=int,
                       access=pt.AttrWriteType.READ_WRITE,
                       unit="nbr",
                       format="%3d",
                       min_value=0,
                       max_value=255,
                       fget="get_bucket",
                       fset="set_bucket",
                       doc="Target bucket to inject into",
                       memorized=True,
                       hw_memorized=False)

    time_window = attribute(label="Time window",
                            dtype=float,
                            access=pt.AttrWriteType.READ_WRITE,
                            unit="ps",
                            format="%4.1f",
                            min_value=0.0,
                            max_value=2000.0,
                            fget="get_time_window",
                            fset="set_timewindow",
                            doc="Time window for coincidence to trigger",
                            memorized=True,
                            hw_memorized=False)

    laser_trig = attribute(label="Laser trigger source",
                           dtype=str,
                           access=pt.AttrWriteType.READ_WRITE,
                           unit="",
                           format="",
                           fget="get_laser_trig",
                           fset="set_laser_trig",
                           doc="Select laser trigger source MRF or COINCIDENCE",
                           memorized=True,
                           hw_memorized=False)

    data_pins = device_property(dtype=pt.DevVarShortArray,
                                doc="RPi pin numbers of the 8 data bits for FPGA transfer",
                                default_value=[29, 31, 33, 35, 37, 40, 38, 36])

    mode_pin = device_property(dtype=int,
                               doc="RPi pin number of the mode bit for FPGA transfer",
                               default_value=32)

    laser_trig_pin = device_property(dtype=int,
                                     doc="RPi pin number of the laser trig source selector",
                                     default_value=18)

    strobe_pin = device_property(dtype=int,
                                 doc="RPi pin number of the strobe for FPGA transfer",
                                 default_value=22)

    strobe_time = device_property(dtype=int,
                                  doc="Time to assert new data with strobe signal (s)",
                                  default_value=0.005)

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)

        self.controller = RPiCoincidenceController(self.data_pins, self.mode_pin,
                                                   self.strobe_pin, self.strobe_time)

        self.set_state(pt.DevState.ON)

    def get_bucket(self):
        self.debug_stream("In get_bucket:")
        return self.controller.get_bucket(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_bucket(self, new_bucket):
        self.debug_stream("In set_bucket: New bucket " + str(new_bucket))
        self.controller.write_target(new_bucket)

    def get_time_window(self):
        self.debug_stream("In get_time_window:")
        return self.controller.get_delat(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_time_window(self, new_window):
        self.debug_stream("In set_time_window: New time window {0} ps".format(new_window))
        self.controller.set_delay(new_window)

    def get_laser_trig(self):
        self.debug_stream("In get_laser_trig:")
        return self.controller.get_laser_trig(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_laser_trig(self, trig_source):
        self.debug_stream("In set_laser_trig: New trig source {0}".format(trig_source.upper()))
        self.controller.set_laser_trig(trig_source)


if __name__ == "__main__":
    pt.server.server_run((CoincidenceDS,))
