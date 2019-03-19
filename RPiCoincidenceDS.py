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
                       hw_memorized=True)

    time_window = attribute(label="Time window",
                            dtype=float,
                            access=pt.AttrWriteType.READ_WRITE,
                            unit="ps",
                            format="%4.1f",
                            min_value=0.0,
                            max_value=2000.0,
                            fget="get_timewindow",
                            fset="set_timewindow",
                            doc="Time window for coincidence to trigger",
                            memorized=True,
                            hw_memorized=True)

    delay_fine_offset = attribute(label="Delay fine offset",
                                  dtype=int,
                                  access=pt.AttrWriteType.READ_WRITE,
                                  unit="taps",
                                  format="%3d",
                                  min_value=0,
                                  max_value=255,
                                  fget="get_offset_fine",
                                  fset="set_offset_fine",
                                  doc="Delay time offset for coincidence generation, "
                                      "set for ring bucket fill rate optimization",
                                  memorized=True,
                                  hw_memorized=True)

    delay_coarse_offset = attribute(label="Delay 2.5 ns steps",
                                    dtype=int,
                                    access=pt.AttrWriteType.READ_WRITE,
                                    unit="2.5 ns steps",
                                    format="%3d",
                                    min_value=0,
                                    max_value=255,
                                    fget="get_offset_coarse",
                                    fset="set_offset_coarse",
                                    doc="Delay time offset for coincidence generation, "
                                        "set for ring bucket fill rate optimization. Each step is 2.5 ns.",
                                    memorized=True,
                                    hw_memorized=True)

    laser_trig = attribute(label="Laser trigger source",
                           dtype=str,
                           access=pt.AttrWriteType.READ_WRITE,
                           unit="",
                           format="",
                           fget="get_laser_trig",
                           fset="set_laser_trig",
                           doc="Select laser trigger source MRF or COINCIDENCE",
                           memorized=True,
                           hw_memorized=True)

    ring_rf = attribute(label="Ring RF source",
                        dtype=str,
                        access=pt.AttrWriteType.READ_WRITE,
                        unit="",
                        format="",
                        fget="get_ring_rf",
                        fset="set_ring_rf",
                        doc="Select ring RF source REV_CLOCK or 100MHZ",
                        memorized=True,
                        hw_memorized=True)

    data_pins = device_property(dtype=pt.DevVarShortArray,
                                doc="RPi pin numbers of the 8 data bits for FPGA transfer",
                                default_value=[29, 31, 33, 35, 37, 40, 38, 36])

    mode_pins = device_property(dtype=pt.DevVarShortArray,
                                doc="RPi pin numbers of the mode bits for FPGA transfer",
                                default_value=[32, 22, 18, 16])

    strobe_pin = device_property(dtype=int,
                                 doc="RPi pin number of the strobe for FPGA transfer",
                                 default_value=12)

    strobe_time = device_property(dtype=float,
                                  doc="Time to assert new data with strobe signal (s)",
                                  default_value=0.005)

    delay_step = device_property(dtype=float,
                                 doc="Delay offset time step size (ps)",
                                 default_value=23.0)

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)

        self.debug_stream("data_pins: {0}".format(self.data_pins))

        self.controller = RPiCoincidenceController(data_pins=self.data_pins,
                                                   mode_pins=self.mode_pins,
                                                   strobe_pin=self.strobe_pin,
                                                   strobe_time=self.strobe_time)
        self.controller.set_avg_phase_advance(self.delay_step * 1e-12)
        self.set_state(pt.DevState.ON)

    def get_bucket(self):
        self.debug_stream("In get_bucket:")
        return self.controller.get_bucket(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_bucket(self, new_bucket):
        self.debug_stream("In set_bucket: New bucket " + str(new_bucket))
        try:
            self.controller.set_bucket(new_bucket)
        except ValueError:
            raise

    def get_timewindow(self):
        self.debug_stream("In get_time_window:")
        return self.controller.get_window(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_timewindow(self, new_window):
        self.debug_stream("In set_time_window: New time window {0} ps".format(new_window))
        self.controller.set_window(new_window)

    def get_laser_trig(self):
        self.debug_stream("In get_laser_trig:")
        return self.controller.get_laser_trig(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_laser_trig(self, trig_source):
        self.debug_stream("In set_laser_trig: New trig source {0}".format(trig_source.upper()))
        self.controller.set_laser_trig(trig_source)

    def get_ring_rf(self):
        self.debug_stream("In get_ring_rf:")
        return self.controller.get_ring_rf_source(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_ring_rf(self, new_ring_rf):
        self.debug_stream("In set_ring_rf: New RF {0}".format(new_ring_rf.upper()))
        self.controller.set_ring_rf_source(new_ring_rf)

    def get_offset_fine(self):
        self.debug_stream("In get_offset_fine:")
        return self.controller.get_offset_fine(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_offset_fine(self, new_offset):
        self.debug_stream("In set_offset_fine: New offset {0} taps".format(new_offset))
        self.controller.set_offset_fine(new_offset)

    def get_offset_coarse(self):
        self.debug_stream("In get_offset_coarse:")
        return self.controller.get_offset_coarse(), time.time(), pt.AttrQuality.ATTR_VALID

    def set_offset_coarse(self, new_offset):
        self.debug_stream("In set_offset_coarse: New offset {0} x 2.5 ns".format(new_offset))
        self.controller.set_offset_coarse(new_offset)


if __name__ == "__main__":
    pt.server.server_run((CoincidenceDS,))
