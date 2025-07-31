# Support for reading acceleration data from an LIS2DW12 chip
#
# Copyright (C) 2025 Antiriad <mail.antiriad@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import bus, bulk_sensor, adxl345

# LIS2DW12 registers and constants
REG_DEVID = 0x0F
REG_CTRL1 = 0x20
REG_CTRL6 = 0x25
REG_FIFO_CTRL = 0x2E
REG_MOD_READ = 0x80

LIS2DW12_DEV_ID = 0x44

LIS2DW12_QUERY_RATES = {
    25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400: 0x7, 800: 0x8, 1600: 0x9,
}

FREEFALL_ACCEL = 9.80665 * 1000.
SCALE_XY = 0.000244140625 * FREEFALL_ACCEL  # 1/4096 * g * 1000 (mg/LSB)
SCALE_Z = 0.000244140625 * FREEFALL_ACCEL

BATCH_UPDATES = 0.100

class LIS2DW12:
    #TODO: implement real FFReader
    class DummyFFReader:
        def note_start(self): pass
        def note_end(self): pass

    class _InternalClient:
        def __init__(self, parent):
            self.parent = parent
            self.active = False
        def start(self):
            if not self.active:
                self.parent._start_measurements()
                self.active = True
        def stop(self):
            if self.active:
                self.parent._finish_measurements()
                self.active = False
        def get_status(self):
            return self.parent.get_status()
        def finish_measurements(self):
            self.stop()

    def start_internal_client(self):
        """
        Returns an internal client object for Klipper sensor interface compatibility.
        """
        client = self._InternalClient(self)
        client.start()
        return client
    def __init__(self, config):
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        # Axes map
        am = {
            "x": (0, SCALE_XY), "y": (1, SCALE_XY), "z": (2, SCALE_Z),
            "-x": (0, -SCALE_XY), "-y": (1, -SCALE_XY), "-z": (2, -SCALE_Z),
        }
        axes_map = config.getlist("axes_map", ["x", "y", "z"], ',', count=3)
        self.axes_map = [am[str(a).strip()] for a in axes_map]
        self.data_rate = config.getint("rate", 1600)
        if self.data_rate not in LIS2DW12_QUERY_RATES:
            raise config.error("Invalid lis2dw12 rate parameter")
        # Setup SPI bus
        self.bus = bus.MCU_SPI_from_config(config, 3, default_speed=5000000)
        self.mcu = self.bus.get_mcu()
        self.oid = self.mcu.create_oid()
        logging.info("LIS2DW12 mcu=%s oid=%d", self.mcu.get_name(), self.oid)
        self.query_sensor_cmd = None
        self.query_sensor_end_cmd = None
        self.query_sensor_status_cmd = None
        self.last_sequence = 0
        self.max_query_duration = 0
        self.last_limit_count = 0
        self.last_error_count = 0
        self.name = config.get_name().split()[-1]
        # Register config commands
        self.mcu.add_config_cmd(
            "config_lis2dw12 oid=%d spi_oid=%d" % (self.oid, self.bus.get_oid()))
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.mcu.add_config_cmd(
            "query_lis2dw12 oid=%d clock=0 rest_ticks=%d" % (self.oid, rest_ticks), on_restart=True)
        self.mcu.register_config_callback(self._build_config)
        # Register MCU response handlers for LIS2DW12 data and status
        self.mcu.register_response(self._handle_lis2dw12_data, 'lis2dw12_data', self.oid)
        self.mcu.register_response(self._handle_lis2dw12_status, 'lis2dw12_status', self.oid)
        self.samples = []
        self.ffreader = self.DummyFFReader()
        # Register G-code commands for manual triggering
        gcode = self.printer.lookup_object('gcode')
        section_name = config.get_name().split()[-1].upper().replace("-", "_")
        gcode.register_command(f"DWTWELFE_READ_{section_name}", self.cmd_lis2dw12_read, desc=f"Trigger LIS2DW12 data read ({section_name})")
        gcode.register_command(f"DWTWELVE_STATUS_{section_name}", self.cmd_lis2dw12_status, False, f"Query LIS2DW12 status ({section_name})")

    def cmd_lis2dw12_read(self, gcmd):
        clock = gcmd.get_int('CLOCK', 0)
        rest_ticks = gcmd.get_int('REST_TICKS', self.mcu.seconds_to_clock(4. / self.data_rate))
        self.query_lis2dw12(clock, rest_ticks)
        gcmd.respond_info("LIS2DW12_READ triggered")

    def cmd_lis2dw12_status(self, gcmd):
        self.query_lis2dw12_status()
        gcmd.respond_info("LIS2DW12_STATUS triggered")

    def _build_config(self):
        cmdqueue = self.bus.get_command_queue()
        self.query_sensor_cmd = self.mcu.lookup_command(
            "query_lis2dw12 oid=%c clock=%u rest_ticks=%u", cq=cmdqueue)
        self.query_sensor_status_cmd = self.mcu.lookup_command(
            "query_lis2dw12_status oid=%c", cq=cmdqueue)

    def query_lis2dw12(self, clock=0, rest_ticks=0):
        self.query_sensor_cmd.send([self.oid, clock, rest_ticks])
        logging.info("Sent query_lis2dw12 command")

    def query_lis2dw12_status(self):
        self.query_sensor_status_cmd.send([self.oid])
        logging.info("Sent query_lis2dw12_status command")

    def _handle_lis2dw12_data(self, params):
        raw_bytes = params.get('data', b'')
        sequence = params.get('sequence', 0)
        # GoKlipper algorithm: extract samples from packed bytes
        # Each sample: 6 bytes (xlow, ylow, zlow, xhigh, yhigh, zhigh)
        samples = []
        for i in range(0, len(raw_bytes), 6):
            if i + 6 > len(raw_bytes):
                break
            xlow = raw_bytes[i]
            ylow = raw_bytes[i+1]
            zlow = raw_bytes[i+2]
            xhigh = raw_bytes[i+3]
            yhigh = raw_bytes[i+4]
            zhigh = raw_bytes[i+5]
            rx = ((xhigh << 8) | xlow) & 0xFFFF
            ry = ((yhigh << 8) | ylow) & 0xFFFF
            rz = ((zhigh << 8) | zlow) & 0xFFFF
            # Convert to signed 16-bit
            rx = rx if rx < 0x8000 else rx - 0x10000
            ry = ry if ry < 0x8000 else ry - 0x10000
            rz = rz if rz < 0x8000 else rz - 0x10000
            samples.append((rx, ry, rz))
        self.samples.extend(samples)
        logging.info(f"LIS2DW12 sequence={sequence} samples={samples}")

    def _handle_lis2dw12_status(self, params):
        logging.info(f"LIS2DW12 status: {params}")

    def read_reg(self, reg):
        params = self.bus.spi_transfer([reg | REG_MOD_READ, 0x00])
        response = bytearray(params['response'])
        return response[1]

    def set_reg(self, reg, val, minclock=0):
        self.bus.spi_send([reg, val & 0xFF], minclock=minclock)
        stored_val = self.read_reg(reg)
        if stored_val != val:
            raise self.printer.command_error(
                "Failed to set LIS2DW12 register [0x%x] to 0x%x: got 0x%x. "
                "This is generally indicative of connection problems "
                "(e.g. faulty wiring) or a faulty lis2dw12 chip." % (
                    reg, val, stored_val))

    def _start_measurements(self):
        # Check device ID
        dev_id = self.read_reg(REG_DEVID)
        if dev_id != LIS2DW12_DEV_ID:
            raise self.printer.command_error(
                "Invalid lis2dw12 id (got %x vs %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty lis2dw12 chip."
                % (dev_id, LIS2DW12_DEV_ID))
        # Setup chip in requested query rate
        self.set_reg(REG_CTRL1, LIS2DW12_QUERY_RATES[self.data_rate] << 4 | 0x04)
        self.set_reg(REG_FIFO_CTRL, 0x00)
        self.set_reg(REG_CTRL6, 0x04)
        self.set_reg(REG_FIFO_CTRL, 0xC0)
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.query_sensor_cmd.send([self.oid, 0, rest_ticks])
        logging.info("LIS2DW12 starting '%s' measurements", self.name)
        self.ffreader.note_start()
        self.last_error_count = 0

    def _finish_measurements(self):
        # Halt bulk reading
        self.set_reg(REG_FIFO_CTRL, 0x00)
        self.query_sensor_cmd.send_wait_ack([self.oid, 0, 0])
        self.ffreader.note_end()
        logging.info("LIS2DW12 finished '%s' measurements", self.name)
        self.set_reg(REG_FIFO_CTRL, 0x00)


def load_config(config):
    return LIS2DW12(config)

def load_config_prefix(config):
    return LIS2DW12(config)
