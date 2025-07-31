# CS1237 support 
#
# Copyright (C) 2025 Antiriad <mail.antiriad@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# State and error constants
SELF_CHECK_STATE      = 1 << 0
SCRATCH_STATE         = 1 << 1
BLOCK_FILAMENT_STATE  = 1 << 2
HEAD_BLOCK_STATE      = 1 << 3
SELF_CHECK_ERR        = SELF_CHECK_STATE      | 0x80
SCRATCH_STATE_ERR     = SCRATCH_STATE         | 0x80
BLOCK_FILAMENT_ERR    = BLOCK_FILAMENT_STATE  | 0x80
HEAD_BLOCK_STATE_ERR  = HEAD_BLOCK_STATE      | 0x80


class CS1237:
    head_block_sensitivity     = 0
    scratch_sensitivity        = 0
    self_check_sensitivity     = 0
    block_filament_sensitivity = 0

    def __init__(self, config):
        # Core references
        self.printer = config.get_printer()
        self.mcu = config.get_printer().lookup_object('mcu nozzle_mcu')
        ppins       = self.printer.lookup_object('pins')

        # Configuration parameters
        self.level_pin_name      = config.get('level_pin', None)
        self.dout_pin_name       = config.get('dout_pin')
        self.sclk_pin_name       = config.get('sclk_pin')
        self.register_address    = config.getint('register', default=60)
        self.sensitivity         = config.getint('sensitivity', default=-2500)
        self.samples_count       = config.getint('samples_count', default=10)
        self.data_bit            = config.getint('data_bit', default=24)
        self._samples_per_second = config.getint('samples_per_second', default=10)
        self._range              = (-0x800000, 0x7FFFFF)

        # Threshold sensitivities
        self.head_block_sensitivity     = config.getint('head_block_sensitivity', default=-300000)
        self.scratch_sensitivity        = config.getint('scratch_sensitivity', default=-100000)
        self.self_check_sensitivity     = config.getint('self_check_sensitivity', default=-400)
        self.block_filament_sensitivity = config.getint('block_filament_sensitivity', default=-3000)

        self._clients        = []
        self._reporting      = False
        self._query_complete = None

        # Resolve pin numbers
        def _get_pin(name):
            return ppins.lookup_pin(name, True, True, None)['pin']

        self.level_pin = _get_pin(self.level_pin_name) if self.level_pin_name else 0
        self.dout_pin  = _get_pin(self.dout_pin_name)
        self.sclk_pin  = _get_pin(self.sclk_pin_name)

        # State variables
        self.adc_value       = None
        self.raw_value       = None
        self.sensor_state    = None
        self.checkself_flag  = None
        self._clients        = []
        self._reporting      = False
        self._query_complete = None

        # Register config callback for MCU command setup
        self.mcu.register_config_callback(self._build_config)

        # Register G-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('CS1237',       self.cmd_dump, when_not_ready=False, desc='')
        gcode.register_command('G9121',       self.cmd_g9121, False, '')
        gcode.register_command('G9122',       self.cmd_g9122, False, '')
        gcode.register_command('G9123',       self.cmd_g9123, False, '')

        # Dedicated MCU test commands
        gcode.register_command('CS1237_CHECKSELF', self.cmd_checkself_cs1237, False, 'Trigger checkself_cs1237 MCU command')
        gcode.register_command('CS1237_CONFIG', self.cmd_config_cs1237, False, 'Trigger config_cs1237 MCU command')
        gcode.register_command('CS1237_RESET', self.cmd_reset_cs1237, False, 'Trigger reset_cs1237 MCU command')
        gcode.register_command('CS1237_REPORT', self.cmd_start_cs1237_report, False, 'Trigger start_cs1237_report MCU command')
        gcode.register_command('CS1237_DIFF', self.cmd_query_cs1237_diff, False, 'Trigger query_cs1237_diff MCU command')
        gcode.register_command('CS1237_ENABLE', self.cmd_enable_cs1237, False, 'Trigger enable_cs1237 MCU command')
        
    def cmd_enable_cs1237(self, gcmd):
        state = gcmd.get_int('STATE', 1, minval=0, maxval=1)
        enable_cmd = self.mcu.lookup_command("enable_cs1237 oid=%c state=%c")
        enable_cmd.send([self._oid, state])
        #logging.warning(f"[CS1237] Sent enable_cs1237 with STATE={state}")
    
    def _enable_cs1237(self, state=1):
        enable_cmd = self.mcu.lookup_command("enable_cs1237 oid=%c state=%c")
        enable_cmd.send([self._oid, state])
        #logging.warning(f"[CS1237] enable_cs1237 called automatically from script with STATE={state}")
    
    # Dedicated G-code command handlers for MCU commands
    def cmd_checkself_cs1237(self, gcmd):
        w = gcmd.get_int('W', 0, minval=0, maxval=3)
        self.cmd_checkself.send([
            self._oid,
            w
        ])
        #logging.warning(f"[CS1237] Sent checkself_cs1237 with W={w}")

    def cmd_config_cs1237(self, gcmd):
        cfg = (
            f"config_cs1237 oid={self._oid} "
            f"level_pin={self.level_pin} dout_pin={self.dout_pin} sclk_pin={self.sclk_pin} "
            f"register={self.register_address} sensitivity={self.sensitivity}"
        )
        self.mcu.add_config_cmd(cfg)
        #logging.warning(f"[CS1237] Sent config_cs1237: {cfg}")

    def cmd_reset_cs1237(self, gcmd):
        count = gcmd.get_int('COUNT', 3, minval=1, maxval=10)
        reset_cmd = self.mcu.lookup_command("reset_cs1237 oid=%c count=%c")
        reset_cmd.send([
            self._oid,
            count
        ])
        #logging.warning(f"[CS1237] Sent reset_cs1237 with COUNT={count}")

    def cmd_start_cs1237_report(self, gcmd):
        ticks = gcmd.get_int('TICKS', self.mcu.seconds_to_clock(1.0 / self._samples_per_second))
        print_state = gcmd.get_int('STATE', 0, minval=0, maxval=3)
        sensitivity = gcmd.get_int('SENS', self.sensitivity)
        self.cmd_start_report.send([
            self._oid,
            1,
            ticks,
            print_state,
            sensitivity
        ])
        #logging.warning(f"[CS1237] Sent start_cs1237_report ENABLE={enable} TICKS={ticks} STATE={print_state} SENS={sensitivity}")

    def cmd_query_cs1237_diff(self, gcmd):
        diff_cmd = self.mcu.lookup_command("query_cs1237_diff oid=%c")
        diff_cmd.send([
            self._oid
        ])
        #logging.warning(f"[CS1237] Sent query_cs1237_diff")
        self.printer.register_event_handler('cs1237:self_check', self._self_check_sequence)

    def _build_config(self):
        self._oid = self.mcu.create_oid()
        cfg = (
            f"config_cs1237 oid={self._oid} "
            f"level_pin={self.level_pin} dout_pin={self.dout_pin} sclk_pin={self.sclk_pin} "
            f"register={self.register_address} sensitivity={self.sensitivity}"
        )
        #logging.warning(f"[CS1237 DEBUG] config command: {cfg}")

        self.mcu.add_config_cmd(cfg)
        self.cmd_start_report = self.mcu.lookup_command(
            "start_cs1237_report oid=%c enable=%c ticks=%i print_state=%c sensitivity=%i"
        )
        self.cmd_checkself = self.mcu.lookup_command(
            "checkself_cs1237 oid=%c write=%c"
        )
        self.mcu.register_response(self._handle_cs1237_report,  'cs1237_state', self._oid)
        self.mcu.register_response(self._handle_cs1237_diff,    'cs1237_diff', self._oid)
        self.mcu.register_response(self._handle_cs1237_check,   'cs1237_checkself_flag', self._oid)

    # ---- New Response Handlers ----
    def _handle_start_report_ack(self, params):
        logging.warning(f"[CS1237] start_cs1237_report ACK received: {params}")

    def _handle_checkself_ack(self, params):
        logging.warning(f"[CS1237] checkself_cs1237 ACK received: {params}")

    # ---- MCU Response Handlers ----
    def _handle_cs1237_report(self, params):
        #logging.warning(f"[cs1237 REPORT] params={params}")
        try:
            self.adc_value    = int(params.get('adc', 0))
            self.raw_value    = int(params.get('raw', 0))
            self.sensor_state = int(params.get('state', 0))
            #logging.warning(f"[cs1237 REPORT] adc={self.adc_value} raw={self.raw_value} state={self.sensor_state}")

            msg = {
                'timestamp': self.printer.get_reactor().monotonic(),
                'adc': self.adc_value,
                'raw': self.raw_value,
                'state': self.sensor_state
            }
            for cb in list(self._clients):
                if cb(msg) is False:
                    self._clients.remove(cb)
        except Exception:
            logging.exception("CS1237: report handler error")

    def _handle_cs1237_diff(self, params):
        try:
            self.raw_value = int(params.get('raw', 0))
            if self._query_complete:
                self._query_complete.set_result(params)
        except Exception:
            logging.exception("CS1237: diff handler error")

    def _handle_cs1237_check(self, params):
        try:
            self.checkself_flag = int(params.get('flag', 0))
            if self._query_complete:
                self._query_complete.set_result(params)
        except Exception:
            logging.exception("CS1237: self-check handler error")

    # ---- Status ----
    def get_status(self, eventtime=None):
        return {
            'adc': self.adc_value,
            'raw': self.raw_value,
            'state': self.sensor_state,
            'checkself_flag': self.checkself_flag
        }

    # ---- Reporting Control ----
    def _start_reporting(self):
        if self._reporting:
            return
        self._reporting = True
        ticks = self.mcu.seconds_to_clock(1.0 / self._samples_per_second)
        self.cmd_start_report.send([
            self._oid,
            1,
            ticks,
            0,
            self.sensitivity
        ])
        logging.warning(f">>> CS1237: _start_reporting (alloc_command)")

    def _stop_reporting(self):
        if not self._reporting:
            return
        self._reporting = False
        self.cmd_start_report.send([
            self._oid,
            0,
            0,
            0,
            self.sensitivity
        ])
        logging.warning(f">>> CS1237: _stop_reporting (alloc_command)")

    def add_client(self, callback):
        if callback not in self._clients:
            self._clients.append(callback)
        self._start_reporting()

    def remove_client(self, callback):
        if callback in self._clients:
            self._clients.remove(callback)
        if not self._clients:
            self._stop_reporting()

    # ---- G-Code Commands ----
    def cmd_dump(self, gcmd):
        logging.warning(f">>> CS1237: cmd_dump called")
        e = gcmd.get_int('E', 0, minval=0, maxval=1)
        t = gcmd.get_float('T', 1.0, minval=0.1, maxval=1.0)
        s = gcmd.get_int('S', SELF_CHECK_STATE, minval=SELF_CHECK_STATE, maxval=HEAD_BLOCK_STATE)
        # map thresholds
        sensitivity_map = {
            SELF_CHECK_STATE:     self.self_check_sensitivity,
            SCRATCH_STATE:        self.scratch_sensitivity,
            BLOCK_FILAMENT_STATE: self.block_filament_sensitivity,
            HEAD_BLOCK_STATE:     self.head_block_sensitivity
        }
        sens = sensitivity_map.get(s, self.sensitivity)

        if e:
           # Tell the host proxy to start reports
            self._enable_cs1237(1)
            self._start_reporting()
           # Then instruct MCU to report cs1237_state every T seconds
            self._check_start(t, s, sens)
        else:
           # First tell MCU to stop cs1237_state messages
            self._check_stop(s)
           # Then tell the host proxy to stop reporting
            self._stop_reporting()
            self._enable_cs1237(0)

    def cmd_g9121(self, gcmd):
        e = gcmd.get_int('E', 0, minval=0, maxval=1)
        if e:
            self._check_start(
                gcmd.get_float('T', 1.0, minval=0.1, maxval=1.0),
                SCRATCH_STATE,
                self.scratch_sensitivity
            )
        else:
            self._check_stop(SCRATCH_STATE)

    def cmd_g9122(self, gcmd):
        e = gcmd.get_int('E', 0, minval=0, maxval=1)
        if e:
            self._check_start(
                gcmd.get_float('T', 1.0, minval=0.1, maxval=1.0),
                SELF_CHECK_STATE,
                self.self_check_sensitivity
            )
        else:
            self._check_stop(SELF_CHECK_STATE)

    def cmd_g9123(self, gcmd):
        w = gcmd.get_int('W', 0, minval=0, maxval=3)
        self._query_complete = self.printer.get_reactor().Completion()
        self.cmd_checkself.send([
            self._oid,
            w
        ])
        params = self._query_complete.Wait(
            self.printer.get_reactor().monotonic() + 2, None
        )
        if params:
            self.checkself_flag = int(params.get('flag', 0))

    # ---- Internal Helpers ----
    def _check_start(self, period, ctype, sens):
        ticks = self.mcu.seconds_to_clock(period)
        self.cmd_start_report.send([
            self._oid,
            1,
            ticks,
            0,
            sens
        ])

    def _check_stop(self, ctype):
        self.cmd_start_report.send([
            self._oid,
            0,
            0,
            0,
            0
        ])

    # ---- Startup Self-Check ----
    def _self_check_sequence(self, _args):
        self._check_start(1.0, SELF_CHECK_STATE, self.self_check_sensitivity)
        self.printer.send_event('cs1237:test_resonances', None)
        self.printer.get_reactor().Pause(
            self.printer.get_reactor().monotonic() + 3.0
        )
        self._check_stop(SELF_CHECK_STATE)
        if self.sensor_state == SELF_CHECK_ERR:
            raise Exception("CS1237 boot up self-check failed")


def load_config(config):
    return CS1237(config)
