# KS1 Custom Probe support (utilizingKlipper probe helpers)
#
# Copyright (C) 2025 Antiriad <mail.antiriad@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import probe


class ProbeKS1:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()

        # --- Klipper probe helper integration ---
        # Wrap the endstop pin in ProbeEndstopWrapper (handles MCU setup, deploy/stow, multi-probe)
        self.mcu_endstop = probe.ProbeEndstopWrapper(config)

        # Query and stepper methods
        self.get_mcu       = self.mcu_endstop.get_mcu
        self.add_stepper   = self.mcu_endstop.add_stepper
        self.get_steppers  = self.mcu_endstop.get_steppers
        self.query_endstop = self.mcu_endstop.query_endstop
        # Homing uses wrapper's methods for correct signatures
        self.home_start    = self.mcu_endstop.home_start
        self.home_wait     = self.mcu_endstop.home_wait

        # CS1237 strain gauge sensor
        self.cs1237       = self.printer.lookup_object('cs1237')
        self.threshold    = config.getfloat('sensitivity', self.cs1237.scratch_sensitivity)

        # Override endstop query to include ADC threshold first
        orig_query = self.mcu_endstop.query_endstop

        def custom_query(timestamp):
            adc_val = self.cs1237.adc_value
            if adc_val is not None and adc_val >= self.threshold:
                return 1
            return orig_query(timestamp)
        self.mcu_endstop.query_endstop = custom_query
        self.query_endstop = custom_query

        # Standard Klipper probe helpers
        # (Note: digital probe now stops on ADC threshold via custom query)
        self.cmd_helper    = probe.ProbeCommandHelper(config, self, self.query_endstop)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.param_helper  = probe.ProbeParameterHelper(config)
        self.homing_helper = probe.HomingViaProbeHelper(config, self, self.param_helper)
        self.probe_session = probe.ProbeSessionHelper(
            config, self.param_helper, self.homing_helper.start_probe_session)
        config.get_printer().add_object('probe', self)
        # --- end helper integration ---

        # KS1-specific parameters
        self.speed                = config.getfloat("speed", 5.0)
        self.lift_speed           = config.getfloat("lift_speed", self.speed)
        self.x_offset             = config.getfloat("x_offset", 0.0)
        self.y_offset             = config.getfloat("y_offset", 0.0)
        self.z_offset             = config.getfloat("z_offset", 0.0)
        self.final_speed          = config.getfloat("final_speed", 2.0)
        self.sample_count         = config.getint("samples", 1)
        self.sample_retract_dist  = config.getfloat("sample_retract_dist", 2.0)
        self.samples_tolerance    = config.getfloat("samples_tolerance", 0.100)
        self.samples_retries      = config.getint("samples_tolerance_retries", 0)
        self.samples_result       = config.getchoice(
            "samples_result", ["median", "average", "weighted"], "average")

    # Interface for ProbeCommandHelper
    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self):
        return self.probe_offsets.get_offsets()

    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    # No physical deploy/retract: always at nozzle
    def probe_prepare(self, hmove, *args, **kwargs):
        #logging.warning("ProbeKS1: probe_prepare")
        self.cs1237._enable_cs1237(1)
        self.cs1237._start_reporting()
        self.mcu_endstop.probe_prepare(hmove)

    def probe_finish(self, hmove, *args, **kwargs):
        #logging.warning("ProbeKS1: probe_finish")
        self.cs1237._stop_reporting
        self.cs1237._enable_cs1237(0)
        self.mcu_endstop.probe_finish(hmove)

    def multi_probe_begin(self):
        #logging.warning("ProbeKS1: multi_probe_begin")
        self.mcu_endstop.multi_probe_begin()

    def multi_probe_end(self):
        #logging.warning("ProbeKS1: multi_probe_end")
        self.mcu_endstop.multi_probe_end()

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed)
        return self.lift_speed

    def raise_probe(self):
        logging.warning("ProbeKS1: raise_probe() (no-op)")

    def lower_probe(self):
        logging.warning("ProbeKS1: lower_probe() (no-op)")

    def probe(self, speed):
        logging.warning("ProbeKS1: probe")
        toolhead = self.printer.lookup_object("toolhead")
        curtime = self.printer.get_reactor().monotonic()
        if "z" not in toolhead.get_status(curtime)["homed_axes"]:
            raise self.printer.command_error("Must home before probe")

        pos = list(toolhead.get_position())
        pos[2] = self.z_offset
        phoming = self.printer.lookup_object("homing")
        epos = phoming.probing_move(self.mcu_endstop, pos, speed)

        # Wait for ADC threshold or digital endstop
        #logging.warning(f"[ProbeKS1] cs1237={cs1237}")
        while True:
            adc = self.cs1237.adc_value
            #logging.warning(f"[ProbeKS1] Strain gauge ADC={adc}, threshold={self.threshold}")
            if adc is not None and adc >= self.threshold:
                #logging.warning(f"[ProbeKS1] Probe triggered by ADC: {adc}")
                break
            state = self.query_endstop(self.printer.get_reactor().monotonic())
            if state:
                #logging.warning("[ProbeKS1] Probe triggered (digital)")
                break

        self.last_z_result = epos[2]
        return epos[:3]

    def run_probe(self, gcmd):
        #logging.warning("ProbeKS1: run_probe")
        try:
            initial = self.cs1237.adc_value
            gcmd.respond_info(f"[ProbeKS1] [DEBUG] Initial ADC={initial}")
        except Exception as e:
            gcmd.respond_info(f"[ProbeKS1] [DEBUG] ADC error: {e}")

        params = self.get_probe_params(gcmd)
        speed = params['probe_speed']
        lift_speed = params['lift_speed']
        cnt = params['samples']
        retract = params['sample_retract_dist']
        tol = params['samples_tolerance']
        retries = params['samples_tolerance_retries']
        result_type = params['samples_result']

        self.multi_probe_begin()
        samples = []
        attempts = 0
        toolhead = self.printer.lookup_object("toolhead")
        xy = toolhead.get_position()[:2]

        while len(samples) < cnt:
            try:
                before = self.cs1237.adc_value
                gcmd.respond_info(f"[ProbeKS1] [DEBUG] Before probe #{len(samples)+1} ADC={before}")
            except Exception as e:
                gcmd.respond_info(f"[ProbeKS1] [DEBUG] ADC error: {e}")

            spd = speed if not samples else self.final_speed
            sample = self.probe(spd)

            try:
                after = self.cs1237.adc_value
                gcmd.respond_info(f"[ProbeKS1] [DEBUG] After probe ADC={after}")
            except Exception as e:
                gcmd.respond_info(f"[ProbeKS1] [DEBUG] ADC error: {e}")

            samples.append(sample)
            zs = [s[2] for s in samples]
            if max(zs) - min(zs) > tol:
                if attempts >= retries:
                    raise self.printer.command_error("Probe samples exceed tolerance")
                gcmd.respond_info("[ProbeKS1] Probe samples exceed tolerance. Retrying...")
                attempts += 1
                samples.clear()
                continue

            if len(samples) < cnt:
                self.move([xy[0], xy[1], sample[2] + retract], lift_speed)

        self.multi_probe_end()

        if result_type == 'median':
            return self.calc_median(samples)
        if result_type == 'weighted':
            return self.calc_weighted(samples)
        return self.calc_mean(samples)

    def move(self, coord, speed):
        self.printer.lookup_object("toolhead").manual_move(coord, speed)

    def calc_mean(self, vals):
        c = float(len(vals))
        return [sum(v[i] for v in vals) / c for i in range(3)]

    def calc_weighted(self, vals):
        if len(vals) == 2:
            return [(vals[0][i] * 2 + vals[1][i] * 3) * 0.2 for i in range(3)]
        return self.calc_mean(vals)

    def calc_median(self, vals):
        s = sorted(vals, key=lambda v: v[2])
        mid = len(s) // 2
        if len(s) % 2:
            return s[mid]
        return self.calc_mean(s[mid-1:mid+1])


def load_config(config):
    return ProbeKS1(config)
