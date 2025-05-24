# uavcontrol.py
# Synchronous flight-control module (no async/await).
# Restores missing get_active_modes().

import threading
import argparse
import json
import traceback
import logging
import struct
import time
import sys
import serial
from collections import deque
from itertools import cycle
from select import select
from threading import Lock, RLock
from statistics import mean
from simple_pid import PID

from unavlib.enums import inav_enums
from unavlib.enums.msp_codes import MSPCodes
from unavlib.enums import msp_vars
from unavlib.modules.utils import dict_index
from unavlib.modules.utils import dict_reverse
from unavlib.modules.utils import TCPSocket
from unavlib.modules.utils import inavutil
from unavlib.modules.process import processMSP
from unavlib.modules.fast_functions import fastMSP
from unavlib.modules import geospatial
from unavlib import MSPy


class UAVControl:
    def __init__(self, device, baudrate, use_tcp=False, platform="AIRPLANE"):
        # ----- INITIAL SET-UP -----------------------------------------
        self.debugprint = False
        self.mspy_min_time_between_writes = 1 / 1000
        self.mspy_loglevel = "INFO"
        self.mspy_timeout = 1 / 100
        self.mspy_logfilename = "MSPy.log"
        self.mspy_logfilemode = "a"

        self.board = MSPy(
            device=device,
            loglevel=self.mspy_loglevel,
            use_tcp=use_tcp,
            baudrate=baudrate,
            min_time_between_writes=self.mspy_min_time_between_writes,
        )

        self.run = False
        self.connected = None
        self.device = device
        self.baudrate = baudrate
        self.chorder = [
            "roll",
            "pitch",
            "throttle",
            "yaw",
            "ch5",
            "ch6",
            "ch7",
            "ch8",
            "ch9",
            "ch10",
            "ch11",
            "ch12",
            "ch13",
            "ch14",
            "ch15",
            "ch16",
            "ch17",
            "ch18",
        ]
        self.channels = [900] * 16
        self.pwm_range = [900, 2100]
        self.pwm_midpoint = mean(self.pwm_range)
        self.channels[0] = 1500
        self.channels[1] = 1500
        self.channels[3] = 1500
        self.modes = {}
        self.active_modes = []          # <— updated by caller if desired
        self.supermodes = {}
        self.active_supermodes = []
        self.pids = {}
        self.msp_receiver = False
        self.msp_override_active = False
        self.msp_override_allowed_ch = []
        self.msp_override_channels = {}
        self.rc_interval = 20           # Hz
        self.telemetry_data_init = False

        # ----- MSP MESSAGE LISTS --------------------------------------
        self.init_msp_msgs = [
            inavutil.msp.MSP_API_VERSION,
            inavutil.msp.MSP_FC_VARIANT,
            inavutil.msp.MSP_FC_VERSION,
            inavutil.msp.MSP_BUILD_INFO,
            inavutil.msp.MSP_BOARD_INFO,
            inavutil.msp.MSP_UID,
            inavutil.msp.MSP_ACC_TRIM,
            inavutil.msp.MSP_NAME,
            inavutil.msp.MSP_STATUS,
            inavutil.msp.MSP_STATUS_EX,
            inavutil.msp.MSP_BATTERY_CONFIG,
            inavutil.msp.MSP_BATTERY_STATE,
            inavutil.msp.MSP_BOXIDS,
            inavutil.msp.MSP2_INAV_STATUS,
            inavutil.msp.MSP2_INAV_ANALOG,
            inavutil.msp.MSP_VOLTAGE_METER_CONFIG,
            inavutil.msp.MSP_SENSOR_CONFIG,
            inavutil.msp.MSP_RC,
            inavutil.msp.MSP_RAW_GPS,
        ]
        default_tele_hz = 20
        self.msp_telemetry_msgs = {
            inavutil.msp.MSP2_INAV_ANALOG: default_tele_hz,
            inavutil.msp.MSP_BATTERY_STATE: default_tele_hz,
            inavutil.msp.MSP2_INAV_STATUS: default_tele_hz,
            inavutil.msp.MSP_MOTOR: default_tele_hz,
            inavutil.msp.MSP_RC: default_tele_hz,
            inavutil.msp.MSP_ATTITUDE: default_tele_hz,
            inavutil.msp.MSP_ALTITUDE: default_tele_hz,
            inavutil.msp.MSP_RAW_IMU: default_tele_hz,
            inavutil.msp.MSP_RAW_GPS: default_tele_hz,
            inavutil.msp.MSP_NAV_STATUS: default_tele_hz,
        }

        # ----- OTHER STATE --------------------------------------------
        self.voltage = 0
        self.sensors = {}
        self.attitude = {}
        self.imu = {}
        self.alt = {}
        self.nav_status = {}
        self.mission = [None] * 255
        self.armed = False
        self.batt_series = 0
        self.min_voltage = 0
        self.warn_voltage = 0
        self.max_voltage = 0

    # ------------------------------------------------------------------ #
    # CONNECTIONS                                                        #
    # ------------------------------------------------------------------ #
    def connect(self):
        self.connected = not self.board.connect(trials=self.board.ser_trials)
        if not self.connected:
            raise ConnectionError(f"Failed to connect to {self.device}")

    def disconnect(self):
        if not self.board.conn.closed:
            self.board.conn.close()
            self.connected = False
            print("FC Disconnected")

    # ------------------------------------------------------------------ #
    # CORE MSP I/O                                                       #
    # ------------------------------------------------------------------ #
    def std_send(self, msg_code: int, data=b""):
        if self.board.send_RAW_msg(msg_code, data, None, None, False):
            dh = self.board.receive_msg()
            self.board.process_recv_data(dh)
            return dh
        return None

    # ------------------------------------------------------------------ #
    # PID                                                                
    # ------------------------------------------------------------------ #
    def add_pid(
        self, pidname: str, Kp=0.1, Ki=0.01, Kd=0.05,
        setpoint=0, starting_output=0, output_limits=(0, 1)
    ):
        self.pids[pidname] = PID(
            Kp=Kp, Ki=Ki, Kd=Kd,
            setpoint=setpoint,
            starting_output=starting_output,
            output_limits=output_limits,
        )

    def update_pid(self, pidname: str, error: float):
        return self.pids[pidname](error)

    # ------------------------------------------------------------------ #
    # MODE CONFIG                                                        #
    # ------------------------------------------------------------------ #
    @staticmethod
    def deserialise_modes(buf):
        PERM_ARM = 0
        MAX_COUNT = 40
        i = 0
        mranges = []
        while i < len(buf) and len(mranges) < MAX_COUNT:
            if buf[i + 3] != 0:
                invalid = buf[0] == PERM_ARM and (buf[i + 3] - buf[i + 2]) > 40
                if not invalid:
                    mranges.append((buf[i], buf[i + 1], buf[i + 2], buf[i + 3]))
            i += 4
        mranges.sort(key=lambda x: (x[0], x[1]))
        return mranges

    def load_modes_config(self, echo=False):
        mranges = []
        code_value = inavutil.msp.MSP_MODE_RANGES
        while True:
            if self.board.send_RAW_msg(code_value, data=[]):
                dh = self.board.receive_msg()
                if dh["packet_error"] == 1:
                    raise Exception("Packet Error")
                if dh["code"] == code_value:
                    mranges = self.deserialise_modes(dh["dataView"])
                    break
        if not mranges:
            raise Exception("No mode settings returned")

        for i in mranges:
            name = inavutil.modesID_INAV.get(i[0])
            auxn = i[1]
            ch = auxn + 5
            vmin = 900 + i[2] * 25
            vmax = 900 + i[3] * 25
            flag = "OVERRIDE ALLOWED" if ch in self.msp_override_allowed_ch else ""
            if echo:
                print(
                    f"ID: {i[0]}\t{name:<16s}:\t{auxn} (channel {ch})\t= {vmin} to {vmax}\t{flag}"
                )
            self.modes[i[0]] = [ch, [vmin, vmax]]

        if echo and self.msp_override_allowed_ch:
            print("MSP Override Allowed:")
            for ch in self.msp_override_allowed_ch:
                print(f"Channel {ch}")

    # ------------------------------------------------------------------ #
    # ARM / MODE SWITCHING                                               #
    # ------------------------------------------------------------------ #
    def arm_enable_check(self):
        flags = self.board.process_armingDisableFlags(
            self.board.CONFIG["armingDisableFlags"]
        )
        valid = [
            f
            for f in flags
            if f
            not in (
                inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE,
                inavutil.armingDisableFlagNames_INAV.WAS_EVER_ARMED,
            )
        ]
        if valid:
            print("Cannot Arm, disable flags:")
            for f in valid:
                print(f"\t{inavutil.armingDisableFlagNames_INAV.get(f)}")
            return False
        return True

    def arm(self):
        flags = self.board.process_armingDisableFlags(
            self.board.CONFIG["armingDisableFlags"]
        )
        check = (not flags) or flags == [
            inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE
        ]
        if check and not self.board.bit_check(self.board.CONFIG["mode"], 0):
            self.set_mode("ARM", True)
            if self.board.bit_check(self.board.CONFIG["mode"], 0):
                print("### VEHICLE ARMED ###")
            else:
                print("!!! COULD NOT ARM !!!")
        elif not check:
            print(f"Could not arm, disable flags: {flags}")
        return self.board.bit_check(self.board.CONFIG["mode"], 0)

    def set_mode(self, mode: int, on: bool):
        if mode not in self.modes:
            return
        ch, pr = self.modes[mode]
        pwm = sum(pr) // 2 if on else 900
        self.channels[ch - 1] = pwm
        print(
            f"mode switch {inavutil.modesID_INAV.get(mode)} to {on} ch {ch-1} to {pwm}"
        )

    # ------------------------------------------------------------------ #
    # SUPERMODES                                                         #
    # ------------------------------------------------------------------ #
    def new_supermode(self, name: str, modes: list):
        for m in modes:
            if m not in self.modes:
                raise Exception(f"Mode {m} not programmed")
        self.supermodes[name] = modes

    def set_supermode(self, name: str, on: bool):
        if name not in self.supermodes:
            raise Exception(f"No such supermode {name}")
        if on:
            if name not in self.active_supermodes:
                self.active_supermodes.append(name)
            for m in self.supermodes[name]:
                self.set_mode(m, True)
        else:
            if name in self.active_supermodes:
                self.active_supermodes.remove(name)
            for m in self.supermodes[name]:
                self.set_mode(m, False)

    # ------------------------------------------------------------------ #
    # DATA ACCESS                                                         #
    # ------------------------------------------------------------------ #
    def get_active_modes(self):
        """Return list of mode IDs whose channel PWM is within its active window."""
        active = []
        for mode, (channel, pwm_range) in self.modes.items():
            chi = channel - 1
            if pwm_range[0] <= self.channels[chi] <= pwm_range[1]:
                active.append(mode)
        return active

    def get_rc_channels(self):
        if not self.run:
            self.std_send(inavutil.msp.MSP_RC)
        return self.board.RC["channels"]

    def set_msp_override_channel(self, channel, val):
        if channel in self.msp_override_allowed_ch:
            self.msp_override_channels[channel] = val

    def get_attitude(self):
        if not self.run:
            self.std_send(inavutil.msp.MSP_ATTITUDE)
        self.attitude = {
            "pitch": self.board.SENSOR_DATA["kinematics"][1],
            "yaw": self.board.SENSOR_DATA["kinematics"][2],
            "roll": self.board.SENSOR_DATA["kinematics"][0],
        }
        return self.attitude

    def get_altitude(self):
        if not self.run:
            self.std_send(inavutil.msp.MSP_ALTITUDE)
        self.alt = self.board.SENSOR_DATA["altitude"]
        return self.alt

    def get_nav_status(self):
        if not self.run:
            self.std_send(inavutil.msp.MSP_NAV_STATUS)
        ns = self.board.NAV_STATUS
        self.nav_status = {
            "mode": ns["mode"],
            "state": ns["state"],
            "wp_action": ns["active_wp_action"],
            "wp_number": ns["active_wp_number"],
            "error": ns["error"],
            "heading_hold_tgt": ns["heading_hold_target"],
        }
        return self.nav_status

    def get_gps_data(self):
        if not self.run:
            self.std_send(inavutil.msp.MSP_RAW_GPS)
        self.std_send(inavutil.msp.MSP_COMP_GPS)
        self.std_send(inavutil.msp.MSP_GPSSTATISTICS)
        ret = self.board.GPS_DATA
        ret["lat"] /= 1e7
        ret["lon"] /= 1e7
        ret["speed"] /= 100
        return ret

    # ------------------------------------------------------------------ #
    # TELEMETRY INIT                                                     #
    # ------------------------------------------------------------------ #
    def telemetry_init(self):
        if not self.telemetry_data_init:
            for msg in self.init_msp_msgs:
                self.std_send(msg)
            self.telemetry_data_init = True

    # ------------------------------------------------------------------ #
    # MAIN FLIGHT LOOP                                                   #
    # ------------------------------------------------------------------ #
    def flight_loop(self):
        CTRL_LOOP_TIME = 1 / 100  # 100 Hz
        self.telemetry_init()
        if not self.board.INAV:
            raise Exception("Non-INAV board detected")

        self.load_modes_config()
        self.msp_override = (
            inavutil.modesID_INAV.MSP_RC_OVERRIDE in self.modes
        )
        self.batt_series = self.board.BATTERY_STATE["cellCount"]
        cfg = self.board.BATTERY_CONFIG
        self.min_voltage = cfg["vbatmincellvoltage"] * self.batt_series
        self.warn_voltage = cfg["vbatwarningcellvoltage"] * self.batt_series
        self.max_voltage = cfg["vbatmaxcellvoltage"] * self.batt_series

        average_cycle = deque([0] * 10)
        telemetry_msgs_t = {m: time.time() for m in self.msp_telemetry_msgs}
        telemetry_msgs = cycle(self.msp_telemetry_msgs)

        self.run = True
        last_rc = time.time()
        self.get_rc_channels()
        print("\n### Flight Control loop started ###")
        try:
            while self.run:
                self.channels = self.board.RC["channels"][:16]

                start = time.time()
                self.armed = self.board.bit_check(self.board.CONFIG["mode"], 0)
                self.msp_override_active = (
                    self.is_override_active() if self.msp_override else False
                )

                # SEND RC ------------------------------------------------
                if self.msp_receiver or self.msp_override_active:
                    if (time.time() - last_rc) >= (1.0 / self.rc_interval):
                        last_rc = time.time()
                        masked = self.channels.copy()
                        for ch in self.msp_override_channels:
                            if ch in self.msp_override_allowed_ch:
                                masked[ch - 1] = self.msp_override_channels[ch]
                        if self.board.send_RAW_RC(masked):
                            dh = self.board.receive_msg()
                            self.board.process_recv_data(dh)

                # TELEMETRY ---------------------------------------------
                for msg in telemetry_msgs:
                    if (time.time() - telemetry_msgs_t[msg]) >= (
                        1.0 / self.msp_telemetry_msgs[msg]
                    ):
                        telemetry_msgs_t[msg] = time.time()
                        self.std_send(msg)
                        break

                # TIMING ------------------------------------------------
                elapsed = time.time() - start
                if elapsed < CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME - elapsed)
                average_cycle.append(elapsed)
                average_cycle.popleft()
        finally:
            print("\n### Flight Control loop finished ###")
            self.disconnect()

    # ------------------------------------------------------------------ #
    # HELPER                                                             
    # ------------------------------------------------------------------ #
    def is_override_active(self):
        if (
            self.msp_receiver
            and inavutil.modesID_INAV.MSP_RC_OVERRIDE in self.active_modes
        ):
            return True
        ch = self.modes[inavutil.modesID_INAV.MSP_RC_OVERRIDE][0] - 1
        v = self.board.RC["channels"][ch]
        rmin, rmax = self.modes[inavutil.modesID_INAV.MSP_RC_OVERRIDE][1]
        return rmin <= v <= rmax
