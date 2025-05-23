# uavcontrol.py

import asyncio
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
from itertools import cycle  # cycle('ABCD') → A B C D A B C D ...
from select import select
from threading import Lock, RLock
from statistics import mean
from concurrent.futures import ThreadPoolExecutor
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
        self.debugprint = False
        self.mspy_min_time_between_writes = 1/1000
        self.mspy_loglevel='INFO'
        self.mspy_timeout=1/100
        self.mspy_logfilename='MSPy.log' 
        self.mspy_logfilemode='a'

        self.board = MSPy(device=device, 
            loglevel=self.mspy_loglevel, 
            use_tcp=use_tcp, 
            baudrate=baudrate,
            min_time_between_writes = self.mspy_min_time_between_writes)

        self.run = False
        self.connected = None
        self.device = device
        self.baudrate = baudrate
        self.executor = ThreadPoolExecutor()
        self.chorder = [
            'roll','pitch','throttle','yaw',
            'ch5','ch6','ch7','ch8','ch9','ch10',
            'ch11','ch12','ch13','ch14','ch15','ch16',
            'ch17','ch18'
        ]
        self.channels = [900] * 16
        self.pwm_range = [900,2100]
        self.pwm_midpoint = mean(self.pwm_range)
        self.channels[0:4] = [1500]*4
        self.modes = {}
        self.active_modes = []
        self.mode_range = [900,2100]
        self.mode_increments = 25
        self.supermodes = {}
        self.active_supermodes = []
        self.pids = {}
        self.msp_receiver = False
        self.msp_override_active = False
        self.msp_override_channels = []
        self.rc_interval = 100  # Hz
        self.telemetry_data_init = False

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

        default_tele_hz = 10
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
            inavutil.msp.MSP_NAV_STATUS: default_tele_hz
        }

        self.voltage = 0
        self.sensors = {}
        self.attitude = {}
        self.imu = {}
        self.alt = {}
        self.nav_status = {}
        self.mission = [None]*255
        self.armed = False
        self.batt_series = 0
        self.min_voltage = 0
        self.warn_voltage = 0
        self.max_voltage = 0

    def stop(self):
        self.run = False

    async def connect(self):
        self.connected = await asyncio.get_running_loop().run_in_executor(
            self.executor, self._try_connect
        )
        if not self.connected:
            raise ConnectionError(f"Failed to connect to {self.device}")

    def _try_connect(self):
        return not self.board.connect(trials=self.board.ser_trials)

    async def disconnect(self):
        if not self.board.conn.closed:
            await asyncio.get_running_loop().run_in_executor(
                self.executor, self.board.conn.close
            )
            self.connected = False
            print('FC Disconnected')

    def reboot(self):
        pass  # left intentionally synchronous

    async def run_sync(self, func, *args):
        return await asyncio.get_running_loop().run_in_executor(
            self.executor, func, *args
        )

    async def std_send(self, msg_code: int, data=[]):
        sent = await self.run_sync(
            self.board.send_RAW_msg, msg_code, data, None, None, False
        )
        if sent:
            dh = await self.run_sync(self.board.receive_msg)
            await self.run_sync(self.board.process_recv_data, dh)
            return dh
        return None

    def add_pid(self, pidname:str, Kp=0.1, Ki=0.01, Kd=0.05,
                setpoint=0, starting_output=0, output_limits=(0, 1)):
        self.pids[pidname] = PID(
            Kp=Kp, Ki=Ki, Kd=Kd,
            setpoint=setpoint,
            starting_output=starting_output,
            output_limits=output_limits
        )

    def update_pid(self, pidname:str, error:float):
        return self.pids[pidname](error)

    @staticmethod
    def deserialise_modes(buf):
        PERM_ARM = 0
        MAX_COUNT = 40
        i = 0
        mranges = []
        while i < len(buf) and len(mranges) < MAX_COUNT:
            if buf[i+3] != 0:
                invalid = (buf[0] == PERM_ARM and (buf[i+3]-buf[i+2]) > 40)
                if not invalid:
                    mranges.append((buf[i], buf[i+1], buf[i+2], buf[i+3]))
            i += 4
        mranges.sort(key=lambda x: (x[0], x[1]))
        return mranges

    def load_modes_config(self):
        mranges = []
        code_value = inavutil.msp.MSP_MODE_RANGES
        while True:
            if self.board.send_RAW_msg(code_value, data=[]):
                dh = self.board.receive_msg()
                if dh['packet_error'] == 1:
                    raise Exception("Packet Error")
                if dh['code'] == code_value:
                    buf = dh["dataView"]
                    mranges = self.deserialise_modes(buf)
                    break
        if not mranges:
            raise Exception("No mode settings returned")
        for i in mranges:
            name = inavutil.modesID_INAV.get(i[0])
            auxn = i[1]
            ch = auxn+5
            vmin = 900 + i[2]*25
            vmax = 900 + i[3]*25
            flag = "OVERRIDE ALLOWED" if ch in self.msp_override_channels else ""
            print(f"ID: {i[0]}\t{name:<16s}:\t{auxn} (channel {ch})\t= {vmin} to {vmax}\t{flag}")
            self.modes[i[0]] = [ch, [vmin, vmax]]

    def load_modes_config_file(self, mode_config_file): 
        with open(mode_config_file,"r") as f:
            cfg = json.load(f)
        del cfg["board_info"]
        self.modes = cfg

    def arm_enable_check(self):
        flags = self.board.process_armingDisableFlags(
            self.board.CONFIG['armingDisableFlags']
        )
        valid = [
            f for f in flags
            if f not in (
                inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE,
                inavutil.armingDisableFlagNames_INAV.WAS_EVER_ARMED
            )
        ]
        if valid:
            print('Cannot Arm, disable flags:')
            for f in valid:
                print(f"\t{inavutil.armingDisableFlagNames_INAV.get(f)}")
            return False
        return True

    def arm(self):
        flags = self.board.process_armingDisableFlags(
            self.board.CONFIG['armingDisableFlags']
        )
        check = (not flags) or flags == [
            inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE
        ]
        if check and not self.board.bit_check(self.board.CONFIG['mode'], 0):
            self.set_mode('ARM', True)
            if self.board.bit_check(self.board.CONFIG['mode'], 0):
                print('### VEHICLE ARMED ###')
            else:
                print('!!! COULD NOT ARM !!!')
        elif not check:
            print(f'Could not arm, disable flags: {flags}')
        return self.board.bit_check(self.board.CONFIG['mode'], 0)

    def set_mode(self, mode: int, on: bool):
        if mode not in self.modes:
            return
        ch, pr = self.modes[mode]
        pwm = (sum(pr)//2) if on else 900
        self.channels[ch-1] = pwm
        print(f'mode switch {inavutil.modesID_INAV.get(mode)} to {on} ch {ch-1} to {pwm}')

    def get_active_modes(self):
        act = []
        for m, (ch, pr) in self.modes.items():
            if self.channels[ch] == sum(pr)//2:
                act.append(m)
        return act

    def get_board_modes(self):
        if not self.run:
            # fire-and-forget
            asyncio.create_task(self.std_send(inavutil.msp.MSP2_INAV_STATUS))
        return self.board.process_mode(self.board.CONFIG['mode'])

    def is_override_active(self):
        if self.msp_receiver and inavutil.modesID_INAV.MSP_RC_OVERRIDE in self.active_modes:
            return True
        ch = self.modes[inavutil.modesID_INAV.MSP_RC_OVERRIDE][0] - 1
        v = self.board.RC['channels'][ch]
        rmin, rmax = self.modes[inavutil.modesID_INAV.MSP_RC_OVERRIDE][1]
        return rmin <= v <= rmax

    def new_supermode(self, name:str, modes:list):
        for m in modes:
            if m not in self.modes:
                raise Exception(f'Mode {m} not programmed')
        self.supermodes[name] = modes

    def set_supermode(self, name:str, on:bool):
        if name not in self.supermodes:
            raise Exception(f"No such supermode {name}")
        if on:
            self.active_supermodes.append(name)
            for m in self.supermodes[name]:
                self.set_mode(m, True)
        else:
            self.active_supermodes.remove(name)
            for m in self.supermodes[name]:
                self.set_mode(m, False)

    def pwm_clamp(self, n:int):
        return max(min(self.pwm_range[0], n), self.pwm_range[1])

    async def get_rc_channels(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_RC)
        self.channels = self.board.RC['channels']
        return self.channels

    async def get_attitude(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_ATTITUDE)
        self.attitude = {
            "pitch": self.board.SENSOR_DATA['kinematics'][1],
            "yaw":   self.board.SENSOR_DATA['kinematics'][2],
            "roll":  self.board.SENSOR_DATA['kinematics'][0]
        }
        return self.attitude

    async def get_altitude(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_ALTITUDE)
        self.alt = self.board.SENSOR_DATA['altitude']
        return self.alt

    async def get_imu(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_RAW_IMU)
        self.imu = {
            "accelerometer": self.board.SENSOR_DATA['accelerometer'],
            "gyroscope":     self.board.SENSOR_DATA['gyroscope'],
            "magnetometer":  self.board.SENSOR_DATA['magnetometer']
        }
        return self.imu

    async def get_analog(self): # Ok, why do i send the data to another variable when it's already in there
        if not self.run:
            await self.std_send(inavutil.msp.MSP2_INAV_ANALOG)
        return self.board.ANALOG

    async def get_nav_status(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_NAV_STATUS)
        ns = self.board.NAV_STATUS
        self.nav_status = {
            "mode":             ns['mode'],
            "state":            ns['state'],
            "wp_action":        ns['active_wp_action'],
            "wp_number":        ns['active_wp_number'],
            "error":            ns['error'],
            "heading_hold_tgt": ns['heading_hold_target']
        }
        return self.nav_status

    async def get_gps_data(self):
        if not self.run:
            await self.std_send(inavutil.msp.MSP_RAW_GPS)
        await self.std_send(inavutil.msp.MSP_COMP_GPS)
        await self.std_send(inavutil.msp.MSP_GPSSTATISTICS)
        ret = self.board.GPS_DATA
        ret['lat']   /= 1e7
        ret['lon']   /= 1e7
        ret['speed'] /= 100
        return ret

    @staticmethod
    def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
        return struct.pack('<BBiiihhhB', wp_no, action, lat, lon, altitude, p1, p2, p3, flag)

    @staticmethod
    def unpack_msp_wp(msp_wp):
        return struct.unpack('<BBiiihhhB', msp_wp)

    async def set_wp(self, wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
        msp_wp = self.pack_msp_wp(
            int(wp_no), int(action),
            int(lat*1e7), int(lon*1e7),
            int(altitude*100), int(p1), int(p2), int(p3), int(flag)
        )
        ret = await self.std_send(inavutil.msp.MSP_SET_WP, data=msp_wp)
        if ret and ret['dataView']:
            b = self.unpack_msp_wp(ret['dataView'])
            return geospatial.Waypoint(
                b[0], b[1],
                float(b[2])/1e7, float(b[3])/1e7,
                float(b[4])/100, b[5], b[6], b[7], b[8]
            )
        print("uavcontrol.set_wp: Waypoint not set")

    async def get_wp(self, wp_no):
        ret = await self.std_send(inavutil.msp.MSP_WP, data=struct.pack('B', wp_no))
        if ret:
            wp = self.board.WP
            return geospatial.Waypoint(
                wp['wp_no'], wp['wp_action'],
                float(wp['lat'])/1e7, float(wp['lon'])/1e7,
                float(wp['alt'])/100, wp['p1'], wp['p2'], wp['p3'], wp['flag']
            )
        return None

    async def get_wp_info(self):
        ret = await self.std_send(inavutil.msp.MSP_WP_GETINFO)
        if ret:
            wi = self.board.WP_INFO
            return {
                "reserved":           wi["reserved"],
                "nav_max_waypoints":  wi["NAV_MAX_WAYPOINTS"],
                "isWaypointListValid":wi["isWaypointListValid"],
                "WaypointCount":      wi["WaypointCount"]
            }

    async def telemetry_init(self):
        if not self.telemetry_data_init:
            for msg in self.init_msp_msgs:
                await self.std_send(msg)
            self.telemetry_data_init = True

    async def flight_loop(self):
        CTRL_LOOP_TIME = 1/1000
        await self.telemetry_init()
        if not self.board.INAV:
            raise Exception('Non-INAV board detected')

        await self.run_sync(self.load_modes_config)
        self.msp_override = inavutil.modesID_INAV.MSP_RC_OVERRIDE in self.modes
        self.batt_series = self.board.BATTERY_STATE['cellCount']
        cfg = self.board.BATTERY_CONFIG
        self.min_voltage  = cfg['vbatmincellvoltage']    * self.batt_series
        self.warn_voltage = cfg['vbatwarningcellvoltage']* self.batt_series
        self.max_voltage  = cfg['vbatmaxcellvoltage']    * self.batt_series

        average_cycle = deque([0]*10)
        telemetry_msgs_t = {m: time.time() for m in self.msp_telemetry_msgs}
        telemetry_msgs = cycle(self.msp_telemetry_msgs)

        self.run = True
        print('\n### Flight Control loop started ###')
        try:
            while self.run:
                start = time.time()
                self.armed = self.board.bit_check(self.board.CONFIG['mode'], 0)
                self.msp_override_active = self.is_override_active() if self.msp_override else False

                # RC send
                if (time.time() - telemetry_msgs_t[inavutil.msp.MSP_RC]) >= (1.0 / self.rc_interval) \
                   and (self.msp_receiver or self.msp_override_active):
                    telemetry_msgs_t[inavutil.msp.MSP_RC] = time.time()
                    sent = await self.run_sync(self.board.send_RAW_RC, self.channels)
                    if sent:
                        dh = await self.run_sync(self.board.receive_msg)
                        await self.run_sync(self.board.process_recv_data, dh)

                # slow telemetry
                for msg in telemetry_msgs:
                    if (time.time() - telemetry_msgs_t[msg]) >= (1.0 / self.msp_telemetry_msgs[msg]):
                        telemetry_msgs_t[msg] = time.time()
                        await self.std_send(msg)
                        break

                elapsed = time.time() - start
                if elapsed < CTRL_LOOP_TIME:
                    await asyncio.sleep(CTRL_LOOP_TIME - elapsed)
                average_cycle.append(elapsed)
                average_cycle.popleft()
        finally:
            print('\n### Flight Control loop finished ###')
            await self.disconnect()
