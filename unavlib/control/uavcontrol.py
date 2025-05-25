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
    def __init__(
        self, 
        device, 
        baudrate, 
        receiver="serial", 
        GPS=True, 
        msp_override_ch=[], 
        use_tcp=False, 
        platform="PLANE"
    ):
        # ----- INITIAL SET-UP -----------------------------------------
        self.debugprint = False
        self.loop_time = 1 / 250  # Hz
        self.mspy_min_time_between_writes = 1 / 50
        self.rc_interval = 20           # Hz
        default_tele_hz = 50
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
        #inav/docs/Rx.md at master · iNavFlight/inav
        #MSP_RC returns AERT regardless of channel map!!!
        self.chorder = [ # ?
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
        #
        self.channels[3] = 1500
        self.modes = {}
        self.active_modes = []
        self.supermodes = {}
        self.active_supermodes = []
        self.pids = {}
        self.gps_enabled = GPS
        self.msp_receiver = True if receiver=="msp" else False
        self.msp_override_active = False
        self.msp_override_allowed_ch = msp_override_ch
        self.msp_override_channels = {}
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
        self.msp_telemetry_msgs = {
            inavutil.msp.MSP2_INAV_ANALOG: default_tele_hz,
            inavutil.msp.MSP_BATTERY_STATE: default_tele_hz,
            inavutil.msp.MSP2_INAV_STATUS: default_tele_hz,
            inavutil.msp.MSP_MOTOR: default_tele_hz,
            inavutil.msp.MSP_RC: default_tele_hz,
            inavutil.msp.MSP_ATTITUDE: default_tele_hz*2,
            inavutil.msp.MSP_ALTITUDE: default_tele_hz,
            inavutil.msp.MSP_RAW_IMU: default_tele_hz,
        }
        if self.gps_enabled:
            self.msp_telemetry_msgs[inavutil.msp.MSP_RAW_GPS] = default_tele_hz
            self.msp_telemetry_msgs[inavutil.msp.MSP_NAV_STATUS] = default_tele_hz

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
    def std_send(self, msg_code:int, data=[]):
        if self.board.send_RAW_msg(msg_code, data=data, blocking=None, timeout=None, flush=False):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
            return dataHandler
        else:
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

    def load_modes_config_file(self, mode_config_file): 
        with open(mode_config_file,"r") as file:
            cfg = json.loads(file.read())
        del cfg["board_info"]
        self.modes = cfg

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

    def arm_check(self):
        return self.board.bit_check(self.board.CONFIG["mode"], 0)

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

    def pwm_clamp(self, n:int):
        return max(min(self.pwm_range[0], n), self.pwm_range[1])

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
        

    def get_sensor_config(self):
        if not self.run: self.std_send(inavutil.msp.MSP_SENSOR_CONFIG)
        self.sensors = {
            'accelerationSensor': self.board.SENSOR_CONFIG['acc_hardware'], 
            'baroSensor': self.board.SENSOR_CONFIG['baro_hardware'], 
            'mag_hardware': self.board.SENSOR_CONFIG['mag_hardware'],
            'pitotSensor': self.board.SENSOR_CONFIG['pitot'],
            'rangefinderType': self.board.SENSOR_CONFIG['rangefinder'], 
            'opticalFlowSensor': self.board.SENSOR_CONFIG['opflow']
            }

    def get_imu(self):
        if not self.run: self.stf_send(inavutil.msp.MSP_RAW_IMU)
        self.imu = {"accelerometer": self.board.SENSOR_DATA['accelerometer'],
                "gyroscope": self.board.SENSOR_DATA['gyroscope'],
                "magnetometer": self.board.SENSOR_DATA['magnetometer']}
        return self.imu

    def get_rc_channels(self):
        #inav/docs/Rx.md at master · iNavFlight/inav
        #MSP_RC returns AERT regardless of channel map!!!
        if not self.run:
            self.std_send(inavutil.msp.MSP_RC)
        chs = self.board.RC["channels"].copy()
        chs[self.chorder.index("roll")] = self.board.RC["channels"][0]
        chs[self.chorder.index("pitch")] = self.board.RC["channels"][1]
        chs[self.chorder.index("yaw")] = self.board.RC["channels"][2]
        chs[self.chorder.index("throttle")] = self.board.RC["channels"][3]
        return chs

    
    def get_ch(self, ch:str): # uses string indexes
        chi = self.chorder.index(ch)+1
        chs = self.board.RC["channels"].copy()
        chs[self.chorder.index("roll")] = self.board.RC["channels"][0]
        chs[self.chorder.index("pitch")] = self.board.RC["channels"][1]
        chs[self.chorder.index("yaw")] = self.board.RC["channels"][2]
        chs[self.chorder.index("throttle")] = self.board.RC["channels"][3]
        return chs[self.chorder.index(ch)]

    def set_rc_channel(self, ch:str, value:int): # uses string indexes
        chi = self.chorder.index(ch)+1
        if (self.msp_receiver) or (self.msp_override and chi in self.msp_override_allowed_ch):
            self.msp_override_channels[self.chorder.index(ch)] = value
        else:
            return None

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
        a = True if self.run else self.std_send(inavutil.msp.MSP_RAW_GPS)
        b = self.std_send(inavutil.msp.MSP_COMP_GPS)
        c = self.std_send(inavutil.msp.MSP_GPSSTATISTICS)
        if not b or not c or not a:
            return None
        else:
            ret = self.board.GPS_DATA
            ret['lat'] = ret['lat'] / 1e7
            ret['lon'] = ret['lon'] / 1e7
            ret['speed'] = ret['speed'] / 100
            return ret


    @staticmethod
    def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
        #print(f"packed: wp_no={wp_no}, action={action}, lat={lat}, lon={lon}, altitude={altitude}, p1={p1}, p2={p2}, p3={p3}, flag={flag}")
        msp_wp = struct.pack('<BBiiihhhB', wp_no, action, lat, lon, altitude, p1, p2, p3, flag)
        return msp_wp

    @staticmethod
    def unpack_msp_wp(msp_wp):
        unpacked_data = struct.unpack('<BBiiihhhB', msp_wp)
        return unpacked_data

    # ------------------------------------------------------------------ #
    # WAYPOINTS                                                          #
    # ------------------------------------------------------------------ #
    # https://github.com/iNavFlight/inav/wiki/MSP-Navigation-Messages
    # wp_no = 0-254 (0 = home, 254 = GCS NAV POSHOLD)
    # action = see inav/wiki/MSP-Navigation-Messages
    # latlon = standard decimal, converted
    # alt = in METERS, converted to CM
    # p1, p2, p3 = see inav/wiki/MSP-Navigation-Messages
    # flag = In general, flag is 0, unless it's the last point in a mission, in which case it is set to 0xa5 (165) (or 0x48 (72) for FBH WP)
    def set_wp(self, wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
        msp_wp = self.pack_msp_wp(int(wp_no), 
                                int(action), 
                                int(lat * 1e7), 
                                int(lon * 1e7), 
                                int(altitude*100), #you little shit
                                int(p1), 
                                int(p2), 
                                int(p3), 
                                int(flag))

        ret = self.std_send(inavutil.msp.MSP_SET_WP, data=msp_wp)
        if len(ret['dataView'])>0: #never returns anything?
            b = self.unpack_msp_wp(ret['dataView'])
            wp = geospatial.Waypoint(b[0] ,b[1], float(b[2]) / 1e7, float(b[3]) / 1e7, float(b[4]) / 100, b[5], b[6], b[7], b[8])
            return wp
        else:
            print("uavcontrol.set_wp: Waypoint not set")

    def _set_wp(self, wp):
        msp_wp = self.pack_msp_wp(wp.wp_no, 
                                wp.action, 
                                int(wp.pos.lat * 1e7), 
                                int(wp.pos.lon * 1e7), 
                                int(wp.pos.alt*100), 
                                wp.p1, 
                                wp.p2, 
                                wp.p3, 
                                wp.flag)
        ret = self.std_send(inavutil.msp.MSP_SET_WP, data=msp_wp)
        if len(ret['dataView'])>0: 
            b = self.unpack_msp_wp(ret['dataView'])
            wp = geospatial.Waypoint(b[0] ,b[1], float(b[2]) / 1e7, float(b[3]) / 1e7, float(b[4]) / 100, b[5], b[6], b[7], b[8])
            return wp
        else:
            print("uavcontrol.set_wp: Waypoint not set")

    def get_wp(self,wp_no):
        # Special waypoints are 0, 254, and 255. #0 returns the RTH (Home) position, #254 returns the current desired position (e.g. target waypoint), #255 returns the current position.
        ret = self.std_send(inavutil.msp.MSP_WP, data=struct.pack('B', wp_no))
        if ret:
            wp = geospatial.Waypoint(
                self.board.WP['wp_no'],
                self.board.WP['wp_action'], 
                float(self.board.WP['lat']) / 1e7, 
                float(self.board.WP['lon']) / 1e7, 
                float(self.board.WP['alt']) / 100, 
                self.board.WP['p1'], 
                self.board.WP['p2'],
                self.board.WP['p3'], 
                self.board.WP['flag']
                )
            return wp
        else:
            return None

    def get_wp_info(self):
        ret = self.std_send(inavutil.msp.MSP_WP_GETINFO, data=[])
        if ret:
            return {
                "reserved": self.board.WP_INFO["reserved"],
                "nav_max_waypoints": self.board.WP_INFO["NAV_MAX_WAYPOINTS"],
                "isWaypointListValid": self.board.WP_INFO["isWaypointListValid"] ,
                "WaypointCount": self.board.WP_INFO["WaypointCount"]
            }

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
        
        self.telemetry_init()
        if not self.board.INAV:
            raise Exception("Non-INAV board detected")

        self.load_modes_config()
        self.msp_override = (
            inavutil.modesID_INAV.MSP_RC_OVERRIDE in self.modes
        )
        print()
        print("Mapping:")
        for i, s in enumerate(self.chorder):
            print(f"{s}: CH{i+1}")
        
        self.batt_series = self.board.BATTERY_STATE['cellCount']
        self.min_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*self.batt_series
        self.warn_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*self.batt_series
        self.max_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*self.batt_series

        average_cycle = deque([self.loop_time] * 10) # Initialize with target time
        
        # telemetry_last_sent_timestamp: Stores the actual time.time() when a message was last sent
        # Initialize to 0 (or a time far in the past) to ensure messages are sent on first opportunity
        telemetry_last_sent_timestamp = {msg_code: 0 for msg_code in self.msp_telemetry_msgs.keys()}
        
        # telemetry_msg_codes_cycle: An iterator that cycles through the MSP codes for telemetry
        telemetry_msg_codes_cycle = cycle(self.msp_telemetry_msgs.keys())

        self.run = True
        last_rc_send_time = 0 # Initialize to ensure first send

        print("\n### Flight Control loop started ###")
        try:
            while self.run:
                current_loop_start_time = time.time()

                self.channels = self.board.RC.get("channels", self.channels)[:16]
                self.armed = self.board.bit_check(self.board.CONFIG["mode"], 0)
                
                self.msp_override_active = (
                    self.is_override_active() if self.msp_override else False
                )

                # SEND RC ------------------------------------------------
                if (current_loop_start_time - last_rc_send_time) >= (1.0 / self.rc_interval):
                    if self.msp_receiver or self.msp_override_active:
                        last_rc_send_time = current_loop_start_time # Update timestamp
                        
                        sent = True
                        masked = self.channels.copy()
                        
                        for ch in self.msp_override_channels:
                            if ch+1 in self.msp_override_allowed_ch:
                                masked[ch] = self.msp_override_channels[ch]

                        if self.board.send_RAW_RC(masked):
                            dataHandler = self.board.receive_msg()
                            self.board.process_recv_data(dataHandler)

                # TELEMETRY ---------------------------------------------
                for _i in range(len(self.msp_telemetry_msgs)): 
                    msg_code = next(telemetry_msg_codes_cycle)
                    
                    duration_since_last_send = current_loop_start_time - telemetry_last_sent_timestamp[msg_code]
                    
                    desired_interval = 1.0 / self.msp_telemetry_msgs[msg_code]
                    
                    if duration_since_last_send >= desired_interval:
                        self.std_send(msg_code) 
                        telemetry_last_sent_timestamp[msg_code] = current_loop_start_time # Update last sent timestamp
                        break # Sent one telemetry message, proceed to next flight_loop iteration

                # TIMING ------------------------------------------------
                elapsed_this_cycle = time.time() - current_loop_start_time
                
                sleep_duration = self.loop_time - elapsed_this_cycle
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
                
                final_cycle_time = time.time() - current_loop_start_time # For metrics
                average_cycle.append(final_cycle_time)
                average_cycle.popleft()



        except Exception: # Catch-all for the flight loop
            if shared.run:
                print('!!! Error in Flight Control loop !!!')
                print(traceback.format_exc())
                # self.disconnect() # disconnect is called in finally
                # self.run = False  # run is set in finally
                # No return 1 here, let finally handle cleanup
        finally:
            print("\n### Flight Control loop finished ###")
            self.disconnect() # Ensure disconnection
            self.run = False    # Ensure loop terminates if it broke out

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
