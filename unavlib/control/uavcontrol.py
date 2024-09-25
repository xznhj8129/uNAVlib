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
from itertools import cycle # cycle('ABCD') → A B C D A B C D ...
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
from unavlib.modules import msp_ctrl
from unavlib.modules.boardconn import connMSP
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
            min_time_between_writes = self.mspy_min_time_between_writes)  # MSPy instance

        self.run = True
        self.connected = None
        self.device = device
        self.baudrate = baudrate
        self.executor = ThreadPoolExecutor()
        self.chorder = ['roll', 'pitch', 'throttle', 'yaw', 'ch5', 'ch6', 'ch7', 'ch8', 'ch9', 'ch10', 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16', 'ch17', 'ch18']
        self.channels = [900] * 16
        self.pwm_range = [900,2100]
        self.pwm_midpoint = mean(self.pwm_range)
        self.channels[0] = 1500 # bad things happens if sticks not centered
        self.channels[1] = 1500
        self.channels[3] = 1500
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
        self.rc_interval = 100 # Hz
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
            inavutil.msp.MSP_BOXIDS,  #instead of MSP_BOXNAMES
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

        #self.telemetry_intervals: {} # Default 10HZ
        #for i in self.msp_telemetry_msgs:
        #    self.telemetry_intervals[i] = 10
        #self.in_queue = asyncio.Queue()
        #self.out_queue = asyncio.Queue()

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
        self.connected = await asyncio.get_running_loop().run_in_executor(self.executor, self._try_connect)
        if not self.connected:
            raise ConnectionError(f"Failed to connect to {self.device}")

    def _try_connect(self):
        return not self.board.connect(trials=self.board.ser_trials)

    async def disconnect(self):
        if not self.board.conn.closed:
            await asyncio.get_running_loop().run_in_executor(self.executor, self.board.conn.close)
            self.connected = False
            print('FC Disconnected')
        else:
            print('FC already disconnected')

    def reboot(self): # implement auto-reconnect
        pass

    def std_send(self, msg_code:int, data=[]):
        # wrap the following used literally everywhere in the code
        if self.board.send_RAW_msg(msg_code, data=data, blocking=None, timeout=None, flush=False):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
            return dataHandler
        else:
            return None

    def add_pid(self, pidname:str, Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0, starting_output=0, output_limits=(0, 1)):
        self.pids[pidname] = PID(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=setpoint, starting_output=starting_output, output_limits=output_limits)

    def update_pid(self, pidname:str, error:float):
        return self.pids[pidname](error)

    def run_sync(self, func, *args):
        # Helper method to run synchronous methods of MSPy in an executor
        # need to (maybe) wrap all the MSP FC comm functions in here with this
        return asyncio.get_running_loop().run_in_executor(self.executor, func, *args)

    @staticmethod
    def deserialise_modes(buf):
        PERM_ARM = 0
        MAX_MODE_ACTIVATION_CONDITION_COUNT = 40
        i = 0
        mranges = []
        while i < len(buf) and len(mranges) < MAX_MODE_ACTIVATION_CONDITION_COUNT:
            if buf[i+3] != 0:
                invalid = (buf[0] == PERM_ARM and (buf[i+3]-buf[i+2]) > 40)
                if not invalid:
                    print(buf[i], buf[i+1], buf[i+2], buf[i+3])
                    mranges.append((buf[i], buf[i+1], buf[i+2], buf[i+3]))
            i += 4

        mranges.sort(key=lambda x: (x[0], x[1]))
        return mranges

    def load_modes_config(self):
        mranges = []
        msg_processed = False
        code_value = inavutil.msp.MSP_MODE_RANGES

        while not msg_processed:
            if self.board.send_RAW_msg(code_value, data=[]):
                dataHandler = self.board.receive_msg()
                
                if dataHandler['packet_error']==1:
                    raise Exception("Packet Error")

                if dataHandler['code'] == code_value:
                    msg_processed = True
                    buf = dataHandler["dataView"]
                    #print(buf)

                    PERM_ARM = 0
                    MAX_MODE_ACTIVATION_CONDITION_COUNT = 40
                    i = 0
                    while i < len(buf) and len(mranges) < MAX_MODE_ACTIVATION_CONDITION_COUNT:
                        if buf[i+3] != 0:
                            invalid = (buf[0] == PERM_ARM and (buf[i+3]-buf[i+2]) > 40)
                            if not invalid:
                                mranges.append((buf[i], buf[i+1], buf[i+2], buf[i+3]))
                        i += 4

        if len(mranges)==0:
            raise Exception("No mode settings returned from Flight Controller")
        
        channels = [[]] * 18
        for i in mranges:
            modename = inavutil.modesID.get(i[0])
            auxn = i[1]
            ch = auxn+5
            valmin = 900+(i[2]*25)
            valmax = 900+(i[3]*25)
            if ch in self.msp_override_channels:
                or_flag = "OVERRIDE ALLOWED"
            else:
                or_flag = ""
            print("ID: {}\t{:<16s}:\t{} (channel {})\t= {} to {}\t{}".format(i[0], modename, auxn, ch, valmin, valmax, or_flag))
            self.modes[i[0]] = [ch,[valmin, valmax]]

    def load_modes_config_file(self, mode_config_file): 
        with open(mode_config_file,"r") as file:
            cfg = json.loads(file.read())
        del cfg["board_info"]
        self.modes = cfg

    def arm_enable_check(self):
        arm_d_flags = self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])
        valid_flags = []
        for i in arm_d_flags:
            if i!= inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE and \
                i!= inavutil.armingDisableFlagNames_INAV.WAS_EVER_ARMED:
                valid_flags.append(i)
        if len(valid_flags)>0:
            print(f'Cannot Arm, disable flags:')
            for i in valid_flags:
                print(f"\t{inavutil.armingDisableFlagNames_INAV.get(i)}")
            return False
        else:
            return True

    def arm(self): # wonky, even need to use?
        arm_d_flags = self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])
        if arm_d_flags!=[inavutil.armingDisableFlagNames_INAV.SIMULATOR_MODE]:
            check = len(arm_d_flags) == 0
        else:
            check = True
        armed = self.board.bit_check(self.board.CONFIG['mode'],0)
        if check and not armed:
            self.set_mode('ARM', True)
            armed = self.board.bit_check(self.board.CONFIG['mode'],0)

            if armed: 
                print('### VEHICLE ARMED ###')
            else:
                print('!!! COULD NOT ARM !!!')
        elif not check:
            print(f'Could not arm, active disable flags: {arm_d_flags}')
        return armed

    def set_mode(self, mode: int, on: bool) -> None:
        if mode not in self.modes:
            return  # Mode doesn't exist
        
        channel, pwm_range = self.modes[mode]
        
        if on:
            pwm = (pwm_range[0] + pwm_range[1]) // 2
        else:
            pwm = 900
        self.channels[channel-1] = pwm
        print(f'mode switch {inavutil.modesID.get(mode)} to {on} ch {channel-1} to {pwm}')
    
    def get_active_modes(self) -> list:
        active_modes = []
        for mode, (channel, pwm_range) in self.modes.items():
            middle_value = (pwm_range[0] + pwm_range[1]) // 2
            if self.channels[channel] == middle_value:
                active_modes.append(mode)
        return active_modes

    def get_board_modes(self):
        boardmodes = self.board.process_mode(self.board.CONFIG['mode'])
        return  boardmodes

    def is_override_active(self):
        if self.msp_receiver and inavutil.modesID.MSP_RC_OVERRIDE in self.active_modes:
            return True
        elif not self.msp_receiver:
            modemean = mean(self.modes[inavutil.modesID.MSP_RC_OVERRIDE][1])
            ch = self.modes[inavutil.modesID.MSP_RC_OVERRIDE][0]-1
            chval = self.board.RC['channels'][ch]
            chrange = self.modes[inavutil.modesID.MSP_RC_OVERRIDE][1]
            if chval >= chrange[0] and chval <= chrange[1]:
                return True
            else:
                return False


    def new_supermode(self, name:str, modes:list):
        for i in modes:
            if i not in self.modes.keys():
                raise Exception(f'Mode {i} not programmed in flight controller')
        self.supermodes[name] = modes

    def set_supermode(self, supermode:str, on:bool):
        if supermode in self.supermodes:
            if on and supermode not in self.active_supermodes:
                self.active_supermodes.append(supermode)
                for mode in self.supermodes[supermode]:
                    self.set_mode(mode, on=True)

            elif not on and supermode in self.active_supermodes:
                del self.active_supermodes[self.active_supermodes.index(supermode)]
                for mode in self.supermodes[supermode]:
                    self.set_mode(mode, on=False)
        else:
            raise Exception(f"No such supermode {supermode}")

    def pwm_clamp(self, n:int):
        return max(min(self.pwm_range[0], n), self.pwm_range[1])

    def set_rc_channel(self, ch:str, value:int): # uses string indexes
        if (self.msp_receiver) or (self.msp_override and self.chorder.index(ch)+1 in self.msp_override_channels):
            self.channels[self.chorder.index(ch)] = value
        else:
            return None

    def get_sensor_config(self):
        #ret = self.std_send(inavutil.msp.MSP_SENSOR_CONFIG)
        #if ret:
        self.sensors = {
            'accelerationSensor': self.board.SENSOR_CONFIG['acc_hardware'], 
            'baroSensor': self.board.SENSOR_CONFIG['baro_hardware'], 
            'mag_hardware': self.board.SENSOR_CONFIG['mag_hardware'],
            'pitotSensor': self.board.SENSOR_CONFIG['pitot'],
            'rangefinderType': self.board.SENSOR_CONFIG['rangefinder'], 
            'opticalFlowSensor': self.board.SENSOR_CONFIG['opflow']
            }
        #    return self.sensors
        #else:
        #    return None

    def get_attitude(self):
        # Request, read and process the ATTITUDE (Roll, Pitch and Yaw in degrees)
        #if self.std_send(inavutil.msp.MSP_ATTITUDE):
        self.attitude = {"pitch": self.board.SENSOR_DATA['kinematics'][1],
                    "yaw": self.board.SENSOR_DATA['kinematics'][2],
                    "roll": self.board.SENSOR_DATA['kinematics'][0]}
        return self.attitude
    
    def get_altitude(self):
        #if self.std_send(inavutil.msp.MSP_ALTITUDE):
        self.alt = self.board.SENSOR_DATA['altitude']
        return self.alt

    def get_imu(self):
        #if self.stf_send(inavutil.msp.MSP_RAW_IMU):
        self.imu = {"accelerometer": self.board.SENSOR_DATA['accelerometer'],
                "gyroscope": self.board.SENSOR_DATA['gyroscope'],
                "magnetometer": self.board.SENSOR_DATA['magnetometer']}
        return self.imu

    def get_nav_status(self):
        #if self.std_send(inavutil.msp.MSP_NAV_STATUS):
        self.nav_status = {
            "mode": self.board.NAV_STATUS['mode'],
            "state": self.board.NAV_STATUS['state'],
            "wp_action": self.board.NAV_STATUS['active_wp_action'],
            "wp_number": self.board.NAV_STATUS['active_wp_number'],
            "error": self.board.NAV_STATUS['error'],
            "heading_hold_tgt": self.board.NAV_STATUS['heading_hold_target']
        }
        return self.nav_status

    def get_gps_data(self):
        #a = self.std_send(inavutil.msp.MSP_RAW_GPS)
        b = self.std_send(inavutil.msp.MSP_COMP_GPS)
        c = self.std_send(inavutil.msp.MSP_GPSSTATISTICS)
        if not b or not c:# or not a:
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

    async def telemetry_init(self):
        if not self.telemetry_data_init:
            for msg in self.init_msp_msgs: 
                self.std_send(msg)

            self.telemetry_data_init = True


    async def flight_loop(self):
        try:
            if self.board == 1: # an error occurred...
                return 1

            # load modes here
            #self.load_modes_config_file('modes.json')
            self.load_modes_config()
            self.msp_override = inavutil.modesID.MSP_RC_OVERRIDE in self.modes
            
            # default simpleUI values, to be set in another way
            # this is a problem
            CTRL_LOOP_TIME = 1/1000
            #telemetry_msgs_LOOP_TIME = 1/100 

            # artifact, remove it when you can
            NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            # see self.command_list
            await self.telemetry_init()

            if not self.board.INAV: # it always should be
                raise Exception('Non-INAV board detected')

            self.batt_series = self.board.BATTERY_STATE['cellCount']
            self.min_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*self.batt_series
            self.warn_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*self.batt_series
            self.max_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*self.batt_series

            if self.debugprint: print()
            if self.debugprint: print("apiVersion: {}".format(self.board.CONFIG['apiVersion']))
            if self.debugprint: print("flightControllerIdentifier: {}".format(self.board.CONFIG['flightControllerIdentifier']))
            if self.debugprint: print("flightControllerVersion: {}".format(self.board.CONFIG['flightControllerVersion']))
            if self.debugprint: print("boardIdentifier: {}".format(self.board.CONFIG['boardIdentifier']))
            if self.debugprint: print("boardName: {}".format(self.board.CONFIG['boardName']))
            if self.debugprint: print("name: {}".format(self.board.CONFIG['name']))

            telemetry_msgs_t = {}
            rc_interval_t = 0
            for i in msp_telemetry_msgs:
                telemetry_msgs_t[i] = time.time()
                
            telemetry_msgs = cycle(msp_telemetry_msgs)

            print('\n### Flight Control loop started ###')
            while self.run:
                start_time = time.time()
                sent = False

                self.armed = self.board.bit_check(self.board.CONFIG['mode'],0)
                self.msp_override_active = self.is_override_active() if self.msp_override else False

                # Send the RC channel values to the FC
                if (time.time()-rc_interval_t) >= (1.0 / self.rc_interval) and (self.msp_receiver or self.msp_override_active):
                    rc_interval_t = time.time()
                    sent = True
                    
                    if self.board.send_RAW_RC(self.channels):
                        dataHandler = self.board.receive_msg()
                        self.board.process_recv_data(dataHandler)


                # Telemetry messages (slow)
                for msg in telemetry_msgs:
                    lastsent = time.time() - telemetry_msgs_t[msg]
                    if (time.time()-lastsent) >= (1.0 / telemetry_msgs[i]):
                        telemetry_msgs_t[msg] = time.time()
                        self.std_send(msg)
                        break # only send one per cycle

                """if (time.time()-last_slow_msg_time) >= telemetry_msgs_LOOP_TIME:
                    sent=True
                    last_slow_msg_time = time.time()
                    next_msg = next(telemetry_msgs) # circular list
                    lastsent = time.time()-telemetry_msgs_t[next_msg]
                    telemetry_msgs_t[next_msg] = time.time()
                    self.std_send(next_msg)
                    
                    if next_msg == inavutil.msp.MSP_ANALOG: ######## not done
                        self.voltage = self.board.ANALOG['voltage']

                    elif next_msg == inavutil.msp.MSP_STATUS_EX:
                        ARMED = self.board.bit_check(self.board.CONFIG['mode'],0)
                        if self.debugprint: print("ARMED: {}".format(ARMED))
                        if self.debugprint: print("armingDisableFlags: {}".format(self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])))
                        if self.debugprint: print("Flight Mode: {}".format(self.board.process_mode(self.board.CONFIG['mode'])))

                    elif next_msg == inavutil.msp.MSP_MOTOR:
                        if self.debugprint: print("Motor Values: {}".format(self.board.MOTOR_DATA))

                    elif next_msg == inavutil.msp.MSP_RC:
                        if self.debugprint: print("Receiver RC Channels: ", self.board.RC['channels'])
                        if self.debugprint: print("Auto RC Channels:", self.channels)"""
                        

                end_time = time.time()
                last_cycleTime = end_time-start_time

                if (end_time-start_time)<CTRL_LOOP_TIME:
                    await asyncio.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()
            
            print('\n### Flight Control loop finished ###')
            await self.disconnect()
            return 0

        except Exception:
            print('!!! Error in Flight Control loop !!!')
            print(traceback.format_exc())
            await self.disconnect()
            self.run = False
            return 1


                