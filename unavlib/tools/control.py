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

from unavlib.enums import inav_enums
from unavlib.enums.msp_codes import MSPCodes
from unavlib.enums import msp_vars
from unavlib.modules.utils import dict_index
from unavlib.modules.utils import dict_reverse
from unavlib.modules.utils import TCPSocket
from unavlib.modules import msp_ctrl
from unavlib.modules.boardconn import connMSP
from unavlib.modules.process import processMSP
from unavlib.modules.fast_functions import fastMSP
from unavlib.modules import geo
from unavlib import MSPy

class UAVControl:
    def __init__(self, device, baudrate, drone_type="plane"):
        self.debugprint = False
        self.board = MSPy(device=device, loglevel='DEBUG', baudrate=baudrate)  # MSPy instance
        self.run = True
        self.connected = None
        self.device = device
        self.baudrate = baudrate
        self.executor = ThreadPoolExecutor()
        self.chorder = ['roll', 'pitch', 'throttle', 'yaw', 'ch5', 'ch6', 'ch7', 'ch8', 'ch9', 'ch10', 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16', 'ch17', 'ch18']
        self.channels = [900] * 16
        self.pwm_range = [1000,2000]
        self.pwm_midpoint = mean(self.pwm_range)
        self.channels[0] = 1500
        self.channels[1] = 1500
        self.channels[3] = 1500
        self.modes = {}
        self.active_modes = []
        self.mode_range = [900,2100]
        self.mode_increments = 25
        self.supermodes = {}
        self.active_supermodes = []
        self.pids = {}
        self.msp_override_channels = []
        self.mspy_min_time_between_writes = 1/100
        self.mspy_loglevel='INFO'
        self.mspy_timeout=1/100
        self.mspy_logfilename='MSPy.log' 
        self.mspy_logfilemode='a'
        #self.in_queue = asyncio.Queue()
        #self.out_queue = asyncio.Queue()

        self.voltage = 0
        self.attitude = {}
        self.imu = {}
        self.alt = {}
        self.nav_status = {}

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

    def std_send(self, msg_code:str, data=[]):
        if self.board.send_RAW_msg(self.board.MSPCodes[msg_code], data=data, blocking=None, timeout=None, flush=False):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
            return dataHandler
        else:
            return None

    def add_pid(self, pidname:str, Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0, output_limits=(0, 1)):
        self.pids[pidname] = PID(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=setpoint, output_limits=output_limits)

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
        code_value = self.board.MSPCodes['MSP_MODE_RANGES']

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
            modename = self.board.R_modesID_INAV[i[0]]
            ch = i[1]
            valmin = 900+(i[2]*25)
            valmax = 900+(i[3]*25)
            print("ID: {}\t{:<16s}:\t{} (channel {})\t= {} to {}".format(i[0], modename, ch, ch+5, valmin, valmax))
            self.modes[modename] = [ch+5,[valmin, valmax]]

    def load_modes_config_file(self, mode_config_file):  # add function to get at init
        with open(mode_config_file,"r") as file:
            cfg = json.loads(file.read())
        del cfg["board_info"]
        self.modes = cfg

    def arm_enable_check(self):
        arm_d_flags = self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])
        if len(arm_d_flags)>0:
            print(f'Cannot Arm, disable flags: {arm_d_flags}')
            return False
        else:
            return True

    def arm(self):
        arm_d_flags = self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])
        check = len(arm_d_flags) == 0
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

    def set_mode(self, mode:str, on:bool): # this needs to be more robust to handle when a mode is switched off
        if mode=='ARM':
            raise Exception(f"Cannot Arm through set_mode")
        else:
            if on and mode not in self.active_modes:
                modemean = mean(self.modes[mode][1])
                modech = self.modes[mode][0]-1
                self.channels[ modech ] = modemean
                self.active_modes.append(mode)

            elif mode in self.active_modes:
                self.channels[ self.modes[mode][0]-1 ] = self.mode_range[0]
                del self.active_modes[self.active_modes.index(mode)]

    def get_active_modes(self):
        # re-do this to just find active modes from current RC channel values
        boardmodes = self.board.process_mode(self.board.CONFIG['mode'])
        excluded = ["MSP RC OVERRIDE", "ARMED", "ARM"]
        for i in self.active_modes:
            if i not in excluded and i not in boardmodes:
                raise Exception(f"Mode mismatch detected, UAControl mode {i} not in Flight Controller mode flags")
        return self.active_modes

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
        if self.msp_override and self.chorder.index(ch)+1 in self.msp_override_channels:
            self.channels[self.chorder.index(ch)] = value
        else:
            return None

    def get_sensor_config(self):
        ret = self.std_send('MSP_SENSOR_CONFIG')
        if ret:
            return {
                    'acc_hardware': self.board.R_accelerationSensor[self.board.SENSOR_CONFIG['acc_hardware']], 
                    'baro_hardware': self.board.R_baroSensor[self.board.SENSOR_CONFIG['baro_hardware']], 
                    'mag_hardware': self.board.R_magSensor[self.board.SENSOR_CONFIG['mag_hardware']],
                    'pitot': self.board.R_pitotSensor[self.board.SENSOR_CONFIG['pitot']],
                    'rangefinder': self.board.R_rangefinderType[self.board.SENSOR_CONFIG['rangefinder']], 
                    'opflow': self.board.R_opticalFlowSensor[self.board.SENSOR_CONFIG['opflow']]
                }
        else:
            return None

    def get_attitude(self):
        # Request, read and process the ATTITUDE (Roll, Pitch and Yaw in degrees)
        ret = self.std_send('MSP_ATTITUDE')
        if ret:
            return {"pitch": self.board.SENSOR_DATA['kinematics'][1],
                    "yaw": self.board.SENSOR_DATA['kinematics'][2],
                    "roll": self.board.SENSOR_DATA['kinematics'][0]}
    
    def get_altitude(self):
        ret = self.std_send("MSP_ALTITUDE")
        if ret:
            return self.board.SENSOR_DATA['altitude']

    def get_imu(self):
        ret = self.stf_send('MSP_RAW_IMU')
        if ret:
            return {"accelerometer": self.board.SENSOR_DATA['accelerometer'],
                    "gyroscope": self.board.SENSOR_DATA['gyroscope'],
                    "magnetometer": self.board.SENSOR_DATA['magnetometer']}

    def get_nav_status(self):
        ret = self.std_send('MSP_NAV_STATUS')
        if ret:
            return {
                "mode": self.board.R_navSystemStatus_Mode[self.board.NAV_STATUS['mode']],
                "state": self.board.R_navSystemStatus_State[self.board.NAV_STATUS['state']],
                "wp_action": self.board.NAV_STATUS['active_wp_action'],
                "wp_number": self.board.NAV_STATUS['active_wp_number'],
                "error": self.board.R_navSystemStatus_Error[self.board.NAV_STATUS['error']],
                "heading_hold_tgt": self.board.NAV_STATUS['heading_hold_target']
            }

    def get_gps_data(self):
        a = self.std_send('MSP_RAW_GPS')
        b = self.std_send('MSP_COMP_GPS')
        c = self.std_send('MSP_GPSSTATISTICS')
        if not a or not b or not c:
            return None
        else:
            ret = self.board.GPS_DATA
            ret['lat'] = ret['lat'] / 1e7
            ret['lon'] = ret['lon'] / 1e7
            ret['speed'] = ret['speed'] / 100
            return ret


    @staticmethod
    def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
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
        msp_wp = self.pack_msp_wp(wp_no, 
                                action, 
                                int(lat * 1e7), 
                                int(lon * 1e7), 
                                altitude*100, 
                                p1, 
                                p2, 
                                p3, 
                                flag)
        ret = self.std_send('MSP_SET_WP', data=msp_wp)
        print(ret)
        if len(ret['dataView'])>0:
            return self.unpack_msp_wp(ret['dataView']) # parse me


    async def flight_loop(self):
        try:

            if self.board == 1: # an error occurred...
                return 1

            # load modes here
            #self.load_modes_config_file('modes.json')
            self.load_modes_config() #
            self.msp_override = "MSP RC OVERRIDE" in self.modes
            #print(self.modes, self.msp_override)
            
            # default simpleUI values, to be set in another way
            CTRL_LOOP_TIME = 1/100 
            SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...
            NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 
                            'MSP_FC_VARIANT', 
                            'MSP_FC_VERSION', 
                            'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 
                            'MSP_UID', 
                            'MSP_ACC_TRIM', 
                            'MSP_NAME', 
                            'MSP_STATUS', 
                            'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 
                            'MSP_BATTERY_STATE', 
                            'MSP_BOXNAMES', 
                            'MSP2_INAV_STATUS', 
                            'MSP2_INAV_ANALOG',
                            'MSP_VOLTAGE_METER_CONFIG',
                            'MSP_RC']

            for msg in command_list: 
                self.std_send(msg)

            if not self.board.INAV: # it always should be
                raise Exception('Non-INAV board detected')

            cellCount = self.board.BATTERY_STATE['cellCount']

            self.min_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            self.warn_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            self.max_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            if self.debugprint: print()
            if self.debugprint: print("apiVersion: {}".format(self.board.CONFIG['apiVersion']))
            if self.debugprint: print("flightControllerIdentifier: {}".format(self.board.CONFIG['flightControllerIdentifier']))
            if self.debugprint: print("flightControllerVersion: {}".format(self.board.CONFIG['flightControllerVersion']))
            if self.debugprint: print("boardIdentifier: {}".format(self.board.CONFIG['boardIdentifier']))
            if self.debugprint: print("boardName: {}".format(self.board.CONFIG['boardName']))
            if self.debugprint: print("name: {}".format(self.board.CONFIG['name']))

            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            print('\n### Flight Control loop started ###')
            while self.run:
                #if self.debugprint: print('\n########################################')
                start_time = time.time()
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME and "MSP RC OVERRIDE" in self.active_modes:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if self.board.send_RAW_RC(self.channels):
                        dataHandler = self.board.receive_msg()
                        self.board.process_recv_data(dataHandler)

                #
                # fix this 
                #
                # SLOW MSG processing (user GUI)
                #
                
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()
                    next_msg = next(slow_msgs) # circular list
                    self.std_send(next_msg)
                    
                    if next_msg == 'MSP_ANALOG':
                        self.voltage = self.board.ANALOG['voltage']

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = self.board.bit_check(self.board.CONFIG['mode'],0)
                        if self.debugprint: print("ARMED: {}".format(ARMED))
                        if self.debugprint: print("armingDisableFlags: {}".format(self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])))
                        if self.debugprint: print("cpuload: {}".format(self.board.CONFIG['cpuload']))
                        if self.debugprint: print("cycleTime: {}".format(self.board.CONFIG['cycleTime']))
                        if self.debugprint: print("Flight Mode: {}".format(self.board.process_mode(self.board.CONFIG['mode'])))

                    elif next_msg == 'MSP_MOTOR':
                        if self.debugprint: print("Motor Values: {}".format(self.board.MOTOR_DATA))

                    elif next_msg == 'MSP_RC':
                        if self.debugprint: print("RC Channels: ", self.board.RC['channels'])
                        

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

                await asyncio.sleep(0.1)
            
            print('\n### Flight Control loop finished ###')
            await self.disconnect()
            return 0

        except Exception:
            print('!!! Error in Flight Control loop !!!')
            print(traceback.format_exc())
            await self.disconnect()
            return 1


                