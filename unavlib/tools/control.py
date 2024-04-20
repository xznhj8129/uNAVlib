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
from unavlib import MSPy

class UAVControl:
    def __init__(self, device, baudrate):
        self.board = MSPy(device=device, loglevel='DEBUG', baudrate=baudrate)  # MSPy instance
        self.run = True
        self.connected = None
        self.device = device
        self.baudrate = baudrate
        self.executor = ThreadPoolExecutor()
        self.chorder = ['roll', 'pitch', 'throttle', 'yaw', 'ch5', 'ch6', 'ch7', 'ch8', 'ch9', 'ch10', 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16', 'ch17', 'ch18']
        self.channels = [900] * 18
        self.pwm_range = [1000,2000]
        self.modes = {}
        self.mode_range = [900,2100]
        self.mode_increments = 25
        self.pwm_midpoint = mean(self.pwm_range)
        self.msp_override = True
        self.msp_override_chs = []
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
        # Emulate the __enter__ method's connection setup
        self.connected = await asyncio.get_running_loop().run_in_executor(self.executor, self._try_connect)
        if not self.connected:
            raise ConnectionError(f"Failed to connect to {self.device}")

    def _try_connect(self):
        # This method tries to connect and mimics the __enter__ method of MSPy
        return not self.board.connect(trials=self.board.ser_trials)

    async def disconnect(self):
        # Emulate the __exit__ method's cleanup logic
        if not self.board.conn.closed:
            await asyncio.get_running_loop().run_in_executor(self.executor, self.board.conn.close)
            self.connected = False
            print('FC Disconnected')
        else:
            print('FC already disconnected')

    def run_sync(self, func, *args):
        # Helper method to run synchronous methods of MSPy in an executor
        return asyncio.get_running_loop().run_in_executor(self.executor, func, *args)

    def load_modes_config_file(self, mode_config_file):  # add function to get at init
        with open(mode_config_file,"r") as file:
            cfg = json.loads(file.read())
        del cfg["board_info"]
        self.modes = cfg

    def get_modes(self):
        for i in self.modes:
            print(i, self.modes[i])
        return self.modes.keys()

    def set_node(self, mode, on): # this needs to be more robust to handle when a mode is switched off
        if on:
            modemean = mean(self.modes[mode][1])
            self.channels[ self.modes[mode][0]-1 ] = modemean
        else:
            self.channels[ self.modes[mode][0]-1 ] = self.mode_range[0]

    def pwm_clamp(self, n):
        return max(min(self.pwm_range[0], n), self.pwm_range[1])

    def set_rc_channel(self, ch, value): # uses string indexes
        if self.msp_override and self.chorder.index(ch)+1 in self.msp_override_chs:
            self.channels[self.chorder.index(ch)] = value
        else:
            return None
        #else:
        #    chan = ch
        #    if self.msp_override and ch in self.msp_override_chs:
        #        self.channels[ch-1] = value

    def get_sensor_config(self):
        if self.board.send_RAW_msg(self.board.MSPCodes['MSP_SENSOR_CONFIG'], data=[]):
            dataHandler = self.board.receive_msg()
            ret = self.board.process_MSP_SENSOR_CONFIG(dataHandler)
            print(ret)
            return {
                'acc_hardware': inav_enums.accelerationSensor_R[self.board.SENSOR_CONFIG[0]], 
                'baro_hardware': dict_index(inav_enums.baroSensor, self.board.SENSOR_CONFIG[1]), 
                'mag_hardware': dict_index(inav_enums.magSensor, self.board.SENSOR_CONFIG[2]), 
                'pitot': dict_index(inav_enums.pitotSensor, self.board.SENSOR_CONFIG[3]),
                'rangefinder': dict_index(inav_enums.rangefinderType, self.board.SENSOR_CONFIG[4]), 
                'opflow': dict_index(inav_enums.opflowSensor, self.board.SENSOR_CONFIG[5])
            }
        else:
            return None

    def get_nav_status(self):
        if self.board.send_RAW_msg(MSPCodes['MSP_NAV_STATUS'], data=[]):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
            # here do  self.mspy.readbytes(data, size=n, unsigned=True) instead of struct
            mode = self.board.readbytes(dataHandler['dataView'], size=8, unsigned=True)
            state = self.board.readbytes(dataHandler['dataView'], size=8, unsigned=True)
            active_wp_action = self.board.readbytes(dataHandler['dataView'], size=8, unsigned=True)
            active_wp_number = self.board.readbytes(dataHandler['dataView'], size=8, unsigned=True)
            error = self.board.readbytes(dataHandler['dataView'], size=8, unsigned=True)
            heading_hold_target = self.board.readbytes(dataHandler['dataView'], size=16, unsigned=True)
            #mode, state, active_wp_action, active_wp_number, error, heading_hold_target = struct.unpack('<BBBBBH', dataHandler['dataView'])

            mode = dict_index(inav_enums.navSystemStatus_Mode, mode)
            state = dict_index(inav_enums.navSystemStatus_State, state)
            error = dict_index(inav_enums.navSystemStatus_Error, error)
            print()
            print('NAV_STATUS:')
            print(f"Mode: {mode}")
            print(f"State: {state}")
            print(f"Active WP Action: {active_wp_action}")
            print(f"Active WP Number: {active_wp_number}")
            print(f"Error: {error}")
            print(f"Heading Hold Target: {heading_hold_target}")
            return {
                "mode": mode,
                "state": state,
                "wp_action": active_wp_action,
                "wp_number": active_wp_number,
                "error": error,
                "heading_hold_tgt": heading_hold_target
            }
        else:
            return None

    def get_gps_data(self):
        if self.board.send_RAW_msg(MSPCodes['MSP_RAW_GPS'], data=[]):
            dataHandler = self.board.receive_msg()
            self.board.process_MSP_RAW_GPS(dataHandler['dataView'])
        else:
            return None
        if self.board.send_RAW_msg(MSPCodes['MSP_COMP_GPS'], data=[]):
            dataHandler = self.board.receive_msg()
            self.board.process_MSP_COMP_GPS(dataHandler['dataView'])
        else:
            return None
        if self.board.send_RAW_msg(MSPCodes['MSP_GPSSTATISTICS'], data=[]):
            dataHandler = self.board.receive_msg()
            self.board.process_MSP_GPSSTATISTICS(dataHandler['dataView'])
        else:
            return None
        print(self.board.GPS_DATA) # debug

    async def flight_loop(self):
        self.load_modes_config_file('modes.json')
        try:

            if self.board == 1: # an error occurred...
                return 1

            debugprint = False
            self.get_modes()
            
            CTRL_LOOP_TIME = 1/100 # 
            SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...
            NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if self.board.INAV:
                command_list.append('MSP2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')
            for msg in command_list: 
                if self.board.send_RAW_msg(MSPCodes[msg], data=[]):
                    dataHandler = self.board.receive_msg()
                    self.board.process_recv_data(dataHandler)
            if self.board.INAV:
                cellCount = self.board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSP2_INAV_ANALOG is necessary

            self.min_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            self.warn_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            self.max_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            if debugprint: print("apiVersion: {}".format(self.board.CONFIG['apiVersion']))
            if debugprint: print("flightControllerIdentifier: {}".format(self.board.CONFIG['flightControllerIdentifier']))
            if debugprint: print("flightControllerVersion: {}".format(self.board.CONFIG['flightControllerVersion']))
            if debugprint: print("boardIdentifier: {}".format(self.board.CONFIG['boardIdentifier']))
            if debugprint: print("boardName: {}".format(self.board.CONFIG['boardName']))
            if debugprint: print("name: {}".format(self.board.CONFIG['name']))

            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            while self.run:
                start_time = time.time()
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    #print(self.channels)
                    if self.board.send_RAW_RC(self.channels):
                        dataHandler = self.board.receive_msg()
                        self.board.process_recv_data(dataHandler)



                # SUPER FUCKING UNRELIABLE
                #print('imu:',self.board.fast_read_imu())
                #print('att:',self.board.fast_read_attitude()) 
                #print('alt:',self.board.fast_read_altitude())

                #
                # fix this shit
                #
                # SLOW MSG processing (user GUI)
                #
                
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC

                    if self.board.send_RAW_msg(MSPCodes[next_msg], data=[]):
                        dataHandler = self.board.receive_msg()
                        self.board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        self.voltage = self.board.ANALOG['voltage']

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = self.board.bit_check(self.board.CONFIG['mode'],0)
                        if debugprint: print("ARMED: {}".format(ARMED))
                        if debugprint: print("armingDisableFlags: {}".format(self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])))
                        if debugprint: print("\ncpuload: {}".format(self.board.CONFIG['cpuload']))
                        if debugprint: print("cycleTime: {}".format(self.board.CONFIG['cycleTime']))
                        if debugprint: print("mode: {}".format(self.board.CONFIG['mode']))
                        if debugprint: print("Flight Mode: {}".format(self.board.process_mode(self.board.CONFIG['mode'])))

                    elif next_msg == 'MSP_MOTOR':
                        if debugprint: print("Motor Values: {}".format(self.board.MOTOR_DATA))

                    #elif next_msg == 'MSP_RC':
                    #    if debugprint: print("RC Channels Values: {}".format(self.board.RC['channels']))
                        

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

                await asyncio.sleep(0.1)
            
            print('Flight control loop finished')
            await self.disconnect()
            return 0

        except Exception:
            print('Error in flight control loop')
            print(traceback.format_exc())
            await self.disconnect()
            return 1


                