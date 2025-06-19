# Reworking flight loop for more reliability, serial issues

from collections import deque
from itertools import cycle 

import asyncio
import traceback
import socket
import json
import select
import errno
import struct
import numpy as np
import math
from copy import deepcopy 
import asyncio
import os
import signal
import time
import sys

from unavlib.control.uavcontrol import UAVControl
from unavlib.modules.utils import inavutil
from unavlib.modules import geospatial

# ------------------------------------------------------------------ #

MSP_PORT = "/dev/ttyACM0"

LOOP_SLEEP = 0.02
CONNECTION_TIMEOUT = 0.5
RECONNECT_DELAY = 2.0            



class SharedData:
    def __init__(self):
        self.run = True

        self.rc_output = {}

        self.flight_state = {           
            "time": 0.0, 
            "pos": np.array([0.0, 0.0, 0.0]),
            "vel": np.array([0.0, 0.0, 0.0]),
            "angles": np.array([0.0, 0.0, 0.0]),
            "ang_vel": np.array([0.0, 0.0, 0.0]),
            "speed": 0.0,
            "altitude": 0.0,
            "rc": self.rc_input,
            "override_output": self.override_output,
        }


# ------------------------------------------------------------------ #

async def main_control_loop(shared: SharedData, uav: UAVControl):

    uav = UAVControl(
        device="/dev/ttyACM0",
        baudrate=115200,
        platform="COPTER",
        receiver="serial",
        GPS=False,
        msp_override_ch=[1,2,3,4]
    )
    uav.msp_receiver = False
    uav.debugprint = False
    print("Connected to the flight controller")

    loop_time = 1 / 100.0
    rc_interval = 1 / 30.0
    print_interval = 1 / 4.0

    loopn = 0
    last_loop_time = 0
    last_rc_time = 0
    last_print_time = 0

    default_tele_hz = 5
    msp_telemetry_msgs = {
        inavutil.msp.MSP2_INAV_ANALOG: default_tele_hz,
        inavutil.msp.MSP_BATTERY_STATE: default_tele_hz,
        inavutil.msp.MSP2_INAV_STATUS: default_tele_hz,
        #inavutil.msp.MSP_MOTOR: default_tele_hz,
        inavutil.msp.MSP_RC: default_tele_hz,
        inavutil.msp.MSP_ATTITUDE: default_tele_hz,
        inavutil.msp.MSP_ALTITUDE: default_tele_hz,
        inavutil.msp.MSP_RAW_IMU: default_tele_hz,
    }
    telemetry_last_sent_timestamp = {msg_code: 0 for msg_code in msp_telemetry_msgs.keys()}
    telemetry_msg_codes_cycle = cycle(msp_telemetry_msgs.keys())

    #planned_msg_sec = 0
    #cycles_for_tele = default_tele_hz * len(msp_telemetry_msgs)
    #print("cycles_for_tele:",cycles_for_tele)
    #time_to_send = cycles_for_tele * loop_time # 1 msg/cycle
    #print(f"Time to send full tele: {time_to_send*1000:.3f} ms")
    #planned_msg_sec += len(msp_telemetry_msgs) * ( 1.0 / time_to_send )
    #planned_msg_sec += ( 1.0 / rc_interval )
    #print("Planned messages/second:",planned_msg_sec)
    
    try:
        uav.connect()
        uav.telemetry_init()
        uav.load_modes_config(echo=True)
        print()
        print("Mapping:")
        for i, s in enumerate(uav.chorder):
            print(f"{s}: CH{i+1}")

        start_time = time.monotonic()
        msg_counter = uav.board.total_sent

        while shared.run:
            if not uav.connected:
                print('Error: FC not connected')
                await asyncio.sleep(0.5)
                uav.connect()
                if not uav.connected:
                    shared.run = False
            curtime = time.monotonic()
            uav.board_channels = uav.board.RC["channels"][:16]

            # TELEMETRY ---------------------------------------------
            for _i in range(len(msp_telemetry_msgs)): 
                msg_code = next(telemetry_msg_codes_cycle)
                
                last_send = curtime - telemetry_last_sent_timestamp[msg_code]
                
                desired_interval = 1.0 / msp_telemetry_msgs[msg_code]
                
                if last_send >= desired_interval:
                    #print(f"{loopn} {inavutil.msp.get(msg_code)}\t{last_send:.3f}\t{desired_interval:.3f}")
                    uav.std_send(msg_code) 
                    telemetry_last_sent_timestamp[msg_code] = curtime 
                    break 

            # SEND RC ------------------------------------------------
            if override or self.msp_receiver:
                uav.set_rc_channel("roll", shared.override_output["roll"])
                uav.set_rc_channel("pitch", shared.override_output["pitch"])
                uav.set_rc_channel("yaw", shared.override_output["yaw"])
                uav.set_rc_channel("throttle", shared.override_output["throt"])
                
                if (curtime - last_rc_time) >= rc_interval:
                    masked = uav.channels.copy()
                    last_send = curtime - last_rc_time
                    for ch in uav.msp_override_channels:
                        if ch+1 in uav.msp_override_allowed_ch:
                            masked[ch] = uav.msp_override_channels[ch]

                    #print(f"{loopn} SEND_RAW_RC\t{last_send:.3f}\t{rc_interval:.3f}")
                    if uav.board.send_RAW_RC(masked):
                        dataHandler = uav.board.receive_msg()
                        uav.board.process_recv_data(dataHandler)

                    last_rc_time = curtime


            # TIMING ------------------------------------------------
            msgtimer = uav.board.total_sent - msg_counter
            since_last = time.monotonic() - last_print_time
            msg_sec = msgtimer / since_last
            msg_counter = uav.board.total_sent

            elapsed_this_cycle = time.monotonic() - curtime
            free_time = loop_time - elapsed_this_cycle
            since_start = time.monotonic() - start_time
            sleep_time = free_time

            loopn +=1
            last_loop_time = curtime
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except Exception:
        print("!!! Autipilot error:")
        traceback.print_exc()
        

    finally:
        uav.disconnect()
        shared.run = False
        print('\n### Autopilot stopped')
        
# ------------------------------------------------------------------ #

async def user_loop(shared: SharedData, uav: UAVControl):
    print("User script started.")
    
    loop_sim_time = 0.0 
    last_print_time = 0.0
    print_interval = 0.25
    
    while shared.run:
        curtime = time.monotonic()

        alt = uav.get_altitude()
        gyro = uav.get_imu()['gyroscope'] #dps
        rad_gyro = [math.radians(i) for i in gyro]
        attitude = uav.get_attitude()
        channels = uav.get_rc_channels()
        am = uav.get_active_modes()
        m = []
        for i in am:
            m.append(inavutil.modesID_INAV.get(i))

        override = uav.is_override_active()
        failsafe = inavutil.modesID_INAV.FAILSAFE in am
        armed = uav.arm_check() and not failsafe
        

        if (curtime - last_print_time) >= print_interval:
            print("ARMED" if armed else "DISARMED")
            if failsafe: print("FAILSAFE")
            if override: print("OVERRIDE")
            print('Modes:',m)
            print(f"Alt: {alt}")
            print(f"Attitude: {attitude}")
            print(f"Gyro: {rad_gyro}")
            print(f"Channels: {channels}")
            print(f"Total Messages sent: {uav.board.total_sent}")
            print(f"Messages/second: {msg_sec:.3f}")
            print(f"cpuload: {uav.board.CONFIG['cpuload']}")
            print(f"cycleTime: {uav.board.CONFIG['cycleTime']}")
            last_print_time = curtime

            if override:
                for i in shared.override_output:
                    print(f'\t{i}: {shared.override_output[i]}')
        

        await asyncio.sleep(0.5)
    print("User script stopped.")


# ------------------------------------------------------------------ #

async def main():
    shared = SharedData()
    uav = UAVControl(
        device="/dev/ttyACM0",
        baudrate=115200,
        platform="COPTER",
        receiver="serial",
        GPS=False,
        msp_override_ch=[1,2,3,4]
    )

    fc_task = asyncio.create_task(main_control_loop(shared, uav))
    ctl_task = asyncio.create_task(user_loop(shared))

    try:
        await asyncio.gather(fc_task, ctl_task)
    except Exception:
        print("Error in one of the loops:")
        traceback.print_exc()
    except asyncio.exceptions.CancelledError:
        pass
    finally:
        shared.run = False
        await asyncio.to_thread(uav.disconnect)
        print("Connection closed")

if __name__ == "__main__":
    asyncio.run(main())

