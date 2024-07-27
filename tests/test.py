import asyncio
import socket
import time
from unavlib.control import UAVControl
from unavlib.control import geospatial

import struct
from unavlib import MSPy
from unavlib.modules.utils import dict_reverse

R_MSPCodes = dict_reverse(MSPy.MSPCodes)

# Define a function to pack data into the MSP waypoint structure
def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
    msp_wp = struct.pack('<BBiiihhhB', wp_no, action, lat, lon, altitude, p1, p2, p3, flag)
    return msp_wp

def unpack_msp_wp(msp_wp):
    unpacked_data = struct.unpack('<BBiiihhhB', msp_wp)
    return unpacked_data

def send(board, msg_code, data=[]):
    print('sending',msg_code,MSPy.MSPCodes[msg_code],data)
    if board.send_RAW_msg(MSPy.MSPCodes[msg_code], data=data):
        dataHandler = board.receive_msg()
        print('received', R_MSPCodes[dataHandler['code']])
        board.process_recv_data(dataHandler)
        return dataHandler

async def telemetry_display(uav):
    uav.debugprint = False
    #await asyncio.sleep(3)
    #uav.new_supermode('GOTO', ["GCS NAV", "NAV POSHOLD"])
    # assuming proper compile flag and bitmask config
    #uav.set_mode("MSP RC OVERRIDE", on=True)
    #uav.set_supermode("GOTO", on=True)
    #while "GCS NAV" not in uav.get_active_modes() and "FAILSAFE" in uav.get_active_modes():
    #    await asyncio.sleep(1)

    print('Modes:', uav.board.CONFIG['mode'], uav.get_active_modes())
    wp = geospatial.GPSposition(45.487363, -73.812242, 20)
    a = uav.set_wp(2, 1, wp.lat, wp.lon, 100, 0, 0, 0, 0)

    print()
    print('MSP_WP_GETINFO')
    # https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c#L1423
    wpinfo = uav.get_wp_info()

    print()
    print('MSP_WP')
    # Special waypoints are 0, 254, and 255. #0 returns the RTH (Home) position, #254 returns the current desired position (e.g. target waypoint), #255 returns the current position.
    for i in range(1, wpinfo['WaypointCount']+1):
        iwp = uav.get_wp(i)
        print(iwp)
    #await asyncio.sleep(10)
    uav.stop()

async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    mydrone.msp_override_channels=[1, 2, 3, 4, 13]
    try:
        await mydrone.connect()
        print("Connected to the flight controller")

        flight_control_task = asyncio.create_task(mydrone.flight_loop())
        user_script_task = asyncio.create_task(telemetry_display(mydrone))

        await asyncio.gather(flight_control_task, user_script_task)
    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
