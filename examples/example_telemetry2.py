# Simpler example not using the flight loop
import asyncio
import socket
import time
from collections import deque
from itertools import cycle 
from unavlib.control import UAVControl
from unavlib.control import geospatial
from unavlib.modules.utils import inavutil
from unavlib.modules.fast_functions import fastMSP
from unavlib.modules import geospatial
from unavlib import MSPy

async def main():
    loop_time = 1/10.0

    mydrone = UAVControl(
        device='/dev/ttyUSB0', 
        baudrate=115200, 
        platform="AIRPLANE"
        )
    print("Connected to the flight controller")
    mydrone.msp_receiver = False
    mydrone.debugprint = False
    
    try:
        await mydrone.connect()
        await mydrone.telemetry_init()
        mydrone.load_modes_config()

        while 1:
            t = time.time()

            #if mydrone.board.send_RAW_RC(mydrone.channels):
            #    dataHandler = mydrone.board.receive_msg()
            #    mydrone.board.process_recv_data(dataHandler)

            gpsd = mydrone.get_gps_data()
            speed = gpsd['speed']
            alt = mydrone.get_altitude()
            pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
            gyro = mydrone.get_attitude()
            navstatus = mydrone.get_nav_status()

            print('\n')
            print("Channels:",mydrone.channels)
            print('Modes:', mydrone.board.CONFIG['mode'], mydrone.get_board_modes())
            print('Position:', pos)
            print('Attitude:', gyro)
            print('Altitude:', alt)
            print(f"Navstatus: {navstatus}")
            print(f"cpuload: {mydrone.board.CONFIG['cpuload']}")
            print(f"cycleTime: {mydrone.board.CONFIG['cycleTime']}")
            print(mydrone.get_board_modes())

            await asyncio.sleep(loop_time)

        mydrone.stop()
        

    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
