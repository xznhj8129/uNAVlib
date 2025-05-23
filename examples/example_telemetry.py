import asyncio
import socket
import time
from unavlib.control import UAVControl
from unavlib.modules import geospatial
from unavlib.modules.utils import inavutil

async def telemetry_display(uav):
    uav.debugprint = True
    print('start telemetry')
    while 1:
        print('telemetry')
        gpsd = uav.get_gps_data()
        speed = gpsd['speed']
        alt = uav.get_altitude()
        pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        gyro = uav.get_attitude()
        print('\n')
        print("Channels:",uav.channels)
        print('Modes:', uav.board.CONFIG['mode'], uav.get_active_modes())
        print('Position:', pos)
        print('Attitude:', gyro)
        print('Altitude:', alt)
        print(f"Navstatus: {uav.get_nav_status()}")

        await asyncio.sleep(0.5)
    uav.stop()

async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    mydrone.msp_receiver = False
    
    try:
        await mydrone.connect()
        print("Connected to the flight controller")

        flight_control_task = asyncio.create_task(mydrone.flight_loop())
        user_script_task = asyncio.create_task(telemetry_display(mydrone))

        await asyncio.gather(flight_control_task, user_script_task)
    except asyncio.CancelledError:
        # clean-up if needed
        raise
    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
