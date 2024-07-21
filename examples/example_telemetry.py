import asyncio
import socket
import time
from unavlib.control import UAVControl
from unavlib.control import geospatial

async def telemetry_display(uav):
    #uav.debugprint = True
    print(uav.board.AUX_CONFIG)
    while 1:
        gpsd = uav.get_gps_data()
        speed = gpsd['speed']
        alt = uav.get_altitude()
        pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        gyro = uav.get_attitude()
        print('\n')
        print("Channels:",uav.channels)
        print('Modes:', uav.get_active_modes())
        print('Position:', pos)
        print('Attitude:', gyro)
        print('Altitude:', alt)

        await asyncio.sleep(1)
    uav.stop()

async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")

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
