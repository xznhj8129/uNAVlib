# example_telemetry.py

import asyncio
import traceback
from unavlib.control.uavcontrol import UAVControl
from unavlib.modules.utils import inavutil
from unavlib.modules import geospatial

async def telemetry_display(uav):
    uav.debugprint = True
    uav.load_modes_config(echo=True)
    try:
        await asyncio.sleep(1)
        while True:
            print('')
            print('#'*54)
            alt  = await uav.get_altitude()
            gpsd = await uav.get_gps_data()
            if gpsd:
                speed = gpsd['speed']
                pos   = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
                print('Speed:', speed)
                print('Position:', pos)
            else:
                print("No GPS data yet")
            gyro  = await uav.get_attitude()
            nav   = await uav.get_nav_status()

            channels = await uav.get_rc_channels()
            am = uav.get_active_modes()
            #bm = await uav.get_board_modes()
            print("Channels:", channels)
            print('Modes:', am)#, bm)
            print('Modes:', [inavutil.modesID_INAV.get(i) for i in am])
            #print('Modes:', [inavutil.modesID_INAV.get(i) for i in bm])
            print('Attitude:', gyro)
            print('Altitude:', alt)
            print(f"Navstatus: {nav}")
            print('Override:',uav.is_override_active())

            if inavutil.modesID_INAV.FAILSAFE in bm:
                print("FAILSAFE!")
            

            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        print("Telemetry display cancelled")

async def main():
    mydrone = UAVControl(device='/dev/ttyACM0', baudrate=115200, platform="COPTER")
    mydrone.msp_receiver = False

    try:
        await mydrone.connect()
        print("Connected to the flight controller")

        fc_task  = asyncio.create_task(mydrone.flight_loop())
        tel_task = asyncio.create_task(telemetry_display(mydrone))

        await asyncio.gather(fc_task, tel_task)
    except Exception:
        print("Error in main:")
        traceback.print_exc()
    finally:
        await mydrone.disconnect()
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
