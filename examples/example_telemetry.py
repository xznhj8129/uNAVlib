# example_telemetry.py
# Only this user-side script is async; all UAVControl calls remain blocking.
import asyncio
import traceback
import time

from unavlib.control.uavcontrol import UAVControl
from unavlib.modules.utils import inavutil
from unavlib.modules import geospatial


def telemetry_display_sync(uav: UAVControl):
    """Blocking telemetry loop run inside a single background thread."""
    uav.debugprint = True
    uav.load_modes_config(echo=True)
    time.sleep(1)

    while True:
        alt = uav.get_altitude()
        if uav.gps_enabled:
            gpsd = uav.get_gps_data()
            if gpsd:
                speed = gpsd["speed"]
                pos = geospatial.GPSposition(gpsd["lat"], gpsd["lon"], alt)
            nav = uav.get_nav_status()
            print(f"Nav: {nav}")

        gyro = uav.get_attitude()

        channels = uav.get_rc_channels()
        am = uav.get_active_modes()
        override = uav.is_override_active()

        print(f"Gyro: {gyro}")
        print(f"Channels: {channels}")
        if override:
            print("OVERRIDE")

        if inavutil.modesID_INAV.FAILSAFE in am:
            print("FAILSAFE!")

        time.sleep(0.1)


async def main():
    mydrone = UAVControl(
        device="/dev/ttyACM0", 
        baudrate=115200, 
        platform="COPTER", 
        receiver="serial", 
        GPS=False, 
        msp_override_ch=[1,2,3,4]
    )

    try:
        # Blocking connect
        await asyncio.to_thread(mydrone.connect)
        print("Connected to the flight controller")

        # Run both the FC loop and telemetry loop in separate threads
        fc_task = asyncio.to_thread(mydrone.flight_loop)
        tel_task = asyncio.to_thread(telemetry_display_sync, mydrone)

        await asyncio.gather(fc_task, tel_task)
    except Exception:
        print("Error in main:")
        traceback.print_exc()
    finally:
        await asyncio.to_thread(mydrone.disconnect)
        print("\nConnection closed")


if __name__ == "__main__":
    asyncio.run(main())
