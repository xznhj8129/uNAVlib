import asyncio
import time
from unavlib.control.uavcontrol import UAVControl
from unavlib.modules import geospatial
from unavlib.modules.utils import inavutil

async def my_plan(uav):
    set_alt = 50
    uav.debugprint = False

    # create a combined supermode and enable overrides
    uav.new_supermode('GOTO', [inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD])
    uav.set_mode(inavutil.modesID.MSP_RC_OVERRIDE, on=True)
    uav.set_mode(inavutil.modesID.ANGLE, on=True)

    # arm sequence
    uav.arm_enable_check()
    await asyncio.sleep(1)
    uav.set_mode(inavutil.modesID.ARM, on=True)
    await asyncio.sleep(1)

    # takeoff
    print('takeoff')
    uav.set_rc_channel('throttle', 2000)
    await asyncio.sleep(0.1)  # let FC process throttle change
    uavalt = await uav.get_altitude()
    t = 0
    while uavalt < set_alt:
        uav.set_rc_channel('throttle', 2100)
        uav.set_rc_channel('pitch',    1100)
        gpsd   = await uav.get_gps_data()
        uavspeed = gpsd.get('speed', 0)
        uavalt   = await uav.get_altitude()
        print('Speed:', uavspeed, 'Alt:', uavalt)
        await asyncio.sleep(1)
        t += 1
        if t > 30:
            print('Aborting')
            uav.stop()
            return 1

    await asyncio.sleep(3)

    # climb complete, switch to loiter then GOTO
    uav.set_rc_channel('pitch', 1500)
    uav.set_supermode("GOTO", on=True)
    # wait until GCS_NAV is active
    while inavutil.modesID.GCS_NAV not in uav.get_active_modes():
        await asyncio.sleep(1)

    # upload a waypoint and start navigation
    wp = geospatial.GPSposition(45.487363, -73.812242, set_alt)
    await uav.set_wp(255, 1, wp.lat, wp.lon, wp.alt, 0, 0, 0, 0)
    nav = await uav.get_nav_status()
    print(f"Navstatus: {nav}")

    # fly to waypoint
    while True:
        gpsd = await uav.get_gps_data()
        alt  = await uav.get_altitude()
        pos  = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        vector = geospatial.gps_to_vector(pos, wp)
        if vector.dist <= 50:
            break

        gyro = await uav.get_attitude()
        print('\n')
        print('Channels:', await uav.get_rc_channels())
        print('Active modes:', uav.get_active_modes())
        print('Position:', pos)
        print('Attitude:', gyro)
        print('Altitude:', alt)
        print('Vector to waypoint:', vector)
        print('Bearing:', vector.az - gyro['yaw'])
        await asyncio.sleep(1)

    # loiter one minute
    for i in range(60, 0, -1):
        print(f"Loiter: {i}s")
        await asyncio.sleep(1)

    # return home
    uav.set_supermode("GOTO", on=False)
    uav.set_mode(inavutil.modesID.NAV_RTH, on=True)

    alt = await uav.get_altitude()
    while alt > 1:
        gpsd = await uav.get_gps_data()
        dist = gpsd.get('distanceToHome', None)
        print('Distance to home:', dist)
        await asyncio.sleep(1)
        alt = await uav.get_altitude()

    uav.stop()

async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    mydrone.msp_override_allowed_ch = [1,2,3,4,5,6,12,13,14]
    mydrone.msp_receiver = True

    try:
        await mydrone.connect()
        print("Connected to the flight controller")
        flight_task = asyncio.create_task(mydrone.flight_loop())
        mission_task = asyncio.create_task(my_plan(mydrone))
        await asyncio.gather(flight_task, mission_task)
    finally:
        await mydrone.disconnect()
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
