import asyncio
import time
from unavlib.tools.control import UAVControl
from unavlib.modules import geospatial

# Example Mission and use of UAVControl class
# Work in progress
# Uses X-Plane 11 HITL starting from Montreal Intl Airport
# Start, arm and set mode to Override, then run this
# Auto-takeoff is sketchy so just be ready to prevent it from stalling and switch back to override

async def my_plan(uav):

    uav.debugprint = False
    uav.modes.keys() # show all currently programmed modes

    # create new supermode (combination of multiple modes)
    uav.new_supermode('GOTO', ["GCS NAV", "NAV POSHOLD"])

    # assuming proper compile flag and bitmask config
    uav.set_mode("MSP RC OVERRIDE", on=True)
    uav.arm_enable_check()
    uav.arm()
    await asyncio.sleep(1)

    #takeoff (crash)
    print('takeoff')
    uav.set_rc_channel('throttle',2000)
    uavspeed = uav.get_gps_data()['speed']
    uavalt = uav.get_altitude()
    while uavalt<2:
        uav.set_rc_channel('throttle',2000)
        uav.set_rc_channel('pitch',1000)
        uavspeed = uav.get_gps_data()['speed']
        uavalt = uav.get_altitude()
        print(uavspeed,uavalt)
        await asyncio.sleep(1)

    uav.set_rc_channel('pitch',1500)
    await asyncio.sleep(3)

    # set waypoint and fly auto
    uav.set_supermode("GOTO", on=True)
    await asyncio.sleep(2)
    wp = geospatial.GPSposition(45.487363, -73.812242, 20)
    uav.set_wp(255, 1, wp.lat, wp.lon, 100, 0, 0, 0, 0)
    print(f"Navstatus: {uav.get_nav_status()}")

    gpsd = uav.get_gps_data()
    alt = uav.get_altitude()
    pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
    vector = geospatial.gps_to_vector(pos, wp)
    while vector.dist>50:
        gpsd = uav.get_gps_data()
        speed = gpsd['speed']
        alt = uav.get_altitude()
        pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        vector = geospatial.gps_to_vector(pos, wp)
        gyro = uav.get_attitude()

        print('\n')
        print('Modes:', uav.board.CONFIG['mode'], uav.board.process_mode(uav.board.CONFIG['mode']))
        print('Active modes:', uav.get_active_modes())
        print('Position:', pos)
        print('Attitude:', gyro)
        print('Altitude:', alt)
        print('Vector to waypoint:', vector)
        print('Bearing:',vector.az - gyro['yaw'])
        await asyncio.sleep(1)

    # loiter a minute
    for i in range(60):
        print(60-i)
        await asyncio.sleep(1)

    # bring it back
    uav.set_supermode("GOTO", on=False)
    uav.set_mode("NAV RTH", on=True)

    # rth and land
    while alt > 1:
        gpsd = uav.get_gps_data()
        alt = uav.get_altitude()
        print('Distance to home:',gpsd['distanceToHome'])
        await asyncio.sleep(1)

    uav.stop()


async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200)
    mydrone.msp_override_channels=[1, 2, 3, 4, 12, 13, 14]

    try:
        await mydrone.connect()
        print("Connected to the flight controller")
        flight_control_task = asyncio.create_task(mydrone.flight_loop())
        user_script_task = asyncio.create_task(my_plan(mydrone))
        await asyncio.gather(flight_control_task, user_script_task)
    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
