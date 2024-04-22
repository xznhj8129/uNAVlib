import asyncio
import time
from unavlib.tools.control import UAVControl
from unavlib.modules import geo

async def my_plan(uav):

    uav.debugprint = False
    uav.modes.keys() # show all currently programmed modes

    # create new supermode (combination of multiple modes)
    #uav.new_supermode('DIVE', ["NAV ALTHOLD", "NAV COURSE HOLD"])
    uav.new_supermode('GOTO', ["GCS NAV", "NAV POSHOLD"])

    # assuming proper compile flag and bitmask config
    uav.set_mode("MSP RC OVERRIDE", on=True)
    uav.arm_enable_check()
    uav.arm()
    await asyncio.sleep(1)

    #takeoff
    print('takeoff')
    uav.set_rc_channel('throttle',2000)
    uavspeed = uav.get_gps_data()['speed']
    uavalt = uav.get_altitude()
    while uavalt<5:
        uav.set_rc_channel('throttle',2000)
        uav.set_rc_channel('pitch',1000)
        uavspeed = uav.get_gps_data()['speed']
        uavalt = uav.get_altitude()
        print(uavspeed,uavalt)
        await asyncio.sleep(1)

    uav.set_rc_channel('pitch',1500)
    await asyncio.sleep(5)

    uav.set_supermode("GOTO", on=True)
    await asyncio.sleep(2)
    wp = geo.GPSposition(45.477178, -73.799529, 20)
    uav.set_wp(255, 1, wp.lat, wp.lon, 100, 0, 0, 0, 0)

    gpsd = uav.get_gps_data()
    alt = uav.get_altitude()
    pos = geo.GPSposition(gpsd['lat'], gpsd['lon'], alt)
    vector = geo.gps_to_vector(pos, wp)

    while vector.dist>1000:
        gpsd = uav.get_gps_data()
        speed = gpsd['speed']
        alt = uav.get_altitude()
        pos = geo.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        vector = geo.gps_to_vector(pos, wp)

        print('\n\n')
        print('Modes:', uav.board.CONFIG['mode'], uav.board.process_mode(uav.board.CONFIG['mode']))
        print('Active modes:', uav.get_active_modes())
        print('GPS data:', uav.get_gps_data())
        print('Attitude:', uav.get_attitude())
        print('Altitude:', alt)
        print('Vector to waypoint:', vector)
        await asyncio.sleep(1)

    #print(uav.board.AUX_CONFIG)
    #print(uav.board.process_mode(uav.board.CONFIG['mode'])) # why does this not show active modes?
    #print(uav.active_modes)
    print('Active modes:', uav.get_active_modes())

    #uav.arm_enable_check()
    #uav.arm()
    #await asyncio.sleep(10)

    print(f"Navstatus: {uav.get_nav_status()}")
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
