import asyncio
import time
from unavlib.tools.control import UAVControl

async def my_plan(uav):
    uav.get_gps_data()
    #await asyncio.sleep(1)
    navstatus = uav.get_nav_status()
    #print(f"Navstatus: {navstatus}")
    #await asyncio.sleep(1)
    print('imu:',uav.board.fast_read_imu())
    print('att:',uav.board.fast_read_attitude()) 
    print('alt:',uav.board.fast_read_altitude())
    uav.stop()


async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200)
    mydrone.msp_override_channels=[1, 2, 3, 4, 7, 12]

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
