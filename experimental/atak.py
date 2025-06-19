import asyncio
import time
import datetime
import pytz
import pytak
from unavlib.control import UAVControl
from unavlib.control import geospatial
import xml.etree.ElementTree as ET
from configparser import ConfigParser
import multiprocessing
from multiprocessing import Manager


# Example Mission and use of UAVControl class
# Work in progress
# Uses X-Plane 11 HITL starting from Montreal Intl Airport
# set msp_override_channels =  14399
# Use receiver type MSP, set serialport below to your telemetry port (ie: USB-TTL converter)
# Mode settings:
# ID: 0 	ARM             :	0 (channel 5)	= 1800 to 2100
# ID: 1	    ANGLE           :	7 (channel 12)	= 900 to 1100
# ID: 11	NAV POSHOLD     :	7 (channel 12)	= 1100 to 1300
# ID: 10	NAV RTH         :	7 (channel 12)	= 1900 to 2100
# ID: 31	GCS NAV         :	7 (channel 12)	= 1100 to 1300
# ID: 27	FAILSAFE        :	4 (channel 9)	= 1600 to 2100
# ID: 50	MSP RC OVERRIDE :	1 (channel 6)	= 1400 to 1625
# Start, arm and set mode to Override, then run this

def cot_time(offset_seconds=0):
    """
    Generates a COT formatted timestamp offset by a given number of seconds from now.
    """
    return (datetime.datetime.utcnow() + datetime.timedelta(seconds=offset_seconds)).replace(tzinfo=pytz.utc).isoformat()

def gen_cot(lat,lon,alt):
    global ang
    cot = ET.Element('event')
    cot.set('version', '2.0')
    cot.set('uid', '000000001-UAV')
    cot.set('type', 'a-f-G-U-C')
    cot.set('time', pytak.cot_time())
    cot.set('start', pytak.cot_time())
    cot.set('stale', cot_time(60))
    cot.set('how', 'm-g')
    
    point = ET.SubElement(cot, 'point')
    point.set('lat', str(lat))  # Set latitude
    point.set('lon', str(lon))  # Set longitude
    point.set('hae', str(alt))  # Altitude
    point.set('ce', '9999999')
    point.set('le', '9999999')

    detail = ET.SubElement(cot, 'detail')
    contact = ET.SubElement(detail, 'contact')
    contact.set("endpoint", "*:-1:stcp")
    contact.set('callsign', 'UAV123')
    
    group = ET.SubElement(detail, '__group')
    group.set('role', 'Team Member')
    group.set('name', 'Cyan')

    status = ET.SubElement(detail, 'status')
    status.set('battery', '90')  # Example battery level

    return ET.tostring(cot)

def shared_data():
    manager = Manager()
    return manager.dict()


class MySender(pytak.QueueWorker):
    def __init__(self, tx_queue, config, shared_data):
        super().__init__(tx_queue, config)
        self.shared_data = shared_data

    async def run(self):
        while True:
            print(self.shared_data)
            if 'pos' in self.shared_data:
                pos = self.shared_data['pos']
                data = gen_cot(pos[0],pos[1],pos[2])
                if data:
                    self._logger.info("Sending:\n%s\n", data.decode())
                    await self.handle_data(data)
            await asyncio.sleep(5)

    async def handle_data(self, data):
        await self.put_queue(data)

class MyReceiver(pytak.QueueWorker):

    async def handle_data(self, data):
        self._logger.info("Received:\n%s\n", data.decode())

    async def run(self):  # pylint: disable=arguments-differ
        while 1:
            data = (await self.queue.get())  # this is how we get the received CoT from rx_queue
            await self.handle_data(data)



async def my_plan(uav, shared):

    uav.debugprint = False
    #uav.modes.keys() # show all currently programmed modes
    # create new supermode (combination of multiple modes)
    uav.new_supermode('GOTO', ["GCS NAV", "NAV POSHOLD"])
    # assuming proper compile flag and bitmask config
    uav.set_mode("MSP RC OVERRIDE", on=True)
    uav.set_mode("ANGLE", on=True)
    #await asyncio.sleep(3)
    uav.arm_enable_check()
    await asyncio.sleep(1)
    uav.set_mode("ARM", on=True)
    await asyncio.sleep(1)

    #takeoff
    print('takeoff')
    uav.set_rc_channel('throttle',2000)
    uavalt = uav.get_altitude()
    t=0
    while uavalt<2:
        uav.set_rc_channel('throttle',2000)
        uav.set_rc_channel('pitch',1300)
        uavspeed = uav.get_gps_data()['speed']
        uavalt = uav.get_altitude()
        print('Speed:', uavspeed,'Alt:', uavalt)
        await asyncio.sleep(1)
        t+=1
        if t>30:
            print('Aborting')
            uav.stop()
            return 1

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
        shared['pos'] = [gpsd['lat'], gpsd['lon'], alt]
        speed = gpsd['speed']
        alt = uav.get_altitude()
        pos = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
        vector = geospatial.gps_to_vector(pos, wp)
        gyro = uav.get_attitude()

        print('\n')
        #print('Modes:', uav.board.CONFIG['mode'], uav.board.process_mode(uav.board.CONFIG['mode']))
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
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    mydrone.msp_override_channels=[1, 2, 3, 4, 5, 6, 12, 13, 14]

    try:
        await mydrone.connect()
        ipc = shared_data()
        print("Connected to the flight controller")
        flight_control_task = asyncio.create_task(mydrone.flight_loop())
        user_script_task = asyncio.create_task(my_plan(mydrone, ipc))

        # Setup for PyTAK communication
        config = ConfigParser()
        config["mycottool"] = {"COT_URL": "tcp://127.0.0.1:8087"}
        config = config["mycottool"]

        # Initializes worker queues and tasks for PyTAK
        clitool = pytak.CLITool(config)
        await clitool.setup()

        # Create instances of MySender and MyReceiver, ensuring they are using the correct queues from clitool
        sender = MySender(clitool.tx_queue, config, ipc)
        receiver = MyReceiver(clitool.rx_queue, config)  

        # Create tasks for sender and receiver
        sender_task = asyncio.create_task(sender.run())
        receiver_task = asyncio.create_task(receiver.run())

        # Start all tasks.
        await asyncio.gather(flight_control_task, user_script_task, sender_task, receiver_task)

    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
    

