# Welcome to the uNAVlib wiki!
Here you will find reference on how to use this library and documentation on the various functions.

## This is a work in progress and things may change quickly 

## Simple example
```
import asyncio
import time
from unavlib.control import UAVControl

async def my_plan(uav):
    uav.arm_enable_check()
    uav.set_mode("ARM",on=True)
    await asyncio.sleep(1)
    print(uav.get_gps_data())
    await asyncio.sleep(1)
    uav.stop()

async def main():
    mydrone = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    mydrone.msp_override_channels=[1, 2, 3, 4, 5]

    try:
        await mydrone.connect()
        print("Connected to the flight controller")
        flight_control_task = asyncio.create_task(mydrone.flight_loop())
        user_script_task = asyncio.create_task(my_plan(mydrone))
        await asyncio.gather(flight_control_task, user_script_task)
    finally:
        print('Connection closed')

if __name__ == '__main__':
    asyncio.run(main())
```

### Accessing MSP and INAV enums
```
>>> from unavlib.modules.utils import inavutil
>>> inavutil.msp.MSP_BOARD_INFO
4
>>> inavutil.msp.get(4)
'MSP_BOARD_INFO'
>>> inavutil.modesID.ANGLE
1
>> inavutil.navSystemStatus_State.get(5)
'MW_NAV_STATE_WP_ENROUTE'
```

### Accessing flight controller data
Some functions have been implemented in the main uavcontrol class that send the appropriate MSP messages and parses them
```
# in a flight script like the example mission above

async def my_plan(uav):
    print(uav.get_gps_data())
```
Which executes
```
# in uavcontrol.py

    def get_gps_data(self):
        a = self.std_send(inavutil.msp.MSP_RAW_GPS)
        b = self.std_send(inavutil.msp.MSP_COMP_GPS)
        c = self.std_send(inavutil.msp.MSP_GPSSTATISTICS)
        if not a or not b or not c:
            return None
        else:
            ret = self.board.GPS_DATA
            ret['lat'] = ret['lat'] / 1e7
            ret['lon'] = ret['lon'] / 1e7
            ret['speed'] = ret['speed'] / 100
            return ret
```
which sends message MSP_RAW_GPS to the flight controller which is parsed like this:
```
# in modules.process.py

    def process_MSP_RAW_GPS(self, data):
        self.board.GPS_DATA['fix'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_DATA['numSat'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_DATA['lat'] = self.board.readbytes(data, size=32, unsigned=False)
        self.board.GPS_DATA['lon'] = self.board.readbytes(data, size=32, unsigned=False)
        self.board.GPS_DATA['alt'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['speed'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['ground_course'] = self.board.readbytes(data, size=16, unsigned=True)

        if self.board.INAV:
            self.board.GPS_DATA['hdop'] = self.board.readbytes(data, size=16, unsigned=True)
```

Otherwise, messages received will have the basic implementation from YAMSPy handled by a named function in process.py that is given the MSP data from that message from the connection functions, like process_MSP_RAW_GPS above. It sets variables (always capitalized) in uav.board with the parsed data, such as this:
```
def process_MSP_ATTITUDE(self, data):
    self.board.SENSOR_DATA['kinematics'][0] = self.board.readbytes(data, size=16, unsigned=False) / 10.0 # x
    self.board.SENSOR_DATA['kinematics'][1] = self.board.readbytes(data, size=16, unsigned=False) / 10.0 # y
    self.board.SENSOR_DATA['kinematics'][2] = self.board.readbytes(data, size=16, unsigned=False) # z
```
You can directly access the data, but they are meant as read-only holders for data received from the FC.
Those functions are the primary way that MSP messages are implemented.

## Class UAVControl
Wraps the main MSPy board connection and implements features to facilitate programming


### Variables
<table>
<tr>
    <td> Name </td> <td> Description </td> <td> Structure </td>
</tr>
<tr>
    <td> self.modes </td>
    <td> Flight modes as configured in INAV </td>
    <td>

    {
        "ANGLE": [6,[1300,1700]],
        "ARM": [5,[1800,2100]],
        "FAILSAFE": [10,[1600,2100]],
        "MANUAL": [6,[1700,2100]],
        "NAV ALTHOLD": [7,[1300,1700]]
    }

</td>
</tr>
<tr>
    <td> self.modes </td>
    <td> Flight modes as configured in INAV </td>
    <td>

    {
        "ANGLE": [6,[1300,1700]],
        "ARM": [5,[1800,2100]],
        "FAILSAFE": [10,[1600,2100]],
        "MANUAL": [6,[1700,2100]],
        "NAV ALTHOLD": [7,[1300,1700]]
    }

</td>
</tr>
</table>


### Functions:
| Function | Description | Arguments | Returns |
| --- | --- | --- | --- |
| uavcontrol.connect | Connects to board at specified address | |  |
| uavcontrol.disconnect | Safely disconnects board | | bool: self.connected |
| uavcontrol.flight_loop | Starts main async control loop | |  |
| uavcontrol.stop | Stops main async control loop | |  |
| uavcontrol.load_modes_config | Polls Flight Controller for AUX modes, prints them and loads as dict into self.modes| |
| uavcontrol.get_sensor_config | Requests, parses, returns sensors configuration | |json
``` {'acc_hardware': inavutil.accelerationSensor,<br> 'baro_hardware': inavutil.baroSensor,<br> 'mag_hardware': inavutil.magSensor,<br>'pitot': inavutil.pitotSensor,<br>'rangefinder': inavutil.rangefinderType,<br> 'opflow': inavutil.opticalFlowSensor<br>}``` |
| uavcontrol.run_sync(func, *args) | | |  |
| uavcontrol.std_send(str msg_code, data=[]) | | |  |
| uavcontrol.add_pid(str pidname, Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0, output_limits=(0, 1)) | | |  |
| uavcontrol.update_pid(str pidname, float error) | | |  |
| uavcontrol.load_modes_config() | | |  |
| uavcontrol.load_modes_config_file(mode_config_file) | | |  |
| uavcontrol.arm_enable_check() | | |  |
| uavcontrol.arm() | | |  |
| uavcontrol.set_mode(str mode, bool on) | | |  |
| uavcontrol.get_active_modes() | | |  |
| uavcontrol.new_supermode(str name, list modes) | | |  |
| uavcontrol.set_supermode(str supermode, bool on) | | |  |
| uavcontrol.pwm_clamp(int n) | | |  |
| uavcontrol.set_rc_channel(str ch, int value) | | |  |
| uavcontrol.get_attitude() | | |  |
| uavcontrol.get_altitude() | | |  |
| uavcontrol.get_imu() | | |  |
| uavcontrol.get_nav_status() | | |  |
| uavcontrol.get_gps_data() | | |  |
| uavcontrol.set_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag) | | |  |
| uavcontrol._set_wp(wp_no, wp) | | |  |
| uavcontrol.pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag) ||||
| uavcontrol.get_wp(wp_no) ||||
| uavcontrol.get_wp_info() |||dict: {"reserved": self.board.WP_INFO["reserved"], "nav_max_waypoints": self.board.WP_INFO["NAV_MAX_WAYPOINTS"], "isWaypointListValid": self.board.WP_INFO["isWaypointListValid"] ,