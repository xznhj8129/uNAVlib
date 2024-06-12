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


## class UAVControl
Wraps the main MSPy board connection and implements features to facilitate programming

#### Important to note:
* The generated source enums are read at class initialization and set as an attribute for lookup, such as uav.board.accelerationSensor; as well as a reverse integer-indexed dict with the prefix R, such as uav.board.R_accelerationSensor.
* When returned from uavcontrol functions, MSP message codes are parsed to their string representation from the enums classes and their reverse lookup. For example when getting sensor configs, the value 3 will be indexed to R_accelerationSensor to return "ACC_MPU6500".
* Not all enums are implemented yet, and their usage/placement might change


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
| uavcontrol.get_sensor_config | Requests, parses, returns sensors configuration | | dict: {<br>'acc_hardware': inav_enums.accelerationSensor,<br> 'baro_hardware': inav_enums.baroSensor,<br> 'mag_hardware': inav_enums.magSensor,<br>'pitot': inav_enums.pitotSensor,<br>'rangefinder': inav_enums.rangefinderType,<br> 'opflow': inav_enums.opticalFlowSensor<br>} |

uavcontrol.run_sync(func, *args)
 
uavcontrol.std_send(str msg_code, data=[])
 
uavcontrol.add_pid(str pidname, Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0, output_limits=(0, 1))
 
uavcontrol.update_pid(str pidname, float error)
 
uavcontrol.load_modes_config()
 
uavcontrol.load_modes_config_file(mode_config_file)
 
uavcontrol.arm_enable_check()
 
uavcontrol.arm()
 
uavcontrol.set_mode(str mode, bool on)
 
uavcontrol.get_active_modes()
 
uavcontrol.new_supermode(str name, list modes)
 
uavcontrol.set_supermode(str supermode, bool on)
 
uavcontrol.pwm_clamp(int n)
 
uavcontrol.set_rc_channel(str ch, int value)
 
uavcontrol.get_attitude()
 
uavcontrol.get_altitude()
 
uavcontrol.get_imu()
 
uavcontrol.get_nav_status()
 
uavcontrol.get_gps_data()
 
uavcontrol.set_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag)
 