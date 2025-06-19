
import math
from unavlib.control import UAVControl
from unavlib.modules import geospatial
from unavlib.modules.utils import inavutil
from pymavlink import mavutil
import time
import inspect 
import traceback
import random
import asyncio

# https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py

def dict_index(d, target_value):
    return [key for key, value in d.items() if value == target_value]

def dict_reverse(d):
    return {v: i for i,v in d.items()}


class Telemetry:
    def __init__(self, mavctl, master, start_time):
        self.mavconn = master
        self.inavctl = None
        self.mavctl = mavctl
        self.start_time = start_time

        # Data stream rates (in Hz)
        self.stream_rates = {
            'HEARTBEAT': 2,
            'ATTITUDE': 10,
            'MISSION_CURRENT': 1,
            'GPS_RAW_INT': 5,
            'GLOBAL_POSITION_INT': 5,
            'GLOBAL_POSITION_ORIGIN': 2,
            'RC_CHANNELS': 10,
            'BATTERY_STATUS': 2,
            'SCALED_PRESSURE': 0.2,
            'VFR_HUD': 3,
            'SYS_STATUS': 1,
            #'EXTRA1': 3,
            #'EXTRA2': 2,
            #'EXTRA3': 1,
        }

        # Track last sent times for different messages
        self.last_gcs_heartbeat = 0
        self.last_msg_time = {key: 0 for key in self.stream_rates}

        # Define the functions associated with each telemetry stream
        self.stream_functions = {
            'HEARTBEAT': self.send_heartbeat,
            'ATTITUDE': self.send_attitude,
            'MISSION_CURRENT': self.send_mission_current,
            'GPS_RAW_INT': self.send_gps_raw_int,
            'GLOBAL_POSITION_INT': self.send_global_position_int,
            'GLOBAL_POSITION_ORIGIN': self.send_global_position_origin,
            'RC_CHANNELS': self.send_rc_channels,
            'BATTERY_STATUS': self.send_battery_status,
            'SCALED_PRESSURE': self.send_scaled_pressure,
            'VFR_HUD': self.send_vfr_hud,
            'SYS_STATUS': self.send_sys_status,
        }

    def send(self, current_time):
        """Iterate over the telemetry streams and send messages if the interval has passed."""
        for stream_name, rate in self.stream_rates.items():
            if current_time - self.last_msg_time[stream_name] >= (1.0 / rate):
                self.stream_functions[stream_name]()
                self.last_msg_time[stream_name] = current_time

    def send_heartbeat(self):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.mavctl.armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        self.mavconn.mav.heartbeat_send(
            type=self.mavctl.type,
            autopilot=self.mavctl.autopilot,
            base_mode=base_mode,
            custom_mode=self.mavctl.mode_id,
            system_status=self.mavctl.system_status
        )

    def send_attitude(self):
        self.mavconn.mav.attitude_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            roll=self.mavctl.attitude['roll'],
            pitch=self.mavctl.attitude['pitch'],
            yaw=self.mavctl.attitude['yaw'],
            rollspeed=self.mavctl.attitude['rollspeed'],
            pitchspeed=self.mavctl.attitude['pitchspeed'],
            yawspeed=self.mavctl.attitude['yawspeed']
        )

    def send_gps_raw_int(self):
        self.mavconn.mav.gps_raw_int_send(
            time_usec=int(time.time() * 1000000),
            fix_type=3,
            lat=int(self.mavctl.position.lat * 1e7),
            lon=int(self.mavctl.position.lon * 1e7),
            alt=int(self.mavctl.position.alt * 1e3),
            eph=100,
            epv=100,
            vel=0,
            cog=0,
            satellites_visible=10
        )

    def send_mission_current(self):
        self.mavconn.mav.mission_current_send(
            seq=self.mavctl.current_mission_seq,
        )

    def send_battery_status(self):
        self.mavconn.mav.battery_status_send(
            id=0,
            battery_function=mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
            type=mavutil.mavlink.MAV_BATTERY_TYPE_LIPO, # can change
            temperature=int(self.mavctl.battery.get('temperature', 25.0) * 100),
            voltages=[int(self.mavctl.battery.get('voltage', 11.1) * 1000)] * 10,
            current_battery=int(self.mavctl.battery.get('current', 1.0) * 100),
            current_consumed=self.mavctl.battery.get('mahdrawn', 100),
            energy_consumed=self.mavctl.battery.get('mwhdrawn', 100),
            battery_remaining=self.mavctl.battery.get('remaining', 100)
        )

    def send_global_position_int(self):
        self.mavconn.mav.global_position_int_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            lat=int(self.mavctl.position.lat * 1e7),
            lon=int(self.mavctl.position.lon * 1e7),
            alt=int(self.mavctl.position.alt * 1e3),
            relative_alt=int(self.mavctl.position.alt * 1e3),
            vx=0, vy=0, vz=0,
            hdg=0
        )

    def send_global_position_origin(self):
        self.mavconn.mav.set_gps_global_origin_send(
            target_system=self.mavconn.target_system,
            latitude=int(self.mavctl.global_position_origin.lat * 1e7),
            longitude=int(self.mavctl.global_position_origin.lon * 1e7),
            altitude=int(self.mavctl.global_position_origin.alt * 1e3)
        )

    def send_rc_channels(self):
        self.mavconn.mav.rc_channels_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            chan1_raw=self.mavctl.rc_channels[0],
            chan2_raw=self.mavctl.rc_channels[1],
            chan3_raw=self.mavctl.rc_channels[2],
            chan4_raw=self.mavctl.rc_channels[3],
            chan5_raw=self.mavctl.rc_channels[4],
            chan6_raw=self.mavctl.rc_channels[5],
            chan7_raw=self.mavctl.rc_channels[6],
            chan8_raw=self.mavctl.rc_channels[7],
            chan9_raw=self.mavctl.rc_channels[8],
            chan10_raw=self.mavctl.rc_channels[9],
            chan11_raw=self.mavctl.rc_channels[10],
            chan12_raw=self.mavctl.rc_channels[11],
            chan13_raw=self.mavctl.rc_channels[12],
            chan14_raw=self.mavctl.rc_channels[13],
            chan15_raw=self.mavctl.rc_channels[14],
            chan16_raw=self.mavctl.rc_channels[15],
            chan17_raw=0, chan18_raw=0,
            chancount=16,
            rssi=self.mavctl.rssi
        )

    def send_scaled_pressure(self):
        self.mavconn.mav.scaled_pressure_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            press_abs=self.mavctl.scaled_pressure.get('press_abs', 1013.25),
            press_diff=self.mavctl.scaled_pressure.get('press_diff', 0.0),
            temperature=int(self.mavctl.scaled_pressure.get('temperature', 20.0) * 100)
        )

    def send_sys_status(self):
        self.mavconn.mav.sys_status_send(
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            load=500,
            voltage_battery=int(self.mavctl.sys_status.get('voltage_battery', 11.1) * 1000),
            current_battery=int(self.mavctl.sys_status.get('current_battery', 1.0) * 100),
            battery_remaining=self.mavctl.sys_status.get('battery_remaining', 100),
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=self.mavctl.sys_status.get('errors', 0),
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )

    def send_vfr_hud(self):
        self.mavconn.mav.vfr_hud_send(
            airspeed=0,
            groundspeed=0,
            heading=0,
            throttle=0,
            alt=self.mavctl.position.alt,
            climb=0
        )

class MavlinkControl:
    def __init__(self, inav_conn, connection_string, inav_type=mavutil.mavlink.MAV_TYPE_QUADROTOR, use_mavlink2=True):
        self.mavconn = mavutil.mavlink_connection(
            connection_string,
            source_system=1,
            source_component=1,
            dialect="ardupilotmega",
            force_mavlink2=use_mavlink2
        )
        self.running = True
        self.start_time = time.time()
        self.system_id = 1
        self.autopilot = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        match inav_type:
            case 0: # MULTIROTOR
                self.type = mavutil.mavlink.MAV_TYPE_QUADROTOR
            case 1: # AIRPLANE
                self.type = mavutil.mavlink.MAV_TYPE_FIXED_WING
            case 2: # HELICOPTER
                self.type = mavutil.mavlink.MAV_TYPE_HELICOPTER
            case 3: # TRICOPTER
                self.type = mavutil.mavlink.MAV_TYPE_TRICOPTER
            case 4: # ROVER
                self.type = mavutil.mavlink.MAV_TYPE_GROUND_ROVER
            case 5: # BOAT
                self.type = mavutil.mavlink.MAV_TYPE_SURFACE_BOAT
            case 6: # ??? 
                self.type = mavutil.mavlink.MAV_TYPE_GENERIC
            case _: #
                raise Exception("Error: Invalid vehicle type")
        self.inav_platform_type = inav_type
        self.inavctl = inav_conn
        self.control_mode = 0
        self.control_active = False


        # Flight variables, this needs work
        self.armed = False
        self.flying = False
        self.takeoff = False
        self.failsafe_recovery = False
        self.failure = 0
        self.start_pos = geospatial.GPSposition(45.0, -73.0, 0)
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0}
        self.position = self.start_pos
        self.gps_speed = [0.0, 0.0, 0.0]
        self.battery = {'voltage': 11.1, 'current': 1.0, 'remaining': 100}
        self.mode = 'STABILIZE'
        self.modes = mavutil.AP_MAV_TYPE_MODE_MAP[self.type]
        self.modes_index = dict_reverse(self.modes)
        self.mode_id = self.modes_index[self.mode]
        self.inav_last_modes_cmd = []
        self.system_status = mavutil.mavlink.MAV_STATE_STANDBY
        self.battery = {'voltage': 11.1, 'current': 1.0, 'remaining': 100, 'temperature': 0.0, 'mahdrawn': 0}
        self.sys_status = {'voltage_battery': 11.1, 'current_battery': 1.0, 'battery_remaining': 100, 'errors': 0}
        self.global_position_origin = self.start_pos
        self.scaled_pressure = {'press_abs': 1013.25, 'press_diff': 0.0, 'temperature': 20.0}
        self.rc_channels = [0] * 16
        self.rssi = 255


        # Mission storage and state tracking
        self.gcs_wp_set = False
        self.guided_dest = geospatial.GPSposition(0.0, 0.0, 0)
        self.mission_download = False
        self.mission_items = []
        self.mission_id = 0
        self.current_mission_seq = 0
        self.mission_state = mavutil.mavlink.MISSION_STATE_NO_MISSION
        self.takeoff_alt = 50

        # Initialize telemetry
        self.telemetry = Telemetry(self, self.mavconn, self.start_time)


    def ap_to_inav_modes(self, apmode):
        match apmode:
            case 0: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # STABILIZE
                    return [ inavutil.modesID.ANGLE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # MANUAL
                    return [ inavutil.modesID.MANUAL ]

            case 1: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ACRO
                    return [  ]  # INAV HAS NO MODE ID DEFINED FOR ACRO AND I'M PISSED
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # CIRCLE
                    return [ inavutil.modesID.NAV_POSHOLD ]

            case 2: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ALT_HOLD
                    return [ inavutil.modesID.NAV_ALTHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # STABILIZE
                    return [ inavutil.modesID.ANGLE ]

            case 3: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTO
                    return [ inavutil.modesID.NAV_WP ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # TRAINING
                    return [ inavutil.modesID.MANUAL ]

            case 4:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # GUIDED
                    return [ inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # ACRO
                    return [  ]

            case 5: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # LOITER
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # FBWA
                    return [ inavutil.modesID.NAV_COURSE_HOLD ]

            case 6:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # RTL
                    return [ inavutil.modesID.NAV_RTH ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # FBWB
                    return [ inavutil.modesID.NAV_COURSE_HOLD , inavutil.modesID.ANG_HOLD]

            case 7: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # CIRCLE
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # CRUISE
                    return [ inavutil.modesID.NAV_CRUISE ]

            case 8: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # POSITION
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AUTOTUNE
                    return [ inavutil.modesID.AUTO_TUNE ]

            case 9:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # LAND
                    return [ inavutil.modesID.NAV_POSHOLD ]  

            case 10:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # OF_LOITER
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AUTO
                    return [ inavutil.modesID.NAV_WP ]

            case 11: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # DRIFT
                    return [ inavutil.modesID.ANGLE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # RTL
                    return [ inavutil.modesID.NAV_RTH ]

            case 12: 
                if self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # LOITER
                    return [ inavutil.modesID.NAV_POSHOLD ]

            case 13: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SPORT
                    return [  ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # TAKEOFF
                    return [ inavutil.modesID.ANGLE ]

            case 14: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FLIP
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AVOID_ADSB
                    return None

            case 15: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTOTUNE
                    return [ inavutil.modesID.AUTO_TUNE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # GUIDED
                    return [ inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD ] 

            case 16: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # POSHOLD
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # INITIALISING
                    return [ inavutil.modesID.NAV_MANUAL ]

            case 17:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # BRAKE
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QSTABILIZE
                    return [ inavutil.modesID.NAV_ANGLE ]

            case 18:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # THROW
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QHOVER
                    return [ inavutil.modesID.NAV_POSHOLD ]

            case 19: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AVOID ADSB
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QLOITER
                    return [ inavutil.modesID.NAV_POSHOLD ]

            case 20: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # GUIDED_NOGPS
                    return [ inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QLAND
                    return [ inavutil.modesID.NAV_POSHOLD ]

            case 21: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SMART_RTL
                    return [ inavutil.modesID.NAV_RTH ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QRTL
                    return [ inavutil.modesID.NAV_RTH ]

            case 22:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FLOWHOLD
                    return [ inavutil.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QAUTOTUNE
                    return [ inavutil.modesID.AUTOTUNE ]

            case 23: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FOLLOW
                    return [ inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QACRO
                    return [  ]

            case 24: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ZIGZAG
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # THERMAL
                    return None

            case 25: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SYSTEMID
                    return [ inavutil.modesID.MANUAL ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # LOITERALTQLAND
                    return [ inavutil.modesID.POSHOLD ]

            case 26:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTOROTATE
                    return None

            case 27:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTO_RTL
                    return [ inavutil.modesID.NAV_RTH ]

            case _:
                return None


    def translate_command(self, command_id):
        if command_id in mavutil.mavlink.enums['MAV_CMD']:
            return mavutil.mavlink.enums['MAV_CMD'][command_id].name
        return f"UNKNOWN_COMMAND_{command_id}"


    def handle_param_request_list(self, msg):
        """Handle PARAM_REQUEST_LIST by responding with the minimal set of parameters."""
        # Respond with each parameter
        for i, param in enumerate(self.parameters):
            self.mavconn.mav.param_value_send(
                param_id=param['param_id'],
                param_value=param['param_value'],
                param_type=param['param_type'],
                param_count=self.param_count,
                param_index=i
            )
        print("Handled PARAM_REQUEST_LIST: Sent minimal set of parameters.")

    def handle_param_set(self, msg):
        """Handle PARAM_SET by updating the parameter value."""
        param_id = msg.param_id.rstrip(b'\x00').decode('utf-8')
        for param in self.parameters:
            if param['param_id'].decode('utf-8') == param_id:
                param['param_value'] = msg.param_value
                print(f"Updated parameter {param_id} to {msg.param_value}")
                self.mavconn.mav.param_value_send(
                    param_id=param['param_id'],
                    param_value=param['param_value'],
                    param_type=param['param_type'],
                    param_count=self.param_count,
                    param_index=self.parameters.index(param)
                )
                break

    def handle_mission_request_list(self, msg):
        """Handle MISSION_REQUEST_LIST by sending stored mission items."""
        for seq in range(len(self.mission_items)):
            self.mavconn.mav.mission_request_int_send(
                target_system=self.mavconn.target_system,
                target_component=self.mavconn.target_component,
                seq=seq
            )
            print(f"Requested to send mission item {seq} to GCS.")
            time.sleep(0.1)  # Small delay to prevent message overload

    def handle_mission_request_int(self, msg):
        """Handle MISSION_REQUEST_INT by sending the specific mission item requested by the GCS."""
        seq = msg.seq
        if seq < len(self.mission_items):
            mission_item = self.mission_items[seq]
            self.mavconn.mav.mission_item_int_send(
                target_system=self.mavconn.target_system,
                target_component=self.mavconn.target_component,
                seq=mission_item.seq,
                frame=mission_item.frame,
                command=mission_item.command,
                current=mission_item.current,
                autocontinue=mission_item.autocontinue,
                param1=mission_item.param1,
                param2=mission_item.param2,
                param3=mission_item.param3,
                param4=mission_item.param4,
                x=mission_item.x,
                y=mission_item.y,
                z=mission_item.z,
                mission_type=mission_item.mission_type
            )
            print(f"Sent mission item {seq} to GCS.")

            if seq == len(self.mission_items) - 1:
                # All mission items have been sent, send MISSION_ACK
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=mavutil.mavlink.MAV_MISSION_ACCEPTED
                )
                print("All mission items sent. Sent MISSION_ACK to GCS.")

        else:
            print(f"Invalid mission item sequence: {seq}")

    async def set_guided_wp(self):
        #if self.mode == "GUIDED" and not self.gcs_wp_set and self.armed and inavutil.modesID.GCS_NAV in self.inavctl.get_active_modes():
        t = time.time()
        while inavutil.modesID.GCS_NAV not in self.inavctl.get_board_modes():
            print(self.inav_mode, self.inavctl.get_board_modes())
            await asyncio.sleep(0.5)
            if time.time() - t > 5:
                raise Exception("timeout error on waiting for GCS NAV mode")
        wp = self.inavctl.set_wp(255, 1, self.guided_dest.lat, self.guided_dest.lon, self.guided_dest.alt, 0, 0, 0, 0)
        self.gcs_wp_set = True

    def send_home_position(self):
        """
        Sends the MAVLink HOME_POSITION message with the current home position data.
        """
        lat = int(self.start_pos.lat * 1e7)  # Latitude in WGS84, degrees * 1e7
        lon = int(self.start_pos.lon * 1e7)  # Longitude in WGS84, degrees * 1e7
        alt = int(self.start_pos.alt * 1000)  # Altitude in millimeters

        # NED (North-East-Down) local coordinates
        x = 0.0
        y = 0.0
        z = 0.0

        # Quaternion for the orientation (set to NaN if not available)
        q = [float('nan')] * 4

        # Approach vector (optional, set to NaN if not available)
        approach_x = float('nan')
        approach_y = float('nan')
        approach_z = float('nan')

        # Timestamp (UNIX Epoch or time since boot)
        time_usec = int(time.time() * 1e6)

        # Send the HOME_POSITION message
        self.mavconn.mav.home_position_send(
            lat, lon, alt, x, y, z, q, approach_x, approach_y, approach_z, time_usec
        )


    async def cmd_arm(self, arm_disarm):
        # arm checks here
        self.inavctl.set_mode(inavutil.modesID.ARM, on=arm_disarm)
        t = time.time()

        while self.armed != arm_disarm:
            await asyncio.sleep(0.5)
            self.armed = self.inavctl.armed
            if time.time() - t > 5:
                print("timeout error on waiting for CMD ARM")
                break
            else:
                self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS)

        print(f"Vehicle {'armed' if self.armed else 'disarmed'}")
        self.system_status = mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else mavutil.mavlink.MAV_STATE_STANDBY
        cmd_res = mavutil.mavlink.MAV_RESULT_ACCEPTED if self.armed == arm_disarm else mavutil.mavlink.MAV_RESULT_FAILED
        self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, result=cmd_res)

    async def arm(self, arm_disarm):
        # arm checks here
        self.inavctl.set_mode(inavutil.modesID.ARM, on=arm_disarm)
        t = time.time()
        while self.armed != arm_disarm:
            await asyncio.sleep(0.1)
            self.armed = self.inavctl.armed
            if time.time() - t > 5:
                print("timeout error on waiting for ARM")
                return False
        print(f"Vehicle {'armed' if self.armed else 'disarmed'}")
        return True

    async def mode_arm(self,arm_disarm, mode):
        pass


    async def set_ap_mode(self, mode_id):
        print('Setting mode to',self.modes.get(mode_id, "UNKNOWN"))
        if mode_id == self.modes_index["AUTO"] and len(self.mission_items)==0:
            print('Error: Cannot set to AUTO, no waypoints on aircraft')
            self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_FAILED)
            return None

        elif mode_id == self.modes_index["GUIDED"] and not self.inavctl.armed:
            print('Error: Cannot set to GUIDED, not armed')
            self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_FAILED)
            return None

        elif (mode_id == self.modes_index["TAKEOFF"] or mode_id == self.modes_index["AUTO"]) and not self.armed: # arm then takeoff
            if mode_id == self.modes_index["TAKEOFF"] and self.flying or self.takeoff!=0:
                print(f'Error: Cannot set to {self.modes.get(mode_id, "UNKNOWN")}, already flying')
                self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_FAILED)
                return None

            armtask = asyncio.create_task(self.arm(True))
            t = time.time()
            while not armtask.done():
                await asyncio.sleep(0.5)
                self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
            armres = await armtask
            if not armres:
                self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_FAILED)
                return None
            else:
                self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
                print(f'Vehicle armed by {self.modes.get(mode_id, "UNKNOWN")}')

        if (mode_id == self.modes_index["TAKEOFF"] or mode_id == self.modes_index["AUTO"]) and self.takeoff == 0:
            asyncio.create_task(self.takeoff_program(self.takeoff_alt, auto=(mode_id == self.modes_index["AUTO"])))

        self.mode_id = mode_id
        self.mode = self.modes.get(mode_id, "UNKNOWN")

        if mode_id == self.modes_index["AUTO"] and self.takeoff == 0:
            self.inav_mode = self.ap_to_inav_modes(self.modes_index["TAKEOFF"])
        else:
            self.inav_mode = self.ap_to_inav_modes(mode_id)
        
        for inav_mode_id in self.inav_last_modes_cmd:
            self.inavctl.set_mode(inav_mode_id, on=False)

        for inav_mode_id in self.inav_mode:
            self.inavctl.set_mode(inav_mode_id, on=True)

        self.inav_last_modes_cmd = self.inav_mode
        
        if self.mode == "GUIDED":
            self.gcs_wp_set = False

        #print('Active modes:')
        #for i in self.inavctl.get_active_modes():
        #    print('\t',i, inavutil.modesID.get(i))
        print(f"Flight mode changed to: {self.mode} ({[inavutil.modesID.get(i) for i in self.inav_mode]})")
        #print('inavctl rc:',self.inavctl.channels)
        #print('real rc:',self.inavctl.board.RC['channels'])
        self.mavconn.mav.command_ack_send(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=mavutil.mavlink.MAV_RESULT_ACCEPTED)


    async def takeoff_program(self, set_alt, auto=False):
        if self.inav_platform_type == 0: # multirotor
            pass

        elif self.inav_platform_type == 1: #fixed wing
            self.takeoff = 1
            #self.inavctl.set_mode(inavutil.modesID.ANGLE, on=True)
            #await asyncio.sleep(1)
            print('Taking off')
            self.inavctl.set_rc_channel('throttle',2000)
            uavalt = 0
            t=time.time()

            while uavalt<set_alt :
                self.inavctl.set_rc_channel('throttle',2100)
                self.inavctl.set_rc_channel('pitch',1100)
                #uavspeed = self.inavctl.get_gps_data()['speed']
                uavalt = self.inavctl.get_altitude()
                #imu = self.inavctl.get_imu()
                print('Alt:', uavalt)
                await asyncio.sleep(0.2)

                if time.time() - t > 30:
                    print('Takeoff failed')
                    await self.set_ap_mode(self.modes_index["MANUAL"])
                    self.takeoff = -1 #failed
                    return False

            await asyncio.sleep(1)
            self.inavctl.set_rc_channel('pitch',1500)
            print("Takeoff success")
            self.takeoff = 2 # success
            self.flying = True
            if auto: 
                await self.set_ap_mode(self.modes_index["AUTO"])
            else:
                await self.set_ap_mode(self.modes_index["LOITER"])
            return True

    async def receive_messages(self):
        msg = self.mavconn.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == 'COMMAND_LONG':
                command_name = self.translate_command(msg.command)
                reply = None
                
                # https://mavlink.io/en/messages/common.html#mav_commands


                if msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    #print(f"SET_MSG_INTERVAL: {self.translate_command(msg.param1)} {msg.param2}")
                    reply = mavutil.mavlink.MAV_RESULT_ACCEPTED # handle later

                # ARM/DISARM
                elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    arm_disarm = msg.param1 == 1.0
                    asyncio.create_task(self.cmd_arm(arm_disarm))
                    reply = mavutil.mavlink.MAV_RESULT_IN_PROGRESS
                
                # mode change
                elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    mode_id = int(msg.param2)
                    asyncio.create_task(self.set_ap_mode(mode_id))
                    reply = mavutil.mavlink.MAV_RESULT_IN_PROGRESS #self.set_ap_mode(mode_id)
                    #print(f"Mode changed: Sending ACK for mode change to {self.mode}")

                elif msg.command == mavutil.mavlink.MAV_CMD_GET_HOME_POSITION:
                    print('Home position requested')
                    self.mavconn.mav.command_ack_send(
                        command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                        result=mavutil.mavlink.MAV_RESULT_ACCEPTED
                    )
                    self.send_home_position()

                elif msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                    print(f"Message requested: {self.translate_command(msg.param1)}")
                    reply = mavutil.mavlink.MAV_RESULT_FAILED

                # mission start
                elif msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                    print("Command: Mission start")
                    reply = mavutil.mavlink.MAV_RESULT_IN_PROGRESS #self.set_ap_mode(self.modes_index["AUTO"])

                # MAV_CMD_GET_HOME_POSITION

                else:
                    print(f"Received message: COMMAND_LONG -> {msg.to_dict()}, Command: {command_name} ({msg.command})")

                if reply is not None:
                    self.mavconn.mav.command_ack_send(command = msg.command, result=reply)

            elif msg.get_type() == 'REQUEST_DATA_STREAM': # deprecated!
                stream_name = mavutil.mavlink.enums['MAV_DATA_STREAM'].get(msg.req_stream_id).name
                #self.telemetry.stream_rates[stream_name] = msg.req_message_rate
                self.mavconn.mav.data_stream_send(
                    stream_id=msg.req_stream_id,
                    message_rate=msg.req_message_rate,
                    on_off=msg.start_stop
                )
                print(f"Received message: REQUEST_DATA_STREAM -> Stream: {stream_name}, Rate: {msg.req_message_rate}, Start/Stop: {msg.start_stop}")


            elif False and msg.get_type() == 'PARAM_REQUEST_LIST': # this makes things worse
                print(f"Received PARAM_REQUEST_LIST -> {msg.to_dict()}")
                self.mavconn.mav.param_value_send(
                    param_id=b'\x00' * 16,  # 16-byte empty parameter ID
                    param_value=0, 
                    param_type=0,
                    param_count=0,  
                    param_index=0 
                )

            # handle this LATER 
            #elif msg.get_type() == 'PARAM_REQUEST_LIST':
            #    print(f"Received PARAM_REQUEST_LIST -> {msg.to_dict()}")
            #    self.handle_param_request_list(msg)
                
            #elif msg.get_type() == 'PARAM_SET':
            #    print(f"Received PARAM_SET -> {msg.to_dict()}")
            #    self.handle_param_set(msg)

            elif msg.get_type() == 'MISSION_ITEM': 
                print(f"Received {msg.get_type()}: Sequence {msg.seq}, Command {msg.command}, " 
                    f"Coordinates (Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z})")

                self.mission_items.append(msg)  # Store the mission item
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=mavutil.mavlink.MAV_MISSION_ACCEPTED
                )
            
            elif msg.get_type() == 'MISSION_ITEM_INT':
                print(f"Received {msg.get_type()}: Sequence {msg.seq}, Command {msg.command}, " 
                    f"Coordinates (Frame: {msg.frame}, Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z}), Param1: {msg.param1}, "
                    f"Param2: {msg.param2}, Param3: {msg.param3}")

                self.mission_items.append(msg)  # Store the mission item

                if len(self.mission_items) < self.expected_mission_count:
                    # Request the next mission item
                    self.mavconn.mav.mission_request_int_send(
                        target_system=self.mavconn.target_system,
                        target_component=self.mavconn.target_component,
                        seq=len(self.mission_items)
                    )
                    print(f"Requested mission item {len(self.mission_items)}")

                else:
                    # All mission items received, send MISSION_ACK
                    result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                    self.mission_download = False
                    self.mavconn.mav.mission_ack_send(
                        target_system=self.mavconn.target_system,
                        target_component=self.mavconn.target_component,
                        type=result
                    )

                    # send mission items to INAV
                    lastalt = 0
                    offset = 0
                    for i in range(len(self.mission_items)):
                        if i==0:
                            continue
                        wp = self.mission_items[i]
                        lat = wp.x / 1e7
                        lon = wp.y / 1e7
                        alt = wp.z
                        p1 = 0
                        p2 = 0
                        p3 = 0
                        match wp.command:
                            case mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                                wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_WAYPOINT 
                                p3 = 0
                                # P3 defines the altitude mode. 0 (default, legacy) = Relative to Home, 1 = Absolute (AMSL)

                            case mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                                p1 = -1
                                wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_HOLD_TIME

                            case mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME:
                                wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_HOLD_TIME
                                p1 = wp.param1
                                # P1 = time, P3 as above

                            case mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                                wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_RTH
                                p1 = 0
                                alt = lastalt
                                # P1>0 = Land

                            case mavutil.mavlink.MAV_CMD_NAV_LAND:
                                #wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_LAND
                                wp_action = inavutil.navWaypointActions.NAV_WP_ACTION_RTH
                                alt = lastalt
                                p1 = 1

                            case mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                                self.wp_takeoff = True
                                self.takeoff_alt = wp.z
                                wp_action = None
                                offset+=1

                            case _:
                                wp_action = None

                        wpi = i - offset
                        if wp_action:
                            print()
                            print(wp)
                            lastalt = alt
                            wpflag = 165 if i == len(self.mission_items)-1 else 0
                            #print('inav wp',wpi, wp_action, lat, lon, alt, p1, p2, p3, wpflag)
                            inavwp = self.inavctl.set_wp(wpi, wp_action, lat, lon, alt, p1, p2, p3, wpflag)
                            print(self.inavctl.get_wp(i))
                        #else:
                            #print('Error: unsupported waypoint type',wp_action)
                        #    break



                    print(f"Sent MISSION_ACK: result={result}")
                    self.current_mission_seq = 0
                    self.mission_state = mavutil.mavlink.MISSION_STATE_NOT_STARTED

            elif msg.get_type() == 'MISSION_COUNT':
                self.expected_mission_count = msg.count
                self.mission_download = True
                print(f"Received MISSION_COUNT: Count {msg.count}")
                # Request the first mission item
                self.mavconn.mav.mission_request_int_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    seq=0
                )
                print("Requested mission item 0")

            elif msg.get_type() == 'MISSION_CLEAR_ALL':
                result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=result
                )
                print(f"Sent MISSION_ACK: result={result}")
                self.current_mission_seq = 0
                self.mission_state = mavutil.mavlink.MISSION_STATE_NOT_STARTED
                self.mission_items = []

            elif msg.get_type() == 'MISSION_REQUEST_LIST':
                print(f"Received MISSION_REQUEST_LIST: Sending {len(self.mission_items)} mission items.")
                self.handle_mission_request_list(msg)

            elif msg.get_type() == 'MISSION_REQUEST_INT':
                self.handle_mission_request_int(msg)


            elif msg.get_type() == 'COMMAND_INT':

                if msg.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                    print(f"Received MAV_CMD_DO_REPOSITION: Speed={msg.param1} m/s, Bitmask={msg.param2}, "
                        f"Radius={msg.param3} m, Yaw={msg.param4} deg, Latitude={msg.x}, Longitude={msg.y}, Altitude={msg.z}")
                    self.guided_dest.lat = msg.x / 1e7
                    self.guided_dest.lon = msg.y / 1e7
                    self.guided_dest.alt = msg.z
                    self.gcs_wp_set = False
                    asyncio.create_task(self.set_guided_wp())
                else:
                    print(f"Received COMMAND_INT: Command {msg.command}, Coordinates (Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z})")

                result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=result
                )
                print(f"Sent MISSION_ACK: result={result}")

            elif msg.get_type() == 'HEARTBEAT':
                self.last_gcs_heartbeat = time.time()

            elif msg.get_type() == 'SET_MODE':
                print(msg)
                print('SET_MODE',msg.custom_mode)
                mode_id = int(msg.custom_mode)
                self.set_ap_mode(mode_id)
                asyncio.create_task(self.set_ap_mode(mode_id))

            elif msg.get_type() == 'MANUAL_CONTROL':
                pass
                #print("MANUAL CONTROL SPAM:",msg)

            else: 
                print(f"Received -> {msg.to_dict()}")


    def update(self):
        if len(self.mission_items) == 0:
            self.mission_state = mavutil.mavlink.MISSION_STATE_NO_MISSION

        elif self.mode_id == 3 and self.mission_state in (2,4):
            self.mission_state = mavutil.mavlink.MISSION_STATE_ACTIVE
            self.current_mission_seq = 0

        elif self.mode_id != 3 and self.mission_state == 3:
            self.mission_state = mavutil.mavlink.MISSION_STATE_PAUSED

        if self.control_mode == 0:
            self.control_active = self.inavctl.msp_override_active

        if self.failure == 0:
            self.system_status = mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else mavutil.mavlink.MAV_STATE_STANDBY
        else:
            self.system_status = mavutil.mavlink.MAV_STATE_CRITICAL

        # detect arm conflict (script has switch to armed but vehicle disarmed etc)
        self.armed = self.inavctl.armed

        if self.flying and not self.armed and self.altitude<1 and self.speed<=1:
            self.flying = False
            self.takeoff = 0
            print('Landing detected')

            
    async def mav_ctl(self):
        print("Starting MAVLINK interface...")
        try:
            #set_alt = 50

            last_msp = time.time()
            self.inavctl.debugprint = False
            print(self.inavctl.get_sensor_config())
            print()
            #self.inavctl.modes.keys()
            #self.inavctl.new_supermode('GOTO', [inavutil.modesID.GCS_NAV, inavutil.modesID.NAV_POSHOLD])

            if self.control_mode == 1:
                self.control_active = True

            # already flying check
            gpsd = self.inavctl.get_gps_data()
            self.altitude = self.inavctl.get_altitude()
            if gpsd['speed']>5 and self.attitude>10:
                print('Oops, already in flight!')
                self.flying = True
                self.takeoff = 1

            while self.running:
                if not self.inavctl.run: 
                    self.running = False
                    break
                
                current_time = time.time()
                self.update()
                self.telemetry.send(current_time)
                await self.receive_messages()

                if time.time()-last_msp >= 0.1:
                    #print([inavutil.modesID.get(i) for i in self.inavctl.get_board_modes()])
                    #print(inavutil.modesID.FAILSAFE)
                    if inavutil.modesID.FAILSAFE in self.inavctl.get_board_modes() and not self.failsafe_recovery:
                        self.failsafe_recovery = True
                        print('FS RECOVERY')
                        self.inavctl.set_rc_channel('pitch',random.uniform(1000,2000))

                    elif inavutil.modesID.FAILSAFE not in self.inavctl.get_board_modes() and self.failsafe_recovery:
                        self.inavctl.set_rc_channel('pitch',1500)
                        self.failsafe_recovery = False

                    gpsd = self.inavctl.get_gps_data()
                    gyro = self.inavctl.get_attitude()
                    #self.inavctl.std_send(inavutil.msp.MSP_CURRENT_METERS)
                    
                    #print(f"{[inavutil.modesID.get(i) for i in self.inavctl.get_board_modes()]}")

                    self.altitude = self.inavctl.get_altitude()
                    self.position = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], self.altitude)
                    self.speed = gpsd['speed']
                    self.attitude =  {
                        'roll': math.radians(gyro['roll']), 
                        'pitch': math.radians(gyro['pitch']), 
                        'yaw': math.radians(gyro['yaw']), 
                        'rollspeed': 0.0, 
                        'pitchspeed': 0.0, 
                        'yawspeed': 0.0
                    }
                    self.battery =  {
                        'voltage': self.inavctl.board.ANALOG['voltage'] / 10.0, 
                        'current': self.inavctl.board.ANALOG['amperage'], 
                        'remaining': self.inavctl.board.ANALOG['battery_percentage'], 
                        'mwhdrawn': self.inavctl.board.ANALOG['mWhdrawn'],
                        'temperature': 0, 
                        'mahdrawn': self.inavctl.board.ANALOG['mAhdrawn']
                        }
                    self.sys_status = {
                        'voltage_battery': self.inavctl.board.ANALOG['voltage'], 
                        'current_battery': self.inavctl.board.ANALOG['amperage'], 
                        'battery_remaining': self.inavctl.board.ANALOG['battery_percentage'], 
                        'errors': 0
                        }
                    nav_status = self.inavctl.get_nav_status()
                    #print(nav_status)

                #vector = geospatial.gps_to_vector(pos, wp)
                #print('\n')
                #print('Channels:', self.inavctl.board.RC['channels'])
                #inavctl.set_mode("MSP RC OVERRIDE", on=True)
                #print('Active modes:', self.inavctl.get_active_modes())
                #print('Position:', self.position)
                #print('Attitude:', gyro)
                #print('Altitude:', alt)
                #print('Vector to waypoint:', vector)
                #print('Bearing:',vector.az - gyro['yaw'])
                #if inavctl.msp_override_active:
                #    print('OVERRIDE ACTIVE')
                await asyncio.sleep(0.01)

            self.running = False
            inavctl.stop()
            print('Stopped')

                

        except Exception:
            print('!!! Error in Flight Control loop !!!')
            print(traceback.format_exc())
            self.running = False
            self.system_status = mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION
            for i in range(3):
                self.telemetry.send_heartbeat()
                await asyncio.sleep(1)
            return 1



async def main():
    sys_id = 1
    inav_platform = inavutil.flyingPlatformType.PLATFORM_AIRPLANE
    uavctl = UAVControl(device='/dev/ttyUSB1', baudrate=115200, platform=inav_platform)
    #uavctl.msp_override_channels=[1, 2, 3, 4, 5, 6, 11]
    mav = MavlinkControl(inav_conn=uavctl, connection_string='udpout:localhost:14550', inav_type=inav_platform, use_mavlink2=True)

    OVERRIDE_CONTROL = 0
    MSP_RC_CONTROL = 1
    MAVLINK_RC_CONTROL = 2
    mav.control_mode = MAVLINK_RC_CONTROL # change this

    uavctl.msp_receiver = mav.control_mode == 1
    try:
        await uavctl.connect()
        print("Connected to the flight controller")
        flight_control_task = asyncio.create_task(uavctl.flight_loop())
        mavloop = asyncio.create_task(mav.mav_ctl())
        await asyncio.gather(flight_control_task, mavloop)
    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
