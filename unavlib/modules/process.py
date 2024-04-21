
import logging
import struct
import time
import sys
import serial
from select import select

class processMSP():
    def __init__(self, mspy_instance):
        self.mspy = mspy_instance

    def process_MSP_STATUS(self, data):
        self.mspy.CONFIG['cycleTime'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['i2cError'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['activeSensors'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['mode'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.CONFIG['profile'] = self.mspy.readbytes(data, size=8, unsigned=True)
        
    def process_MSP_STATUS_EX(self, data):
        self.mspy.CONFIG['cycleTime'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['i2cError'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['activeSensors'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['mode'] = self.mspy.readbytes(data, size=32, unsigned=True)

        self.mspy.CONFIG['profile'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.CONFIG['cpuload'] = self.mspy.readbytes(data, size=16, unsigned=True)
        
        if not self.mspy.INAV:
            self.mspy.CONFIG['numProfiles'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.CONFIG['rateProfile'] = self.mspy.readbytes(data, size=8, unsigned=True)

            # Read flight mode flags
            byteCount = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.CONFIG['flightModeFlags'] = [] # this was not implemented on betaflight-configurator
            for _ in range(byteCount):
                # betaflight-configurator would just discard these bytes
                self.mspy.CONFIG['flightModeFlags'].append(self.mspy.readbytes(data, size=8, unsigned=True))

            # Read arming disable flags
            self.mspy.CONFIG['armingDisableCount'] = self.mspy.readbytes(data, size=8, unsigned=True) # Flag count
            self.mspy.CONFIG['armingDisableFlags'] = self.mspy.readbytes(data, size=32, unsigned=True)
        else:
            self.mspy.CONFIG['armingDisableFlags'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP2_INAV_STATUS(self, data):
        self.mspy.CONFIG['cycleTime'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['i2cError'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['activeSensors'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['cpuload'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.CONFIG['profile'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.CONFIG['armingDisableFlags'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.CONFIG['mode'] = self.mspy.readbytes(data, size=32, unsigned=True)
        

    def process_MSP_RAW_IMU(self, data):
        # /512 for mpu6050, /256 for mma
        # currently we are unable to differentiate between the sensor types, so we are going with 512
        # And what about SENSOR_CONFIG???
        self.mspy.SENSOR_DATA['accelerometer'][0] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['accelerometer'][1] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['accelerometer'][2] = self.mspy.readbytes(data, size=16, unsigned=False)

        # properly scaled (INAV and BF use the same * (4 / 16.4))
        # but this is supposed to be RAW, so raw it is!
        self.mspy.SENSOR_DATA['gyroscope'][0] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['gyroscope'][1] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['gyroscope'][2] = self.mspy.readbytes(data, size=16, unsigned=False)

        # no clue about scaling factor (/1090), so raw
        self.mspy.SENSOR_DATA['magnetometer'][0] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['magnetometer'][1] = self.mspy.readbytes(data, size=16, unsigned=False)
        self.mspy.SENSOR_DATA['magnetometer'][2] = self.mspy.readbytes(data, size=16, unsigned=False)

    def process_MSP_SERVO(self, data):
        servoCount = int(len(data) / 2)
        self.mspy.SERVO_DATA = [self.mspy.readbytes(data, size=16, unsigned=True) for _ in range(servoCount)]

    def process_MSP_MOTOR(self, data):
        motorCount = int(len(data) / 2)
        self.mspy.MOTOR_DATA = [self.mspy.readbytes(data, size=16, unsigned=True) for i in range(motorCount)]

    def process_MSP_RC(self, data):
        n_channels = int(len(data) / 2)
        self.mspy.RC['active_channels'] = n_channels
        self.mspy.RC['channels'] = [self.mspy.readbytes(data, size=16, unsigned=True) for i in range(n_channels)]

    def process_MSP_RAW_GPS(self, data):
        self.mspy.GPS_DATA['fix'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.GPS_DATA['numSat'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.GPS_DATA['lat'] = self.mspy.readbytes(data, size=32, unsigned=False)
        self.mspy.GPS_DATA['lon'] = self.mspy.readbytes(data, size=32, unsigned=False)
        self.mspy.GPS_DATA['alt'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['speed'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['ground_course'] = self.mspy.readbytes(data, size=16, unsigned=True)

        if self.mspy.INAV:
            self.mspy.GPS_DATA['hdop'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_COMP_GPS(self, data):
        self.mspy.GPS_DATA['distanceToHome'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['directionToHome'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['update'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPSSTATISTICS(self, data):
        self.mspy.GPS_DATA['messageDt'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['errors'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.GPS_DATA['timeouts'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.GPS_DATA['packetCount'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.GPS_DATA['hdop'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['eph'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_DATA['epv'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_ATTITUDE(self, data):
        self.mspy.SENSOR_DATA['kinematics'][0] = self.mspy.readbytes(data, size=16, unsigned=False) / 10.0 # x
        self.mspy.SENSOR_DATA['kinematics'][1] = self.mspy.readbytes(data, size=16, unsigned=False) / 10.0 # y
        self.mspy.SENSOR_DATA['kinematics'][2] = self.mspy.readbytes(data, size=16, unsigned=False) # z

    def process_MSP_ALTITUDE(self, data):
        self.mspy.SENSOR_DATA['altitude'] = round((self.mspy.readbytes(data, size=32, unsigned=False) / 100.0), 2) # correct scale factor
        self.mspy.SENSOR_DATA['altitude_vel'] = round(self.mspy.readbytes(data, size=16, unsigned=False) / 100.0, 2)
        # Baro altitude => self.mspy.readbytes(data, size=32, unsigned=True)


    def process_MSP_SONAR(self, data):
        self.mspy.SENSOR_DATA['sonar'] = self.mspy.readbytes(data, size=32, unsigned=False)

    def process_MSP_ANALOG(self, data):
        self.mspy.ANALOG['voltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10.0
        self.mspy.ANALOG['mAhdrawn'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.ANALOG['rssi'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-1023
        self.mspy.ANALOG['amperage'] = self.mspy.readbytes(data, size=16, unsigned=False) / 100 # A
        self.mspy.ANALOG['last_received_timestamp'] = int(time.time()) # why not monotonic? where is time synchronized?
        if not self.mspy.INAV:
            self.mspy.ANALOG['voltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
    
    def process_MSP2_INAV_ANALOG(self, data):
        if self.mspy.INAV:
            tmp = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ANALOG['battery_full_when_plugged_in'] = True if (tmp & 1) else False
            self.mspy.ANALOG['use_capacity_thresholds'] = True if ((tmp & 2) >> 1) else False
            self.mspy.ANALOG['battery_state'] = (tmp & 12) >> 2
            self.mspy.ANALOG['cell_count'] = (tmp & 0xF0) >> 4

            self.mspy.ANALOG['voltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
            self.mspy.ANALOG['amperage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100 # A
            self.mspy.ANALOG['power'] = self.mspy.readbytes(data, size=32, unsigned=True) / 100
            self.mspy.ANALOG['mAhdrawn'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.ANALOG['mWhdrawn'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.ANALOG['battery_remaining_capacity'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.ANALOG['battery_percentage'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ANALOG['rssi'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-1023

            # TODO: update both BF and INAV variables
            self.mspy.BATTERY_STATE['cellCount'] = self.mspy.ANALOG['cell_count']

    def process_MSP_VOLTAGE_METERS(self, data):
        total_bytes_per_meter = (8+8)/8 # just to make it clear where it comes from...
        self.mspy.VOLTAGE_METERS = [{'id':self.mspy.readbytes(data, size=8, unsigned=True),
                                'voltage':self.mspy.readbytes(data, size=8, unsigned=True) / 10.0
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_CURRENT_METERS(self, data):
        total_bytes_per_meter = (8+16+16)/8 # just to make it clear where it comes from...
        self.mspy.CURRENT_METERS = [{'id':self.mspy.readbytes(data, size=8, unsigned=True),
                                'mAhDrawn':self.mspy.readbytes(data, size=16, unsigned=True), # mAh
                                'amperage':self.mspy.readbytes(data, size=16, unsigned=True) / 1000 # A
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_BATTERY_STATE(self, data):
        self.mspy.BATTERY_STATE['cellCount'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.BATTERY_STATE['capacity'] = self.mspy.readbytes(data, size=16, unsigned=True) # mAh
        # BATTERY_STATE.voltage = data.readU8() / 10.0; // V
        self.mspy.BATTERY_STATE['mAhDrawn'] = self.mspy.readbytes(data, size=16, unsigned=True) # mAh
        self.mspy.BATTERY_STATE['amperage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100 # A
        self.mspy.BATTERY_STATE['batteryState'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.BATTERY_STATE['voltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100 # V

    def process_MSP_VOLTAGE_METER_CONFIG(self, data):
        self.mspy.VOLTAGE_METER_CONFIGS = []
        if self.mspy.INAV:
            voltageMeterConfig = {}
            voltageMeterConfig['vbatscale'] = self.mspy.readbytes(data, size=8, unsigned=True)/10
            self.mspy.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)
            self.mspy.BATTERY_CONFIG['vbatmincellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True)/10
            self.mspy.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True)/10
            self.mspy.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True)/10
        else:
            voltage_meter_count = self.mspy.readbytes(data, size=8, unsigned=True)

            for i in range(voltage_meter_count):
                subframe_length = self.mspy.readbytes(data, size=8, unsigned=True)
                if (subframe_length != 5):
                    for j in range(subframe_length):
                        self.mspy.readbytes(data, size=8, unsigned=True)
                else:
                    voltageMeterConfig = {}
                    voltageMeterConfig['id'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['sensorType'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatscale'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivval'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivmultiplier'] = self.mspy.readbytes(data, size=8, unsigned=True)

                    self.mspy.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)

    def process_MSP_CURRENT_METER_CONFIG(self, data):
        self.mspy.CURRENT_METER_CONFIGS = []
        if self.mspy.INAV:
            currentMeterConfig = {}
            currentMeterConfig['scale'] = self.mspy.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['offset'] = self.mspy.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['sensorType'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.CURRENT_METER_CONFIGS.append(currentMeterConfig)
            self.mspy.BATTERY_CONFIG['capacity'] = self.mspy.readbytes(data, size=16, unsigned=True)
        else:
            current_meter_count = self.mspy.readbytes(data, size=8, unsigned=True)
            for i in range(current_meter_count):
                currentMeterConfig = {}
                subframe_length = self.mspy.readbytes(data, size=8, unsigned=True)

                if (subframe_length != 6):
                    for j in range(subframe_length):
                        self.mspy.readbytes(data, size=8, unsigned=True)
                else:
                    currentMeterConfig['id'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['sensorType'] = self.mspy.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['scale'] = self.mspy.readbytes(data, size=16, unsigned=False)
                    currentMeterConfig['offset'] = self.mspy.readbytes(data, size=16, unsigned=False)

                    self.mspy.CURRENT_METER_CONFIGS.append(currentMeterConfig)

    def process_MSP_BATTERY_CONFIG(self, data):
        self.mspy.BATTERY_CONFIG['vbatmincellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.mspy.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.mspy.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.mspy.BATTERY_CONFIG['capacity'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.BATTERY_CONFIG['voltageMeterSource'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.BATTERY_CONFIG['currentMeterSource'] = self.mspy.readbytes(data, size=8, unsigned=True)

        self.mspy.BATTERY_CONFIG['vbatmincellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
        self.mspy.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
        self.mspy.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_RC_TUNING(self, data):
        self.mspy.RC_TUNING['RC_RATE'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.mspy.RC_TUNING['RC_EXPO'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.mspy.RC_TUNING['roll_pitch_rate'] = 0
        self.mspy.RC_TUNING['roll_rate'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.mspy.RC_TUNING['pitch_rate'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.mspy.RC_TUNING['yaw_rate'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.mspy.RC_TUNING['dynamic_THR_PID'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.mspy.RC_TUNING['throttle_MID'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.mspy.RC_TUNING['throttle_EXPO'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.mspy.RC_TUNING['dynamic_THR_breakpoint'] = self.mspy.readbytes(data, size=16, unsigned=True)

        self.mspy.RC_TUNING['RC_YAW_EXPO'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        if not self.mspy.INAV:
            self.mspy.RC_TUNING['rcYawRate'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.mspy.RC_TUNING['rcPitchRate'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)
            self.mspy.RC_TUNING['RC_PITCH_EXPO'] = round((self.mspy.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.mspy.RC_TUNING['throttleLimitType'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RC_TUNING['throttleLimitPercent'] = self.mspy.readbytes(data, size=8, unsigned=True)

            if int("".join((self.mspy.CONFIG['apiVersion'].rsplit('.')))) >= 1420:
                self.mspy.RC_TUNING['roll_rate_limit'] = self.mspy.readbytes(data, size=16, unsigned=True)
                self.mspy.RC_TUNING['pitch_rate_limit'] = self.mspy.readbytes(data, size=16, unsigned=True)
                self.mspy.RC_TUNING['yaw_rate_limit'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_PID(self, data):
        self.mspy.PIDs = [
            [
                self.mspy.readbytes(data, size=8, unsigned=True) for _ in range(3)
            ] 
            for _ in range(int(len(data)/3))
        ]

    def process_MSP2_PID(self, data):
        self.mspy.PIDs = [
            [
                self.mspy.readbytes(data, size=8, unsigned=True) for _ in range(4)
            ] 
            for _ in range(int(len(data)/4))
        ]

    def process_MSP_ARMING_CONFIG(self, data):
        self.mspy.ARMING_CONFIG['auto_disarm_delay'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.ARMING_CONFIG['disarm_kill_switch'] = self.mspy.readbytes(data, size=8, unsigned=True)
        if not self.mspy.INAV:
            self.mspy.ARMING_CONFIG['small_angle'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_LOOP_TIME(self, data):
        if self.mspy.INAV:
            self.mspy.FC_CONFIG['loopTime'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_MISC(self, data): # 22 bytes
        if self.mspy.INAV:
            self.mspy.MISC['midrc'] = self.mspy.RX_CONFIG['midrc'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MISC['minthrottle'] = self.mspy.MOTOR_CONFIG['minthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['maxthrottle'] = self.mspy.MOTOR_CONFIG['maxthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['mincommand'] = self.mspy.MOTOR_CONFIG['mincommand'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['failsafe_throttle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.mspy.MISC['gps_type'] = self.mspy.GPS_CONFIG['provider'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['sensors_baudrate'] = self.mspy.MISC['gps_baudrate'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['gps_ubx_sbas'] = self.mspy.GPS_CONFIG['ublox_sbas'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['multiwiicurrentoutput'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['rssi_channel'] = self.mspy.RSSI_CONFIG['channel'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['placeholder2'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.COMPASS_CONFIG['mag_declination'] = self.mspy.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000
            
            self.mspy.MISC['mag_declination'] = self.mspy.COMPASS_CONFIG['mag_declination']*10

            self.mspy.MISC['vbatscale'] = self.mspy.readbytes(data, size=8, unsigned=True) # 10-200
            self.mspy.MISC['vbatmincellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.mspy.MISC['vbatmaxcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.mspy.MISC['vbatwarningcellvoltage'] = self.mspy.readbytes(data, size=8, unsigned=True) / 10 # 10-50

    def process_MSP2_INAV_MISC(self, data):
        if self.mspy.INAV:
            self.mspy.MISC['midrc'] = self.mspy.RX_CONFIG['midrc'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MISC['minthrottle'] = self.mspy.MOTOR_CONFIG['minthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['maxthrottle'] = self.mspy.MOTOR_CONFIG['maxthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['mincommand'] = self.mspy.MOTOR_CONFIG['mincommand'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
            self.mspy.MISC['failsafe_throttle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.mspy.MISC['gps_type'] = self.mspy.GPS_CONFIG['provider'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['sensors_baudrate'] = self.mspy.MISC['gps_baudrate'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['gps_ubx_sbas'] = self.mspy.GPS_CONFIG['ublox_sbas'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['rssi_channel'] = self.mspy.RSSI_CONFIG['channel'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.MISC['mag_declination'] = self.mspy.readbytes(data, size=16, unsigned=False) / 10 # -18000-18000
            self.mspy.MISC['vbatscale'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MISC['voltage_source'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['battery_cells'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MISC['vbatdetectcellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
            self.mspy.MISC['vbatmincellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
            self.mspy.MISC['vbatmaxcellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
            self.mspy.MISC['vbatwarningcellvoltage'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100
            self.mspy.MISC['battery_capacity'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.MISC['battery_capacity_warning'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.MISC['battery_capacity_critical'] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.MISC['battery_capacity_unit'] = 'mWh' if self.mspy.readbytes(data, size=8, unsigned=True) else 'mAh'

    def process_MSP_MOTOR_CONFIG(self, data):
        self.mspy.MOTOR_CONFIG['minthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
        self.mspy.MOTOR_CONFIG['maxthrottle'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000
        self.mspy.MOTOR_CONFIG['mincommand'] = self.mspy.readbytes(data, size=16, unsigned=True) # 0-2000

        self.mspy.MOTOR_CONFIG['motor_count'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.MOTOR_CONFIG['motor_poles'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.MOTOR_CONFIG['use_dshot_telemetry'] = (self.mspy.readbytes(data, size=8, unsigned=True) != 0)
        self.mspy.MOTOR_CONFIG['use_esc_sensor'] = (self.mspy.readbytes(data, size=8, unsigned=True) != 0)

    def process_MSP_COMPASS_CONFIG(self, data):
        self.mspy.COMPASS_CONFIG['mag_declination'] = self.mspy.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000

    def process_MSP_GPS_CONFIG(self, data):
        self.mspy.GPS_CONFIG['provider'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.GPS_CONFIG['ublox_sbas'] = self.mspy.readbytes(data, size=8, unsigned=True)
        
        self.mspy.GPS_CONFIG['auto_config'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.GPS_CONFIG['auto_baud'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPS_RESCUE(self, data):
        self.mspy.GPS_RESCUE['angle']             = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['initialAltitudeM']  = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['descentDistanceM']  = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['rescueGroundspeed'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['throttleMin']       = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['throttleMax']       = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['throttleHover']     = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.GPS_RESCUE['sanityChecks']      = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.GPS_RESCUE['minSats']           = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_RSSI_CONFIG(self, data):
        self.mspy.RSSI_CONFIG['channel'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_MOTOR_3D_CONFIG(self, data):
        self.mspy.MOTOR_3D_CONFIG['deadband3d_low'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.MOTOR_3D_CONFIG['deadband3d_high'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.MOTOR_3D_CONFIG['neutral'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_BOXNAMES(self, data):
        self.mspy.AUX_CONFIG = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.mspy.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B): # ; (delimeter char)
                self.mspy.AUX_CONFIG.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_PIDNAMES(self, data):
        self.mspy.PIDNAMES = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.mspy.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B):  # ; (delimeter char)
                self.mspy.PIDNAMES.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_BOXIDS(self, data):
        self.mspy.AUX_CONFIG_IDS = [] # empty the array as new data is coming in

        for i in range(len(data)):
            self.mspy.AUX_CONFIG_IDS.append(self.mspy.readbytes(data, size=8, unsigned=True))

    def process_MSP_SERVO_CONFIGURATIONS(self, data):
        self.mspy.SERVO_CONFIG = [] # empty the array as new data is coming in
        if (len(data) % 12 == 0):
            for i in range(0, len(data), 12):
                arr = {
                    'min':                      self.mspy.readbytes(data, size=16, unsigned=True),
                    'max':                      self.mspy.readbytes(data, size=16, unsigned=True),
                    'middle':                   self.mspy.readbytes(data, size=16, unsigned=True),
                    'rate':                     self.mspy.readbytes(data, size=8, unsigned=False),
                    'indexOfChannelToForward':  self.mspy.readbytes(data, size=8, unsigned=True),
                    'reversedInputSources':     self.mspy.readbytes(data, size=32, unsigned=True)
                }

                self.mspy.SERVO_CONFIG.append(arr)

    def process_MSP_RC_DEADBAND(self, data):
        self.mspy.RC_DEADBAND_CONFIG['deadband'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RC_DEADBAND_CONFIG['yaw_deadband'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RC_DEADBAND_CONFIG['alt_hold_deadband'] = self.mspy.readbytes(data, size=8, unsigned=True)

        self.mspy.RC_DEADBAND_CONFIG['deadband3d_throttle'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_ALIGNMENT(self, data):
        self.mspy.SENSOR_ALIGNMENT['align_gyro'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SENSOR_ALIGNMENT['align_acc'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SENSOR_ALIGNMENT['align_mag'] = self.mspy.readbytes(data, size=8, unsigned=True)

        if self.mspy.INAV:
            self.mspy.SENSOR_ALIGNMENT['align_opflow'] = self.mspy.readbytes(data, size=8, unsigned=True)
        else:
            self.mspy.SENSOR_ALIGNMENT['gyro_detection_flags'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.SENSOR_ALIGNMENT['gyro_to_use'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.SENSOR_ALIGNMENT['gyro_1_align'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.SENSOR_ALIGNMENT['gyro_2_align'] = self.mspy.readbytes(data, size=8, unsigned=True)

    # def process_MSP_DISPLAYPORT(self, data):

    def process_MSP_SET_RAW_RC(self, data):
        logging.debug('RAW RC values updated')

    def process_MSP_SET_PID(self, data):
        logging.info('PID settings saved')

    def process_MSP_SET_RC_TUNING(self, data):
        logging.info('RC Tuning saved')

    def process_MSP_ACC_CALIBRATION(self, data):
        logging.info('Accel calibration executed')

    def process_MSP_MAG_CALIBRATION(self, data):
        logging.info('Mag calibration executed')

    def process_MSP_SET_MOTOR_CONFIG(self, data):
        logging.info('Motor Configuration saved')

    def process_MSP_SET_GPS_CONFIG(self, data):
        logging.info('GPS Configuration saved')

    def process_MSP_SET_RSSI_CONFIG(self, data):
        logging.info('RSSI Configuration saved')

    def process_MSP_SET_FEATURE_CONFIG(self, data):
        logging.info('Features saved')

    def process_MSP_SET_BEEPER_CONFIG(self, data):
        logging.info('Beeper Configuration saved')

    def process_MSP_RESET_CONF(self, data):
        logging.info('Settings Reset')

    def process_MSP_SELECT_SETTING(self, data):
        logging.info('Profile selected')

    def process_MSP_SET_SERVO_CONFIGURATION(self, data):
        logging.info('Servo Configuration saved')

    def process_MSP_EEPROM_WRITE(self, data):
        logging.info('Settings Saved in EEPROM')

    def process_MSP_SET_CURRENT_METER_CONFIG(self, data):
        logging.info('Amperage Settings saved')

    def process_MSP_SET_VOLTAGE_METER_CONFIG(self, data):
        logging.info('Voltage config saved')
        
    def process_MSP_DEBUG(self, data):
        for i in range(4):
            self.mspy.SENSOR_DATA['debug'][i] = self.mspy.readbytes(data, size=16, unsigned=False)

    def process_MSP2_INAV_DEBUG(self, data):
        for i in range(8):
            self.mspy.SENSOR_DATA['debug'][i] = self.mspy.readbytes(data, size=32, unsigned=False)

    def process_MSP_SET_MOTOR(self, data):
        logging.info('Motor Speeds Updated')

    def process_MSP_UID(self, data):
        for i in range(3):
            self.mspy.CONFIG['uid'][i] = self.mspy.readbytes(data, size=32, unsigned=True)
    
    def process_MSP_ACC_TRIM(self, data):
        self.mspy.CONFIG['accelerometerTrims'][0] = self.mspy.readbytes(data, size=16, unsigned=False) # pitch
        self.mspy.CONFIG['accelerometerTrims'][1] = self.mspy.readbytes(data, size=16, unsigned=False) # roll

    def process_MSP_SET_ACC_TRIM(self, data):
        logging.info('Accelerometer trimms saved.')

    def process_MSP_GPS_SV_INFO(self, data):
        if (len(data) > 0):
            numCh = self.mspy.readbytes(data, size=8, unsigned=True)

            for i in range(numCh):
                self.mspy.GPS_DATA['chn'].append(self.mspy.readbytes(data, size=8, unsigned=True))
                self.mspy.GPS_DATA['svid'].append(self.mspy.readbytes(data, size=8, unsigned=True))
                self.mspy.GPS_DATA['quality'].append(self.mspy.readbytes(data, size=8, unsigned=True))
                self.mspy.GPS_DATA['cno'].append(self.mspy.readbytes(data, size=8, unsigned=True))

    def process_MSP_RX_MAP(self, data):
        self.mspy.RC_MAP = [] # empty the array as new data is coming in

        for i in range(len(data)):
            self.mspy.RC_MAP.append(self.mspy.readbytes(data, size=8, unsigned=True))

    def process_MSP_SET_RX_MAP(self, data):
        logging.debug('RCMAP saved')
        
    def process_MSP_MIXER_CONFIG(self, data):
        self.mspy.MIXER_CONFIG['mixer'] = self.mspy.readbytes(data, size=8, unsigned=True)
        if not self.mspy.INAV:                    
            self.mspy.MIXER_CONFIG['reverseMotorDir'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_FEATURE_CONFIG(self, data):
        self.mspy.FEATURE_CONFIG['featuremask']  = self.mspy.readbytes(data, size=32, unsigned=True)
        for idx in range(32):
            enabled = self.mspy.bit_check(self.mspy.FEATURE_CONFIG['featuremask'], idx)
            if idx in self.mspy.FEATURE_CONFIG['features'].keys():
                self.mspy.FEATURE_CONFIG['features'][idx]['enabled'] = enabled
            else:
                self.mspy.FEATURE_CONFIG['features'][idx] = {'enabled': enabled}

    def process_MSP_BEEPER_CONFIG(self, data):
        self.mspy.BEEPER_CONFIG['beepers'] = self.mspy.readbytes(data, size=32, unsigned=True)
            
        self.mspy.BEEPER_CONFIG['dshotBeaconTone'] = self.mspy.readbytes(data, size=8, unsigned=True)

        self.mspy.BEEPER_CONFIG['dshotBeaconConditions'] = self.mspy.readbytes(data, size=32, unsigned=True)

    def process_MSP_BOARD_ALIGNMENT_CONFIG(self, data):
        self.mspy.BOARD_ALIGNMENT_CONFIG['roll'] = self.mspy.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.mspy.BOARD_ALIGNMENT_CONFIG['pitch'] = self.mspy.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.mspy.BOARD_ALIGNMENT_CONFIG['yaw'] = self.mspy.readbytes(data, size=16, unsigned=False) # -180 - 360

    def process_MSP_SET_REBOOT(self, data):
        rebootType = self.mspy.readbytes(data, size=8, unsigned=True)

        if ((rebootType == self.mspy.REBOOT_TYPES['MSC']) or (rebootType == self.mspy.REBOOT_TYPES['MSC_UTC'])):
            if (self.mspy.readbytes(data, size=8, unsigned=True) == 0):
                logging.warning('Storage device not ready for reboot.')

        logging.info('Reboot request accepted')

    def process_MSP_API_VERSION(self, data):
        self.mspy.CONFIG['mspProtocolVersion'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.CONFIG['apiVersion'] = str(self.mspy.readbytes(data, size=8, unsigned=True)) + '.' + str(self.mspy.readbytes(data, size=8, unsigned=True)) + '.0'

    def process_MSP_FC_VARIANT(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.mspy.readbytes(data, size=8, unsigned=True))
        self.mspy.CONFIG['flightControllerIdentifier'] = identifier

    def process_MSP_FC_VERSION(self, data):
        self.mspy.CONFIG['flightControllerVersion'] =  str(self.mspy.readbytes(data, size=8, unsigned=True)) + '.'
        self.mspy.CONFIG['flightControllerVersion'] += str(self.mspy.readbytes(data, size=8, unsigned=True)) + '.'
        self.mspy.CONFIG['flightControllerVersion'] += str(self.mspy.readbytes(data, size=8, unsigned=True))

    def process_MSP_BUILD_INFO(self, data):
        dateLength = 11
        buff = []
        for i in range(dateLength):
            buff.append(self.mspy.readbytes(data, size=8, unsigned=True))
        
        buff.append(32) # ascii space

        timeLength = 8
        for i in range(timeLength):
            buff.append(self.mspy.readbytes(data, size=8, unsigned=True))

        self.mspy.CONFIG['buildInfo'] = bytearray(buff).decode("utf-8")

    def process_MSP_BOARD_INFO(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.mspy.readbytes(data, size=8, unsigned=True))

        self.mspy.CONFIG['boardIdentifier'] = identifier
        self.mspy.CONFIG['boardVersion'] = self.mspy.readbytes(data, size=16, unsigned=True)

        self.mspy.CONFIG['boardType'] = self.mspy.readbytes(data, size=8, unsigned=True)

        self.mspy.CONFIG['targetName'] = ""

        self.mspy.CONFIG['commCapabilities'] = self.mspy.readbytes(data, size=8, unsigned=True)

        length = self.mspy.readbytes(data, size=8, unsigned=True)
        
        for i in range(length):
            self.mspy.CONFIG['targetName'] += chr(self.mspy.readbytes(data, size=8, unsigned=True))

        self.mspy.CONFIG['boardName'] = ""
        self.mspy.CONFIG['manufacturerId'] = ""
        self.mspy.CONFIG['signature'] = []
        self.mspy.CONFIG['boardName'] = ""
        self.mspy.CONFIG['mcuTypeId'] = ""

        if data:
            length = self.mspy.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.mspy.CONFIG['boardName'] += chr(self.mspy.readbytes(data, size=8, unsigned=True))

            length = self.mspy.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.mspy.CONFIG['manufacturerId'] += chr(self.mspy.readbytes(data, size=8, unsigned=True))

            for i in range(self.mspy.SIGNATURE_LENGTH):
                self.mspy.CONFIG['signature'].append(self.mspy.readbytes(data, size=8, unsigned=True))

            self.mspy.CONFIG['mcuTypeId'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_NAME(self, data):
        self.mspy.CONFIG['name'] = ''
    
        while len(data)>0:
            char = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.CONFIG['name'] += chr(char)

    # def process_MSP_SET_CHANNEL_FORWARDING(self, data):
    #     logging.info('Channel forwarding saved')

    def process_MSP_CF_SERIAL_CONFIG(self, data):
        self.mspy.SERIAL_CONFIG['ports'] = []
        bytesPerPort = 1 + 2 + (1 * 4)
        serialPortCount = int(len(data) / bytesPerPort)

        for i in range(serialPortCount):
            serialPort = {
                'identifier': self.mspy.readbytes(data, size=8, unsigned=True),
                'functions': self.mspy.serialPortFunctionMaskToFunctions(self.mspy.readbytes(data, size=16, unsigned=True)),
                'msp_baudrate': self.mspy.BAUD_RATES[self.mspy.readbytes(data, size=8, unsigned=True)],
                'gps_baudrate': self.mspy.BAUD_RATES[self.mspy.readbytes(data, size=8, unsigned=True)],
                'telemetry_baudrate': self.mspy.BAUD_RATES[self.mspy.readbytes(data, size=8, unsigned=True)],
                'blackbox_baudrate': self.mspy.BAUD_RATES[self.mspy.readbytes(data, size=8, unsigned=True)]
            }

            self.mspy.SERIAL_CONFIG['ports'].append(serialPort)

    def process_MSP_SET_CF_SERIAL_CONFIG(self, data):
        logging.info('Serial config saved')

    def process_MSP_MODE_RANGES(self, data):
        self.mspy.MODE_RANGES = [] # empty the array as new data is coming in

        modeRangeCount = int(len(data) / 4) # 4 bytes per item.

        for i in range(modeRangeCount):
            modeRange = {
                'id': self.mspy.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.mspy.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.mspy.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.mspy.readbytes(data, size=8, unsigned=True) * 25)
                            }
                }
            self.mspy.MODE_RANGES.append(modeRange)

    def process_MSP_MODE_RANGES_EXTRA(self, data):
        self.mspy.MODE_RANGES_EXTRA = [] # empty the array as new data is coming in

        modeRangeExtraCount = self.mspy.readbytes(data, size=8, unsigned=True)

        for i in range(modeRangeExtraCount):
            modeRangeExtra = {
                'id': self.mspy.readbytes(data, size=8, unsigned=True),
                'modeLogic': self.mspy.readbytes(data, size=8, unsigned=True),
                'linkedTo': self.mspy.readbytes(data, size=8, unsigned=True)
            }
            self.mspy.MODE_RANGES_EXTRA.append(modeRangeExtra)

    def process_MSP_ADJUSTMENT_RANGES(self, data):
        self.mspy.ADJUSTMENT_RANGES = [] # empty the array as new data is coming in

        adjustmentRangeCount = int(len(data) / 6) # 6 bytes per item.

        for i in range(adjustmentRangeCount):
            adjustmentRange = {
                'slotIndex': self.mspy.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.mspy.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.mspy.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.mspy.readbytes(data, size=8, unsigned=True) * 25)
                },
                'adjustmentFunction': self.mspy.readbytes(data, size=8, unsigned=True),
                'auxSwitchChannelIndex': self.mspy.readbytes(data, size=8, unsigned=True)
            }
            self.mspy.ADJUSTMENT_RANGES.append(adjustmentRange)

    def process_MSP_RX_CONFIG(self, data):
        self.mspy.RX_CONFIG['serialrx_provider'] = self.mspy.readbytes(data, size=8, unsigned=True)
        # maxcheck for INAV
        self.mspy.RX_CONFIG['stick_max'] = self.mspy.readbytes(data, size=16, unsigned=True)
        # midrc for INAV
        self.mspy.RX_CONFIG['stick_center'] = self.mspy.readbytes(data, size=16, unsigned=True)
        # mincheck for INAV
        self.mspy.RX_CONFIG['stick_min'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.RX_CONFIG['spektrum_sat_bind'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RX_CONFIG['rx_min_usec'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.RX_CONFIG['rx_max_usec'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.RX_CONFIG['rcInterpolation'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RX_CONFIG['rcInterpolationInterval'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RX_CONFIG['airModeActivateThreshold'] = self.mspy.readbytes(data, size=16, unsigned=True)  
        # spirx_protocol for INAV
        self.mspy.RX_CONFIG['rxSpiProtocol'] = self.mspy.readbytes(data, size=8, unsigned=True)
        # spirx_id for INAV
        self.mspy.RX_CONFIG['rxSpiId'] = self.mspy.readbytes(data, size=32, unsigned=True)
        # spirx_channel_count for INAV
        self.mspy.RX_CONFIG['rxSpiRfChannelCount'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.RX_CONFIG['fpvCamAngleDegrees'] = self.mspy.readbytes(data, size=8, unsigned=True)
        if self.mspy.INAV:
            self.mspy.RX_CONFIG['receiver_type'] = self.mspy.readbytes(data, size=8, unsigned=True)
        else:
            self.mspy.RX_CONFIG['rcInterpolationChannels'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RX_CONFIG['rcSmoothingType'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RX_CONFIG['rcSmoothingInputCutoff'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RX_CONFIG['rcSmoothingDerivativeCutoff'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RX_CONFIG['rcSmoothingInputType'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.RX_CONFIG['rcSmoothingDerivativeType'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_FAILSAFE_CONFIG(self, data):
        self.mspy.FAILSAFE_CONFIG['failsafe_delay'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.FAILSAFE_CONFIG['failsafe_off_delay'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.FAILSAFE_CONFIG['failsafe_throttle'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FAILSAFE_CONFIG['failsafe_switch_mode'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.FAILSAFE_CONFIG['failsafe_throttle_low_delay'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FAILSAFE_CONFIG['failsafe_procedure'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_RXFAIL_CONFIG(self, data):
        self.mspy.RXFAIL_CONFIG = [] # empty the array as new data is coming in

        channelCount = int(len(data) / 3)
        for i in range(channelCount):
            rxfailChannel = {
                'mode':  self.mspy.readbytes(data, size=8, unsigned=True),
                'value': self.mspy.readbytes(data, size=16, unsigned=True)
            }
            self.mspy.RXFAIL_CONFIG.append(rxfailChannel)

    def process_MSP_ADVANCED_CONFIG(self, data):
        self.mspy.PID_ADVANCED_CONFIG['gyro_sync_denom'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.PID_ADVANCED_CONFIG['pid_process_denom'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.PID_ADVANCED_CONFIG['use_unsyncedPwm'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.PID_ADVANCED_CONFIG['fast_pwm_protocol'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.PID_ADVANCED_CONFIG['motor_pwm_rate'] = self.mspy.readbytes(data, size=16, unsigned=True)

        self.mspy.PID_ADVANCED_CONFIG['digitalIdlePercent'] = self.mspy.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_FILTER_CONFIG(self, data):
        self.mspy.FILTER_CONFIG['gyro_lowpass_hz'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.FILTER_CONFIG['dterm_lowpass_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FILTER_CONFIG['yaw_lowpass_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        
        self.mspy.FILTER_CONFIG['gyro_notch_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FILTER_CONFIG['gyro_notch_cutoff'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FILTER_CONFIG['dterm_notch_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FILTER_CONFIG['dterm_notch_cutoff'] = self.mspy.readbytes(data, size=16, unsigned=True)

        self.mspy.FILTER_CONFIG['gyro_notch2_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.FILTER_CONFIG['gyro_notch2_cutoff'] = self.mspy.readbytes(data, size=16, unsigned=True)

        if not self.mspy.INAV:
            self.mspy.FILTER_CONFIG['dterm_lowpass_type'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.FILTER_CONFIG['gyro_hardware_lpf'] = self.mspy.readbytes(data, size=8, unsigned=True)
            
            self.mspy.readbytes(data, size=8, unsigned=True) # must consume this byte

            self.mspy.FILTER_CONFIG['gyro_lowpass_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['gyro_lowpass2_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['gyro_lowpass_type'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.FILTER_CONFIG['gyro_lowpass2_type'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.FILTER_CONFIG['dterm_lowpass2_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)

            self.mspy.FILTER_CONFIG['gyro_32khz_hardware_lpf'] = 0

            self.mspy.FILTER_CONFIG['dterm_lowpass2_type'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.FILTER_CONFIG['gyro_lowpass_dyn_min_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['gyro_lowpass_dyn_max_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['dterm_lowpass_dyn_min_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['dterm_lowpass_dyn_max_hz'] = self.mspy.readbytes(data, size=16, unsigned=True)
        else:
            self.mspy.FILTER_CONFIG['accNotchHz'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['accNotchCutoff'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.FILTER_CONFIG['gyroStage2LowpassHz'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_SET_PID_ADVANCED(self, data):
        logging.info("Advanced PID settings saved")

    def process_MSP_PID_ADVANCED(self, data):
        self.mspy.ADVANCED_TUNING['rollPitchItermIgnoreRate'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.ADVANCED_TUNING['yawItermIgnoreRate'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.ADVANCED_TUNING['yaw_p_limit'] = self.mspy.readbytes(data, size=16, unsigned=True)
        self.mspy.ADVANCED_TUNING['deltaMethod'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.ADVANCED_TUNING['vbatPidCompensation'] = self.mspy.readbytes(data, size=8, unsigned=True)
        if not self.mspy.INAV:
            self.mspy.ADVANCED_TUNING['feedforwardTransition'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.ADVANCED_TUNING['dtermSetpointWeight'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['toleranceBand'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['toleranceBandReduction'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['itermThrottleGain'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['pidMaxVelocity'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['pidMaxVelocityYaw'] = self.mspy.readbytes(data, size=16, unsigned=True)

            self.mspy.ADVANCED_TUNING['levelAngleLimit'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['levelSensitivity'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.ADVANCED_TUNING['itermThrottleThreshold'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['itermAcceleratorGain'] = self.mspy.readbytes(data, size=16, unsigned=True)

            self.mspy.ADVANCED_TUNING['dtermSetpointWeight'] = self.mspy.readbytes(data, size=16, unsigned=True)

            self.mspy.ADVANCED_TUNING['itermRotation'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['smartFeedforward'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['itermRelax'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['itermRelaxType'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['absoluteControlGain'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['throttleBoost'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['acroTrainerAngleLimit'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['feedforwardRoll']  = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['feedforwardPitch'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['feedforwardYaw']   = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['antiGravityMode']  = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.ADVANCED_TUNING['dMinRoll'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['dMinPitch'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['dMinYaw'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['dMinGain'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['dMinAdvance'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['useIntegratedYaw'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['integratedYawRelax'] = self.mspy.readbytes(data, size=8, unsigned=True)
        else:
            self.mspy.ADVANCED_TUNING['setpointRelaxRatio'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['dtermSetpointWeight'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['pidSumLimit'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['itermThrottleGain'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.ADVANCED_TUNING['axisAccelerationLimitRollPitch'] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.ADVANCED_TUNING['axisAccelerationLimitYaw'] = self.mspy.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_CONFIG(self, data):
        self.mspy.SENSOR_CONFIG['acc_hardware'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SENSOR_CONFIG['baro_hardware'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SENSOR_CONFIG['mag_hardware'] = self.mspy.readbytes(data, size=8, unsigned=True)
        if self.mspy.INAV:
            self.mspy.SENSOR_CONFIG['pitot'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.SENSOR_CONFIG['rangefinder'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.SENSOR_CONFIG['opflow'] = self.mspy.readbytes(data, size=8, unsigned=True)

    def process_MSP_DATAFLASH_SUMMARY(self, data):
        flags = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.DATAFLASH['ready'] = ((flags & 1) != 0)
        self.mspy.DATAFLASH['supported'] = ((flags & 2) != 0)
        self.mspy.DATAFLASH['sectors'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.DATAFLASH['totalSize'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.DATAFLASH['usedSize'] = self.mspy.readbytes(data, size=32, unsigned=True)
        # update_dataflash_global();

    def process_MSP_DATAFLASH_ERASE(self, data):
        logging.info("Data flash erase begun...")

    def process_MSP_SDCARD_SUMMARY(self, data):
        flags = self.mspy.readbytes(data, size=8, unsigned=True)

        self.mspy.SDCARD['supported'] = ((flags & 0x01) != 0)
        self.mspy.SDCARD['state'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SDCARD['filesystemLastError'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.SDCARD['freeSizeKB'] = self.mspy.readbytes(data, size=32, unsigned=True)
        self.mspy.SDCARD['totalSizeKB'] = self.mspy.readbytes(data, size=32, unsigned=True)

    def process_MSP_BLACKBOX_CONFIG(self, data):
        if not self.mspy.INAV:
            self.mspy.BLACKBOX['supported'] = (self.mspy.readbytes(data, size=8, unsigned=True) & 1) != 0
            self.mspy.BLACKBOX['blackboxDevice'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.BLACKBOX['blackboxRateNum'] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.BLACKBOX['blackboxRateDenom'] = self.mspy.readbytes(data, size=8, unsigned=True)

            self.mspy.BLACKBOX['blackboxPDenom'] = self.mspy.readbytes(data, size=16, unsigned=True)
        else:
            pass # API no longer supported (INAV 2.3.0)

    def process_MSP_SET_BLACKBOX_CONFIG(self, data):
        logging.info("Blackbox config saved")

    def process_MSP_MOTOR_TELEMETRY(self, data):
        motorCount = self.mspy.readbytes(data, size=8, unsigned=True)
        for i in range(motorCount):
            self.mspy.MOTOR_TELEMETRY_DATA['rpm'][i] = self.mspy.readbytes(data, size=32, unsigned=True)
            self.mspy.MOTOR_TELEMETRY_DATA['invalidPercent'][i] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MOTOR_TELEMETRY_DATA['temperature'][i] = self.mspy.readbytes(data, size=8, unsigned=True)
            self.mspy.MOTOR_TELEMETRY_DATA['voltage'][i] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MOTOR_TELEMETRY_DATA['current'][i] = self.mspy.readbytes(data, size=16, unsigned=True)
            self.mspy.MOTOR_TELEMETRY_DATA['consumption'][i] = self.mspy.readbytes(data, size=16, unsigned=True)

    # TODO: This changed and it will need to check the BF version to decode things correctly
    # def process_MSP_TRANSPONDER_CONFIG(self, data):
    #     bytesRemaining = len(data)

    #     providerCount = self.mspy.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.mspy.TRANSPONDER['supported'] = providerCount > 0
    #     self.mspy.TRANSPONDER['providers'] = []

    #     for i in range(providerCount):
    #         provider = {
    #             'id': self.mspy.readbytes(data, size=8, unsigned=True),
    #             'dataLength': self.mspy.readbytes(data, size=8, unsigned=True)
    #         }
    #         bytesRemaining -= 2

    #         self.mspy.TRANSPONDER['providers'].append(provider)

    #     self.mspy.TRANSPONDER['provider'] = self.mspy.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.mspy.TRANSPONDER['data'] = []
    #     for i in range(bytesRemaining):
    #         self.mspy.TRANSPONDER['data'].append(self.mspy.readbytes(data, size=8, unsigned=True))

    def process_MSP_SET_TRANSPONDER_CONFIG(self, data):
        logging.info("Transponder config saved")

    def process_MSP_SET_MODE_RANGE(self, data):
        logging.info('Mode range saved')

    def process_MSP_SET_ADJUSTMENT_RANGE(self, data):
        logging.info('Adjustment range saved')
        
    def process_MSP_SET_BOARD_ALIGNMENT_CONFIG(self, data):
        logging.info('Board alignment saved')
        
    def process_MSP_PID_CONTROLLER(self, data):
        self.mspy.PID['controller'] = self.mspy.readbytes(data, size=8, unsigned=True)
        
    def process_MSP_SET_PID_CONTROLLER(self, data):
        logging.info('PID controller changed')
        
    def process_MSP_SET_LOOP_TIME(self, data):
        logging.info('Looptime saved')
        
    def process_MSP_SET_ARMING_CONFIG(self, data):
        logging.info('Arming config saved')
        
    def process_MSP_SET_RESET_CURR_PID(self, data):
        logging.info('Current PID profile reset')
        
    def process_MSP_SET_MOTOR_3D_CONFIG(self, data):
        logging.info('3D settings saved')
        
    def process_MSP_SET_MIXER_CONFIG(self, data):
        logging.info('Mixer config saved')
        
    def process_MSP_SET_RC_DEADBAND(self, data):
        logging.info('Rc controls settings saved')
        
    def process_MSP_SET_SENSOR_ALIGNMENT(self, data):
        logging.info('Sensor alignment saved')
        
    def process_MSP_SET_RX_CONFIG(self, data):
        logging.info('Rx config saved')
        
    def process_MSP_SET_RXFAIL_CONFIG(self, data):
        logging.info('Rxfail config saved')
        
    def process_MSP_SET_FAILSAFE_CONFIG(self, data):
        logging.info('Failsafe config saved')
        
    def process_MSP_OSD_CONFIG(self, data):
        logging.info('OSD_CONFIG received')
        
    def process_MSP_SET_OSD_CONFIG(self, data):
        logging.info('OSD config set')
        
    def process_MSP_OSD_CHAR_READ(self, data):
        logging.info('OSD char received')
        
    def process_MSP_OSD_CHAR_WRITE(self, data):
        logging.info('OSD char uploaded')
        
    def process_MSP_VTX_CONFIG(self, data):
        logging.info('VTX_CONFIG received')
        
    def process_MSP_SET_VTX_CONFIG(self, data):
        logging.info('VTX_CONFIG set')
        
    def process_MSP_SET_NAME(self, data):
        logging.info('Name set')
        
    def process_MSP_SET_FILTER_CONFIG(self, data):
        logging.info('Filter config set')
        
    def process_MSP_SET_ADVANCED_CONFIG(self, data):
        logging.info('Advanced config parameters set')
        
    def process_MSP_SET_SENSOR_CONFIG(self, data):
        logging.info('Sensor config parameters set')
        
    def process_MSP_COPY_PROFILE(self, data):
        logging.info('Copy profile')
        
    def process_MSP_ARMING_DISABLE(self, data):
        logging.info('Arming disable')
        
    def process_MSP_SET_RTC(self, data):
        logging.info('Real time clock set')


    # unavlib new functions below

    def process_MSP_NAV_STATUS(self,data):
        self.mspy.NAV_STATUS['mode'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.NAV_STATUS['state'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.NAV_STATUS['active_wp_action'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.NAV_STATUS['active_wp_number'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.NAV_STATUS['error'] = self.mspy.readbytes(data, size=8, unsigned=True)
        self.mspy.NAV_STATUS['heading_hold_target'] = self.mspy.readbytes(data, size=16, unsigned=True)