
import logging
import struct
import time
import sys
import serial
from select import select

class processMSP():
    def __init__(self, mspy_instance):
        self.board = mspy_instance

    def process_MSP_STATUS(self, data):
        self.board.CONFIG['cycleTime'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['i2cError'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['activeSensors'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['mode'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.CONFIG['profile'] = self.board.readbytes(data, size=8, unsigned=True)
        
    def process_MSP_STATUS_EX(self, data):
        self.board.CONFIG['cycleTime'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['i2cError'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['activeSensors'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['mode'] = self.board.readbytes(data, size=32, unsigned=True)

        self.board.CONFIG['profile'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.CONFIG['cpuload'] = self.board.readbytes(data, size=16, unsigned=True)
        
        if not self.board.INAV:
            self.board.CONFIG['numProfiles'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.CONFIG['rateProfile'] = self.board.readbytes(data, size=8, unsigned=True)

            # Read flight mode flags
            byteCount = self.board.readbytes(data, size=8, unsigned=True)
            self.board.CONFIG['flightModeFlags'] = [] # this was not implemented on betaflight-configurator
            for _ in range(byteCount):
                # betaflight-configurator would just discard these bytes
                self.board.CONFIG['flightModeFlags'].append(self.board.readbytes(data, size=8, unsigned=True))

            # Read arming disable flags
            self.board.CONFIG['armingDisableCount'] = self.board.readbytes(data, size=8, unsigned=True) # Flag count
            self.board.CONFIG['armingDisableFlags'] = self.board.readbytes(data, size=32, unsigned=True)
        else:
            self.board.CONFIG['armingDisableFlags'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP2_INAV_STATUS(self, data):
        self.board.CONFIG['cycleTime'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['i2cError'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['activeSensors'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['cpuload'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.CONFIG['profile'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.CONFIG['armingDisableFlags'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.CONFIG['mode'] = self.board.readbytes(data, size=32, unsigned=True)
        

    def process_MSP_RAW_IMU(self, data):
        # /512 for mpu6050, /256 for mma
        # currently we are unable to differentiate between the sensor types, so we are going with 512
        # And what about SENSOR_CONFIG???
        self.board.SENSOR_DATA['accelerometer'][0] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['accelerometer'][1] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['accelerometer'][2] = self.board.readbytes(data, size=16, unsigned=False)

        # properly scaled (INAV and BF use the same * (4 / 16.4))
        # but this is supposed to be RAW, so raw it is!
        self.board.SENSOR_DATA['gyroscope'][0] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['gyroscope'][1] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['gyroscope'][2] = self.board.readbytes(data, size=16, unsigned=False)

        # no clue about scaling factor (/1090), so raw
        self.board.SENSOR_DATA['magnetometer'][0] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['magnetometer'][1] = self.board.readbytes(data, size=16, unsigned=False)
        self.board.SENSOR_DATA['magnetometer'][2] = self.board.readbytes(data, size=16, unsigned=False)

    def process_MSP_SERVO(self, data):
        servoCount = int(len(data) / 2)
        self.board.SERVO_DATA = [self.board.readbytes(data, size=16, unsigned=True) for _ in range(servoCount)]

    def process_MSP_MOTOR(self, data):
        motorCount = int(len(data) / 2)
        self.board.MOTOR_DATA = [self.board.readbytes(data, size=16, unsigned=True) for i in range(motorCount)]

    def process_MSP_RC(self, data):
        n_channels = int(len(data) / 2)
        self.board.RC['active_channels'] = n_channels
        self.board.RC['channels'] = [self.board.readbytes(data, size=16, unsigned=True) for i in range(n_channels)]

    def process_MSP_RAW_GPS(self, data):
        self.board.GPS_DATA['fix'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_DATA['numSat'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_DATA['lat'] = self.board.readbytes(data, size=32, unsigned=False)
        self.board.GPS_DATA['lon'] = self.board.readbytes(data, size=32, unsigned=False)
        self.board.GPS_DATA['alt'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['speed'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['ground_course'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['hdop'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_COMP_GPS(self, data):
        self.board.GPS_DATA['distanceToHome'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['directionToHome'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['update'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPSSTATISTICS(self, data):
        self.board.GPS_DATA['messageDt'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['errors'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.GPS_DATA['timeouts'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.GPS_DATA['packetCount'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.GPS_DATA['hdop'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['eph'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_DATA['epv'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_ATTITUDE(self, data):
        self.board.SENSOR_DATA['kinematics'][0] = self.board.readbytes(data, size=16, unsigned=False) / 10.0 # x
        self.board.SENSOR_DATA['kinematics'][1] = self.board.readbytes(data, size=16, unsigned=False) / 10.0 # y
        self.board.SENSOR_DATA['kinematics'][2] = self.board.readbytes(data, size=16, unsigned=False) # z

    def process_MSP_ALTITUDE(self, data):
        self.board.SENSOR_DATA['altitude'] = round((self.board.readbytes(data, size=32, unsigned=False) / 100.0), 2) # correct scale factor
        self.board.SENSOR_DATA['altitude_vel'] = round(self.board.readbytes(data, size=16, unsigned=False) / 100.0, 2)
        # Baro altitude => self.board.readbytes(data, size=32, unsigned=True)


    def process_MSP_SONAR(self, data):
        self.board.SENSOR_DATA['sonar'] = self.board.readbytes(data, size=32, unsigned=False)

    def process_MSP_ANALOG(self, data):
        self.board.ANALOG['voltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10.0
        self.board.ANALOG['mAhdrawn'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.ANALOG['rssi'] = self.board.readbytes(data, size=16, unsigned=True) # 0-1023
        self.board.ANALOG['amperage'] = self.board.readbytes(data, size=16, unsigned=False) / 100 # A
        self.board.ANALOG['last_received_timestamp'] = int(time.time()) # why not monotonic? where is time synchronized?
        if not self.board.INAV:
            self.board.ANALOG['voltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
    
    def process_MSP2_INAV_ANALOG(self, data):
        if self.board.INAV:
            tmp = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ANALOG['battery_full_when_plugged_in'] = True if (tmp & 1) else False
            self.board.ANALOG['use_capacity_thresholds'] = True if ((tmp & 2) >> 1) else False
            self.board.ANALOG['battery_state'] = (tmp & 12) >> 2
            self.board.ANALOG['cell_count'] = (tmp & 0xF0) >> 4

            self.board.ANALOG['voltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
            self.board.ANALOG['amperage'] = self.board.readbytes(data, size=16, unsigned=True) / 100 # A
            self.board.ANALOG['power'] = self.board.readbytes(data, size=32, unsigned=True) / 100
            self.board.ANALOG['mAhdrawn'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.ANALOG['mWhdrawn'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.ANALOG['battery_remaining_capacity'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.ANALOG['battery_percentage'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ANALOG['rssi'] = self.board.readbytes(data, size=16, unsigned=True) # 0-1023

            # TODO: update both BF and INAV variables
            self.board.BATTERY_STATE['cellCount'] = self.board.ANALOG['cell_count']

    def process_MSP_VOLTAGE_METERS(self, data):
        total_bytes_per_meter = (8+8)/8 # just to make it clear where it comes from...
        self.board.VOLTAGE_METERS = [{'id':self.board.readbytes(data, size=8, unsigned=True),
                                'voltage':self.board.readbytes(data, size=8, unsigned=True) / 10.0
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_CURRENT_METERS(self, data):
        total_bytes_per_meter = (8+16+16)/8 # just to make it clear where it comes from...
        self.board.CURRENT_METERS = [{'id':self.board.readbytes(data, size=8, unsigned=True),
                                'mAhDrawn':self.board.readbytes(data, size=16, unsigned=True), # mAh
                                'amperage':self.board.readbytes(data, size=16, unsigned=True) / 1000 # A
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_BATTERY_STATE(self, data):
        self.board.BATTERY_STATE['cellCount'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.BATTERY_STATE['capacity'] = self.board.readbytes(data, size=16, unsigned=True) # mAh
        # BATTERY_STATE.voltage = data.readU8() / 10.0; // V
        self.board.BATTERY_STATE['mAhDrawn'] = self.board.readbytes(data, size=16, unsigned=True) # mAh
        self.board.BATTERY_STATE['amperage'] = self.board.readbytes(data, size=16, unsigned=True) / 100 # A
        self.board.BATTERY_STATE['batteryState'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.BATTERY_STATE['voltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100 # V

    def process_MSP_VOLTAGE_METER_CONFIG(self, data):
        self.board.VOLTAGE_METER_CONFIGS = []
        if self.board.INAV:
            voltageMeterConfig = {}
            voltageMeterConfig['vbatscale'] = self.board.readbytes(data, size=8, unsigned=True)/10
            self.board.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)
            self.board.BATTERY_CONFIG['vbatmincellvoltage'] = self.board.readbytes(data, size=8, unsigned=True)/10
            self.board.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True)/10
            self.board.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True)/10
        else:
            voltage_meter_count = self.board.readbytes(data, size=8, unsigned=True)

            for i in range(voltage_meter_count):
                subframe_length = self.board.readbytes(data, size=8, unsigned=True)
                if (subframe_length != 5):
                    for j in range(subframe_length):
                        self.board.readbytes(data, size=8, unsigned=True)
                else:
                    voltageMeterConfig = {}
                    voltageMeterConfig['id'] = self.board.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['sensorType'] = self.board.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatscale'] = self.board.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivval'] = self.board.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivmultiplier'] = self.board.readbytes(data, size=8, unsigned=True)

                    self.board.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)

    def process_MSP_CURRENT_METER_CONFIG(self, data):
        self.board.CURRENT_METER_CONFIGS = []
        if self.board.INAV:
            currentMeterConfig = {}
            currentMeterConfig['scale'] = self.board.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['offset'] = self.board.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['sensorType'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.CURRENT_METER_CONFIGS.append(currentMeterConfig)
            self.board.BATTERY_CONFIG['capacity'] = self.board.readbytes(data, size=16, unsigned=True)
        else:
            current_meter_count = self.board.readbytes(data, size=8, unsigned=True)
            for i in range(current_meter_count):
                currentMeterConfig = {}
                subframe_length = self.board.readbytes(data, size=8, unsigned=True)

                if (subframe_length != 6):
                    for j in range(subframe_length):
                        self.board.readbytes(data, size=8, unsigned=True)
                else:
                    currentMeterConfig['id'] = self.board.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['sensorType'] = self.board.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['scale'] = self.board.readbytes(data, size=16, unsigned=False)
                    currentMeterConfig['offset'] = self.board.readbytes(data, size=16, unsigned=False)

                    self.board.CURRENT_METER_CONFIGS.append(currentMeterConfig)

    def process_MSP_BATTERY_CONFIG(self, data):
        self.board.BATTERY_CONFIG['vbatmincellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.board.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.board.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.board.BATTERY_CONFIG['capacity'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.BATTERY_CONFIG['voltageMeterSource'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.BATTERY_CONFIG['currentMeterSource'] = self.board.readbytes(data, size=8, unsigned=True)

        self.board.BATTERY_CONFIG['vbatmincellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
        self.board.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
        self.board.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_RC_TUNING(self, data):
        self.board.RC_TUNING['RC_RATE'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.board.RC_TUNING['RC_EXPO'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.board.RC_TUNING['roll_pitch_rate'] = 0
        self.board.RC_TUNING['roll_rate'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.board.RC_TUNING['pitch_rate'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.board.RC_TUNING['yaw_rate'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.board.RC_TUNING['dynamic_THR_PID'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.board.RC_TUNING['throttle_MID'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.board.RC_TUNING['throttle_EXPO'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.board.RC_TUNING['dynamic_THR_breakpoint'] = self.board.readbytes(data, size=16, unsigned=True)

        self.board.RC_TUNING['RC_YAW_EXPO'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        if not self.board.INAV:
            self.board.RC_TUNING['rcYawRate'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.board.RC_TUNING['rcPitchRate'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)
            self.board.RC_TUNING['RC_PITCH_EXPO'] = round((self.board.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.board.RC_TUNING['throttleLimitType'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RC_TUNING['throttleLimitPercent'] = self.board.readbytes(data, size=8, unsigned=True)

            if int("".join((self.board.CONFIG['apiVersion'].rsplit('.')))) >= 1420:
                self.board.RC_TUNING['roll_rate_limit'] = self.board.readbytes(data, size=16, unsigned=True)
                self.board.RC_TUNING['pitch_rate_limit'] = self.board.readbytes(data, size=16, unsigned=True)
                self.board.RC_TUNING['yaw_rate_limit'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_PID(self, data):
        self.board.PIDs = [
            [
                self.board.readbytes(data, size=8, unsigned=True) for _ in range(3)
            ] 
            for _ in range(int(len(data)/3))
        ]

    def process_MSP2_PID(self, data):
        self.board.PIDs = [
            [
                self.board.readbytes(data, size=8, unsigned=True) for _ in range(4)
            ] 
            for _ in range(int(len(data)/4))
        ]

    def process_MSP_ARMING_CONFIG(self, data):
        self.board.ARMING_CONFIG['auto_disarm_delay'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.ARMING_CONFIG['disarm_kill_switch'] = self.board.readbytes(data, size=8, unsigned=True)
        if not self.board.INAV:
            self.board.ARMING_CONFIG['small_angle'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_LOOP_TIME(self, data):
        if self.board.INAV:
            self.board.FC_CONFIG['loopTime'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_MISC(self, data): # 22 bytes
        if self.board.INAV:
            self.board.MISC['midrc'] = self.board.RX_CONFIG['midrc'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MISC['minthrottle'] = self.board.MOTOR_CONFIG['minthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['maxthrottle'] = self.board.MOTOR_CONFIG['maxthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['mincommand'] = self.board.MOTOR_CONFIG['mincommand'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['failsafe_throttle'] = self.board.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.board.MISC['gps_type'] = self.board.GPS_CONFIG['provider'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['sensors_baudrate'] = self.board.MISC['gps_baudrate'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['gps_ubx_sbas'] = self.board.GPS_CONFIG['ublox_sbas'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['multiwiicurrentoutput'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['rssi_channel'] = self.board.RSSI_CONFIG['channel'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['placeholder2'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.COMPASS_CONFIG['mag_declination'] = self.board.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000
            
            self.board.MISC['mag_declination'] = self.board.COMPASS_CONFIG['mag_declination']*10

            self.board.MISC['vbatscale'] = self.board.readbytes(data, size=8, unsigned=True) # 10-200
            self.board.MISC['vbatmincellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.board.MISC['vbatmaxcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.board.MISC['vbatwarningcellvoltage'] = self.board.readbytes(data, size=8, unsigned=True) / 10 # 10-50

    def process_MSP2_INAV_MISC(self, data):
        if self.board.INAV:
            self.board.MISC['midrc'] = self.board.RX_CONFIG['midrc'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MISC['minthrottle'] = self.board.MOTOR_CONFIG['minthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['maxthrottle'] = self.board.MOTOR_CONFIG['maxthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['mincommand'] = self.board.MOTOR_CONFIG['mincommand'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
            self.board.MISC['failsafe_throttle'] = self.board.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.board.MISC['gps_type'] = self.board.GPS_CONFIG['provider'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['sensors_baudrate'] = self.board.MISC['gps_baudrate'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['gps_ubx_sbas'] = self.board.GPS_CONFIG['ublox_sbas'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['rssi_channel'] = self.board.RSSI_CONFIG['channel'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.MISC['mag_declination'] = self.board.readbytes(data, size=16, unsigned=False) / 10 # -18000-18000
            self.board.MISC['vbatscale'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MISC['voltage_source'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['battery_cells'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MISC['vbatdetectcellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
            self.board.MISC['vbatmincellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
            self.board.MISC['vbatmaxcellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
            self.board.MISC['vbatwarningcellvoltage'] = self.board.readbytes(data, size=16, unsigned=True) / 100
            self.board.MISC['battery_capacity'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.MISC['battery_capacity_warning'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.MISC['battery_capacity_critical'] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.MISC['battery_capacity_unit'] = 'mWh' if self.board.readbytes(data, size=8, unsigned=True) else 'mAh'

    def process_MSP_MOTOR_CONFIG(self, data):
        self.board.MOTOR_CONFIG['minthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
        self.board.MOTOR_CONFIG['maxthrottle'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000
        self.board.MOTOR_CONFIG['mincommand'] = self.board.readbytes(data, size=16, unsigned=True) # 0-2000

        self.board.MOTOR_CONFIG['motor_count'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.MOTOR_CONFIG['motor_poles'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.MOTOR_CONFIG['use_dshot_telemetry'] = (self.board.readbytes(data, size=8, unsigned=True) != 0)
        self.board.MOTOR_CONFIG['use_esc_sensor'] = (self.board.readbytes(data, size=8, unsigned=True) != 0)

    def process_MSP_COMPASS_CONFIG(self, data):
        self.board.COMPASS_CONFIG['mag_declination'] = self.board.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000

    def process_MSP_GPS_CONFIG(self, data):
        self.board.GPS_CONFIG['provider'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_CONFIG['ublox_sbas'] = self.board.readbytes(data, size=8, unsigned=True)
        
        self.board.GPS_CONFIG['auto_config'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_CONFIG['auto_baud'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPS_RESCUE(self, data):
        self.board.GPS_RESCUE['angle']             = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['initialAltitudeM']  = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['descentDistanceM']  = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['rescueGroundspeed'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['throttleMin']       = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['throttleMax']       = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['throttleHover']     = self.board.readbytes(data, size=16, unsigned=True)
        self.board.GPS_RESCUE['sanityChecks']      = self.board.readbytes(data, size=8, unsigned=True)
        self.board.GPS_RESCUE['minSats']           = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_RSSI_CONFIG(self, data):
        self.board.RSSI_CONFIG['channel'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_MOTOR_3D_CONFIG(self, data):
        self.board.MOTOR_3D_CONFIG['deadband3d_low'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.MOTOR_3D_CONFIG['deadband3d_high'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.MOTOR_3D_CONFIG['neutral'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_BOXNAMES(self, data):
        self.board.AUX_CONFIG = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.board.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B): # ; (delimeter char)
                self.board.AUX_CONFIG.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_PIDNAMES(self, data):
        self.board.PIDNAMES = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.board.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B):  # ; (delimeter char)
                self.board.PIDNAMES.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_BOXIDS(self, data):
        self.board.AUX_CONFIG_IDS = [] # empty the array as new data is coming in
        for i in range(len(data)):
            self.board.AUX_CONFIG_IDS.append(self.board.readbytes(data, size=8, unsigned=True))

    def process_MSP_SERVO_CONFIGURATIONS(self, data):
        self.board.SERVO_CONFIG = [] # empty the array as new data is coming in
        if (len(data) % 12 == 0):
            for i in range(0, len(data), 12):
                arr = {
                    'min':                      self.board.readbytes(data, size=16, unsigned=True),
                    'max':                      self.board.readbytes(data, size=16, unsigned=True),
                    'middle':                   self.board.readbytes(data, size=16, unsigned=True),
                    'rate':                     self.board.readbytes(data, size=8, unsigned=False),
                    'indexOfChannelToForward':  self.board.readbytes(data, size=8, unsigned=True),
                    'reversedInputSources':     self.board.readbytes(data, size=32, unsigned=True)
                }

                self.board.SERVO_CONFIG.append(arr)

    def process_MSP_RC_DEADBAND(self, data):
        self.board.RC_DEADBAND_CONFIG['deadband'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RC_DEADBAND_CONFIG['yaw_deadband'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RC_DEADBAND_CONFIG['alt_hold_deadband'] = self.board.readbytes(data, size=8, unsigned=True)

        self.board.RC_DEADBAND_CONFIG['deadband3d_throttle'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_ALIGNMENT(self, data):
        self.board.SENSOR_ALIGNMENT['align_gyro'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SENSOR_ALIGNMENT['align_acc'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SENSOR_ALIGNMENT['align_mag'] = self.board.readbytes(data, size=8, unsigned=True)

        if self.board.INAV:
            self.board.SENSOR_ALIGNMENT['align_opflow'] = self.board.readbytes(data, size=8, unsigned=True)
        else:
            self.board.SENSOR_ALIGNMENT['gyro_detection_flags'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.SENSOR_ALIGNMENT['gyro_to_use'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.SENSOR_ALIGNMENT['gyro_1_align'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.SENSOR_ALIGNMENT['gyro_2_align'] = self.board.readbytes(data, size=8, unsigned=True)

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
            self.board.SENSOR_DATA['debug'][i] = self.board.readbytes(data, size=16, unsigned=False)

    def process_MSP2_INAV_DEBUG(self, data):
        for i in range(8):
            self.board.SENSOR_DATA['debug'][i] = self.board.readbytes(data, size=32, unsigned=False)

    def process_MSP_SET_MOTOR(self, data):
        logging.info('Motor Speeds Updated')

    def process_MSP_UID(self, data):
        for i in range(3):
            self.board.CONFIG['uid'][i] = self.board.readbytes(data, size=32, unsigned=True)
    
    def process_MSP_ACC_TRIM(self, data):
        self.board.CONFIG['accelerometerTrims'][0] = self.board.readbytes(data, size=16, unsigned=False) # pitch
        self.board.CONFIG['accelerometerTrims'][1] = self.board.readbytes(data, size=16, unsigned=False) # roll

    def process_MSP_SET_ACC_TRIM(self, data):
        logging.info('Accelerometer trimms saved.')

    def process_MSP_GPS_SV_INFO(self, data):
        if (len(data) > 0):
            numCh = self.board.readbytes(data, size=8, unsigned=True)

            for i in range(numCh):
                self.board.GPS_DATA['chn'].append(self.board.readbytes(data, size=8, unsigned=True))
                self.board.GPS_DATA['svid'].append(self.board.readbytes(data, size=8, unsigned=True))
                self.board.GPS_DATA['quality'].append(self.board.readbytes(data, size=8, unsigned=True))
                self.board.GPS_DATA['cno'].append(self.board.readbytes(data, size=8, unsigned=True))

    def process_MSP_RX_MAP(self, data):
        self.board.RC_MAP = [] # empty the array as new data is coming in

        for i in range(len(data)):
            self.board.RC_MAP.append(self.board.readbytes(data, size=8, unsigned=True))

    def process_MSP_SET_RX_MAP(self, data):
        logging.debug('RCMAP saved')
        
    def process_MSP_MIXER_CONFIG(self, data):
        self.board.MIXER_CONFIG['mixer'] = self.board.readbytes(data, size=8, unsigned=True)
        if not self.board.INAV:                    
            self.board.MIXER_CONFIG['reverseMotorDir'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_FEATURE_CONFIG(self, data):
        self.board.FEATURE_CONFIG['featuremask']  = self.board.readbytes(data, size=32, unsigned=True)
        for idx in range(32):
            enabled = self.board.bit_check(self.board.FEATURE_CONFIG['featuremask'], idx)
            if idx in self.board.FEATURE_CONFIG['features'].keys():
                self.board.FEATURE_CONFIG['features'][idx]['enabled'] = enabled
            else:
                self.board.FEATURE_CONFIG['features'][idx] = {'enabled': enabled}

    def process_MSP_BEEPER_CONFIG(self, data):
        self.board.BEEPER_CONFIG['beepers'] = self.board.readbytes(data, size=32, unsigned=True)
            
        self.board.BEEPER_CONFIG['dshotBeaconTone'] = self.board.readbytes(data, size=8, unsigned=True)

        self.board.BEEPER_CONFIG['dshotBeaconConditions'] = self.board.readbytes(data, size=32, unsigned=True)

    def process_MSP_BOARD_ALIGNMENT_CONFIG(self, data):
        self.board.BOARD_ALIGNMENT_CONFIG['roll'] = self.board.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.board.BOARD_ALIGNMENT_CONFIG['pitch'] = self.board.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.board.BOARD_ALIGNMENT_CONFIG['yaw'] = self.board.readbytes(data, size=16, unsigned=False) # -180 - 360

    def process_MSP_SET_REBOOT(self, data):
        rebootType = self.board.readbytes(data, size=8, unsigned=True)

        if ((rebootType == self.board.REBOOT_TYPES['MSC']) or (rebootType == self.board.REBOOT_TYPES['MSC_UTC'])):
            if (self.board.readbytes(data, size=8, unsigned=True) == 0):
                logging.warning('Storage device not ready for reboot.')

        logging.info('Reboot request accepted')

    def process_MSP_API_VERSION(self, data):
        self.board.CONFIG['mspProtocolVersion'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.CONFIG['apiVersion'] = str(self.board.readbytes(data, size=8, unsigned=True)) + '.' + str(self.board.readbytes(data, size=8, unsigned=True)) + '.0'

    def process_MSP_FC_VARIANT(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.board.readbytes(data, size=8, unsigned=True))
        self.board.CONFIG['flightControllerIdentifier'] = identifier

    def process_MSP_FC_VERSION(self, data):
        self.board.CONFIG['flightControllerVersion'] =  str(self.board.readbytes(data, size=8, unsigned=True)) + '.'
        self.board.CONFIG['flightControllerVersion'] += str(self.board.readbytes(data, size=8, unsigned=True)) + '.'
        self.board.CONFIG['flightControllerVersion'] += str(self.board.readbytes(data, size=8, unsigned=True))

    def process_MSP_BUILD_INFO(self, data):
        dateLength = 11
        buff = []
        for i in range(dateLength):
            buff.append(self.board.readbytes(data, size=8, unsigned=True))
        
        buff.append(32) # ascii space

        timeLength = 8
        for i in range(timeLength):
            buff.append(self.board.readbytes(data, size=8, unsigned=True))

        self.board.CONFIG['buildInfo'] = bytearray(buff).decode("utf-8")

    def process_MSP_BOARD_INFO(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.board.readbytes(data, size=8, unsigned=True))

        self.board.CONFIG['boardIdentifier'] = identifier
        self.board.CONFIG['boardVersion'] = self.board.readbytes(data, size=16, unsigned=True)

        self.board.CONFIG['boardType'] = self.board.readbytes(data, size=8, unsigned=True)

        self.board.CONFIG['targetName'] = ""

        self.board.CONFIG['commCapabilities'] = self.board.readbytes(data, size=8, unsigned=True)

        length = self.board.readbytes(data, size=8, unsigned=True)
        
        for i in range(length):
            self.board.CONFIG['targetName'] += chr(self.board.readbytes(data, size=8, unsigned=True))

        self.board.CONFIG['boardName'] = ""
        self.board.CONFIG['manufacturerId'] = ""
        self.board.CONFIG['signature'] = []
        self.board.CONFIG['boardName'] = ""
        self.board.CONFIG['mcuTypeId'] = ""

        if data:
            length = self.board.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.board.CONFIG['boardName'] += chr(self.board.readbytes(data, size=8, unsigned=True))

            length = self.board.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.board.CONFIG['manufacturerId'] += chr(self.board.readbytes(data, size=8, unsigned=True))

            for i in range(self.board.SIGNATURE_LENGTH):
                self.board.CONFIG['signature'].append(self.board.readbytes(data, size=8, unsigned=True))

            self.board.CONFIG['mcuTypeId'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_NAME(self, data):
        self.board.CONFIG['name'] = ''
    
        while len(data)>0:
            char = self.board.readbytes(data, size=8, unsigned=True)
            self.board.CONFIG['name'] += chr(char)

    # def process_MSP_SET_CHANNEL_FORWARDING(self, data):
    #     logging.info('Channel forwarding saved')

    def process_MSP_CF_SERIAL_CONFIG(self, data):
        self.board.SERIAL_CONFIG['ports'] = []
        bytesPerPort = 1 + 2 + (1 * 4)
        serialPortCount = int(len(data) / bytesPerPort)

        for i in range(serialPortCount):
            serialPort = {
                'identifier': self.board.readbytes(data, size=8, unsigned=True),
                'functions': self.board.serialPortFunctionMaskToFunctions(self.board.readbytes(data, size=16, unsigned=True)),
                'msp_baudrate': self.board.BAUD_RATES[self.board.readbytes(data, size=8, unsigned=True)],
                'gps_baudrate': self.board.BAUD_RATES[self.board.readbytes(data, size=8, unsigned=True)],
                'telemetry_baudrate': self.board.BAUD_RATES[self.board.readbytes(data, size=8, unsigned=True)],
                'blackbox_baudrate': self.board.BAUD_RATES[self.board.readbytes(data, size=8, unsigned=True)]
            }

            self.board.SERIAL_CONFIG['ports'].append(serialPort)

    def process_MSP_SET_CF_SERIAL_CONFIG(self, data):
        logging.info('Serial config saved')

    def process_MSP_MODE_RANGES(self, data):
        self.board.MODE_RANGES = [] # empty the array as new data is coming in

        modeRangeCount = int(len(data) / 4) # 4 bytes per item.

        for i in range(modeRangeCount):
            modeRange = {
                'id': self.board.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.board.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.board.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.board.readbytes(data, size=8, unsigned=True) * 25)
                            }
                }
            self.board.MODE_RANGES.append(modeRange)

    def process_MSP_MODE_RANGES_EXTRA(self, data):
        self.board.MODE_RANGES_EXTRA = [] # empty the array as new data is coming in

        modeRangeExtraCount = self.board.readbytes(data, size=8, unsigned=True)

        for i in range(modeRangeExtraCount):
            modeRangeExtra = {
                'id': self.board.readbytes(data, size=8, unsigned=True),
                'modeLogic': self.board.readbytes(data, size=8, unsigned=True),
                'linkedTo': self.board.readbytes(data, size=8, unsigned=True)
            }
            self.board.MODE_RANGES_EXTRA.append(modeRangeExtra)

    def process_MSP_ADJUSTMENT_RANGES(self, data):
        self.board.ADJUSTMENT_RANGES = [] # empty the array as new data is coming in

        adjustmentRangeCount = int(len(data) / 6) # 6 bytes per item.

        for i in range(adjustmentRangeCount):
            adjustmentRange = {
                'slotIndex': self.board.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.board.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.board.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.board.readbytes(data, size=8, unsigned=True) * 25)
                },
                'adjustmentFunction': self.board.readbytes(data, size=8, unsigned=True),
                'auxSwitchChannelIndex': self.board.readbytes(data, size=8, unsigned=True)
            }
            self.board.ADJUSTMENT_RANGES.append(adjustmentRange)

    def process_MSP_RX_CONFIG(self, data):
        self.board.RX_CONFIG['serialrx_provider'] = self.board.readbytes(data, size=8, unsigned=True)
        # maxcheck for INAV
        self.board.RX_CONFIG['stick_max'] = self.board.readbytes(data, size=16, unsigned=True)
        # midrc for INAV
        self.board.RX_CONFIG['stick_center'] = self.board.readbytes(data, size=16, unsigned=True)
        # mincheck for INAV
        self.board.RX_CONFIG['stick_min'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.RX_CONFIG['spektrum_sat_bind'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RX_CONFIG['rx_min_usec'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.RX_CONFIG['rx_max_usec'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.RX_CONFIG['rcInterpolation'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RX_CONFIG['rcInterpolationInterval'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RX_CONFIG['airModeActivateThreshold'] = self.board.readbytes(data, size=16, unsigned=True)  
        # spirx_protocol for INAV
        self.board.RX_CONFIG['rxSpiProtocol'] = self.board.readbytes(data, size=8, unsigned=True)
        # spirx_id for INAV
        self.board.RX_CONFIG['rxSpiId'] = self.board.readbytes(data, size=32, unsigned=True)
        # spirx_channel_count for INAV
        self.board.RX_CONFIG['rxSpiRfChannelCount'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.RX_CONFIG['fpvCamAngleDegrees'] = self.board.readbytes(data, size=8, unsigned=True)
        if self.board.INAV:
            self.board.RX_CONFIG['receiver_type'] = self.board.readbytes(data, size=8, unsigned=True)
        else:
            self.board.RX_CONFIG['rcInterpolationChannels'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RX_CONFIG['rcSmoothingType'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RX_CONFIG['rcSmoothingInputCutoff'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RX_CONFIG['rcSmoothingDerivativeCutoff'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RX_CONFIG['rcSmoothingInputType'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.RX_CONFIG['rcSmoothingDerivativeType'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_FAILSAFE_CONFIG(self, data):
        self.board.FAILSAFE_CONFIG['failsafe_delay'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.FAILSAFE_CONFIG['failsafe_off_delay'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.FAILSAFE_CONFIG['failsafe_throttle'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FAILSAFE_CONFIG['failsafe_switch_mode'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.FAILSAFE_CONFIG['failsafe_throttle_low_delay'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FAILSAFE_CONFIG['failsafe_procedure'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_RXFAIL_CONFIG(self, data):
        self.board.RXFAIL_CONFIG = [] # empty the array as new data is coming in

        channelCount = int(len(data) / 3)
        for i in range(channelCount):
            rxfailChannel = {
                'mode':  self.board.readbytes(data, size=8, unsigned=True),
                'value': self.board.readbytes(data, size=16, unsigned=True)
            }
            self.board.RXFAIL_CONFIG.append(rxfailChannel)

    def process_MSP_ADVANCED_CONFIG(self, data):
        self.board.PID_ADVANCED_CONFIG['gyro_sync_denom'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.PID_ADVANCED_CONFIG['pid_process_denom'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.PID_ADVANCED_CONFIG['use_unsyncedPwm'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.PID_ADVANCED_CONFIG['fast_pwm_protocol'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.PID_ADVANCED_CONFIG['motor_pwm_rate'] = self.board.readbytes(data, size=16, unsigned=True)

        self.board.PID_ADVANCED_CONFIG['digitalIdlePercent'] = self.board.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_FILTER_CONFIG(self, data):
        self.board.FILTER_CONFIG['gyro_lowpass_hz'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.FILTER_CONFIG['dterm_lowpass_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FILTER_CONFIG['yaw_lowpass_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        
        self.board.FILTER_CONFIG['gyro_notch_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FILTER_CONFIG['gyro_notch_cutoff'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FILTER_CONFIG['dterm_notch_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FILTER_CONFIG['dterm_notch_cutoff'] = self.board.readbytes(data, size=16, unsigned=True)

        self.board.FILTER_CONFIG['gyro_notch2_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.FILTER_CONFIG['gyro_notch2_cutoff'] = self.board.readbytes(data, size=16, unsigned=True)

        if not self.board.INAV:
            self.board.FILTER_CONFIG['dterm_lowpass_type'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.FILTER_CONFIG['gyro_hardware_lpf'] = self.board.readbytes(data, size=8, unsigned=True)
            
            self.board.readbytes(data, size=8, unsigned=True) # must consume this byte

            self.board.FILTER_CONFIG['gyro_lowpass_hz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['gyro_lowpass2_hz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['gyro_lowpass_type'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.FILTER_CONFIG['gyro_lowpass2_type'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.FILTER_CONFIG['dterm_lowpass2_hz'] = self.board.readbytes(data, size=16, unsigned=True)

            self.board.FILTER_CONFIG['gyro_32khz_hardware_lpf'] = 0

            self.board.FILTER_CONFIG['dterm_lowpass2_type'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.FILTER_CONFIG['gyro_lowpass_dyn_min_hz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['gyro_lowpass_dyn_max_hz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['dterm_lowpass_dyn_min_hz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['dterm_lowpass_dyn_max_hz'] = self.board.readbytes(data, size=16, unsigned=True)
        else:
            self.board.FILTER_CONFIG['accNotchHz'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['accNotchCutoff'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.FILTER_CONFIG['gyroStage2LowpassHz'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_SET_PID_ADVANCED(self, data):
        logging.info("Advanced PID settings saved")

    def process_MSP_PID_ADVANCED(self, data):
        self.board.ADVANCED_TUNING['rollPitchItermIgnoreRate'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.ADVANCED_TUNING['yawItermIgnoreRate'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.ADVANCED_TUNING['yaw_p_limit'] = self.board.readbytes(data, size=16, unsigned=True)
        self.board.ADVANCED_TUNING['deltaMethod'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.ADVANCED_TUNING['vbatPidCompensation'] = self.board.readbytes(data, size=8, unsigned=True)
        if not self.board.INAV:
            self.board.ADVANCED_TUNING['feedforwardTransition'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.ADVANCED_TUNING['dtermSetpointWeight'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['toleranceBand'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['toleranceBandReduction'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['itermThrottleGain'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['pidMaxVelocity'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['pidMaxVelocityYaw'] = self.board.readbytes(data, size=16, unsigned=True)

            self.board.ADVANCED_TUNING['levelAngleLimit'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['levelSensitivity'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.ADVANCED_TUNING['itermThrottleThreshold'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['itermAcceleratorGain'] = self.board.readbytes(data, size=16, unsigned=True)

            self.board.ADVANCED_TUNING['dtermSetpointWeight'] = self.board.readbytes(data, size=16, unsigned=True)

            self.board.ADVANCED_TUNING['itermRotation'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['smartFeedforward'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['itermRelax'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['itermRelaxType'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['absoluteControlGain'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['throttleBoost'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['acroTrainerAngleLimit'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['feedforwardRoll']  = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['feedforwardPitch'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['feedforwardYaw']   = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['antiGravityMode']  = self.board.readbytes(data, size=8, unsigned=True)

            self.board.ADVANCED_TUNING['dMinRoll'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['dMinPitch'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['dMinYaw'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['dMinGain'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['dMinAdvance'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['useIntegratedYaw'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['integratedYawRelax'] = self.board.readbytes(data, size=8, unsigned=True)
        else:
            self.board.ADVANCED_TUNING['setpointRelaxRatio'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['dtermSetpointWeight'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['pidSumLimit'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['itermThrottleGain'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.ADVANCED_TUNING['axisAccelerationLimitRollPitch'] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.ADVANCED_TUNING['axisAccelerationLimitYaw'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_CONFIG(self, data):
        self.board.SENSOR_CONFIG['acc_hardware'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SENSOR_CONFIG['baro_hardware'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SENSOR_CONFIG['mag_hardware'] = self.board.readbytes(data, size=8, unsigned=True)
        if self.board.INAV:
            self.board.SENSOR_CONFIG['pitot'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.SENSOR_CONFIG['rangefinder'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.SENSOR_CONFIG['opflow'] = self.board.readbytes(data, size=8, unsigned=True)

    def process_MSP_DATAFLASH_SUMMARY(self, data):
        flags = self.board.readbytes(data, size=8, unsigned=True)
        self.board.DATAFLASH['ready'] = ((flags & 1) != 0)
        self.board.DATAFLASH['supported'] = ((flags & 2) != 0)
        self.board.DATAFLASH['sectors'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.DATAFLASH['totalSize'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.DATAFLASH['usedSize'] = self.board.readbytes(data, size=32, unsigned=True)
        # update_dataflash_global();

    def process_MSP_DATAFLASH_ERASE(self, data):
        logging.info("Data flash erase begun...")

    def process_MSP_SDCARD_SUMMARY(self, data):
        flags = self.board.readbytes(data, size=8, unsigned=True)

        self.board.SDCARD['supported'] = ((flags & 0x01) != 0)
        self.board.SDCARD['state'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SDCARD['filesystemLastError'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.SDCARD['freeSizeKB'] = self.board.readbytes(data, size=32, unsigned=True)
        self.board.SDCARD['totalSizeKB'] = self.board.readbytes(data, size=32, unsigned=True)

    def process_MSP_BLACKBOX_CONFIG(self, data):
        if not self.board.INAV:
            self.board.BLACKBOX['supported'] = (self.board.readbytes(data, size=8, unsigned=True) & 1) != 0
            self.board.BLACKBOX['blackboxDevice'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.BLACKBOX['blackboxRateNum'] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.BLACKBOX['blackboxRateDenom'] = self.board.readbytes(data, size=8, unsigned=True)

            self.board.BLACKBOX['blackboxPDenom'] = self.board.readbytes(data, size=16, unsigned=True)
        else:
            pass # API no longer supported (INAV 2.3.0)

    def process_MSP_SET_BLACKBOX_CONFIG(self, data):
        logging.info("Blackbox config saved")

    def process_MSP_MOTOR_TELEMETRY(self, data):
        motorCount = self.board.readbytes(data, size=8, unsigned=True)
        for i in range(motorCount):
            self.board.MOTOR_TELEMETRY_DATA['rpm'][i] = self.board.readbytes(data, size=32, unsigned=True)
            self.board.MOTOR_TELEMETRY_DATA['invalidPercent'][i] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MOTOR_TELEMETRY_DATA['temperature'][i] = self.board.readbytes(data, size=8, unsigned=True)
            self.board.MOTOR_TELEMETRY_DATA['voltage'][i] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MOTOR_TELEMETRY_DATA['current'][i] = self.board.readbytes(data, size=16, unsigned=True)
            self.board.MOTOR_TELEMETRY_DATA['consumption'][i] = self.board.readbytes(data, size=16, unsigned=True)

    # TODO: This changed and it will need to check the BF version to decode things correctly
    # def process_MSP_TRANSPONDER_CONFIG(self, data):
    #     bytesRemaining = len(data)

    #     providerCount = self.board.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.board.TRANSPONDER['supported'] = providerCount > 0
    #     self.board.TRANSPONDER['providers'] = []

    #     for i in range(providerCount):
    #         provider = {
    #             'id': self.board.readbytes(data, size=8, unsigned=True),
    #             'dataLength': self.board.readbytes(data, size=8, unsigned=True)
    #         }
    #         bytesRemaining -= 2

    #         self.board.TRANSPONDER['providers'].append(provider)

    #     self.board.TRANSPONDER['provider'] = self.board.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.board.TRANSPONDER['data'] = []
    #     for i in range(bytesRemaining):
    #         self.board.TRANSPONDER['data'].append(self.board.readbytes(data, size=8, unsigned=True))


    # implement new functions below

    def process_MSP_NAV_STATUS(self,data):
        self.board.NAV_STATUS['mode'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.NAV_STATUS['state'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.NAV_STATUS['active_wp_action'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.NAV_STATUS['active_wp_number'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.NAV_STATUS['error'] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.NAV_STATUS['heading_hold_target'] = self.board.readbytes(data, size=16, unsigned=True)

    def process_MSP_WP(self, data):
        self.board.WP['wp_no'] = self.board.readbytes(data, size=8, unsigned=True)      
        self.board.WP['wp_action'] = self.board.readbytes(data, size=8, unsigned=True)   
        self.board.WP['lat'] = self.board.readbytes(data, size=32, unsigned=False)   
        self.board.WP['lon'] = self.board.readbytes(data, size=32, unsigned=False)  
        self.board.WP['alt'] = self.board.readbytes(data, size=32, unsigned=False)  
        self.board.WP['p1'] = self.board.readbytes(data, size=16, unsigned=False)   
        self.board.WP['p2'] = self.board.readbytes(data, size=16, unsigned=False)  
        self.board.WP['p3'] = self.board.readbytes(data, size=16, unsigned=False)  
        self.board.WP['flag'] = self.board.readbytes(data, size=8, unsigned=True)   

    def process_MSP_WP_GETINFO(self,data):
        self.board.WP_INFO["reserved"] = self.board.readbytes(data, size=8, unsigned=True),   
        self.board.WP_INFO["NAV_MAX_WAYPOINTS"] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.WP_INFO["isWaypointListValid"] = self.board.readbytes(data, size=8, unsigned=True)
        self.board.WP_INFO["WaypointCount"] = self.board.readbytes(data, size=8, unsigned=True)   

    def process_MSP_SET_WP(self, data):
        # response to MSP_SET_WP returns the packed waypoint back
        wp_no, action, lat, lon, alt, p1, p2, p3, flag = struct.unpack('<BBiiihhhB', data)
        self.board.WP = {
            'wp_no':           wp_no,
            'wp_action':       action,
            'lat':             lat,
            'lon':             lon,
            'alt':             alt,
            'p1':               p1,
            'p2':               p2,
            'p3':               p3,
            'flag':            flag
        }

    def _process_MSP_WP(self, data):
        # response to MSP_WP (get waypoint #) is the same format
        wp_no, action, lat, lon, alt, p1, p2, p3, flag = struct.unpack('<BBiiihhhB', data)
        self.board.WP = {
            'wp_no':           wp_no,
            'wp_action':       action,
            'lat':             lat,
            'lon':             lon,
            'alt':             alt,
            'p1':               p1,
            'p2':               p2,
            'p3':               p3,
            'flag':            flag
        }

    def process_MSP_WP_GETINFO(self, data):
        # returns: reserved, max_waypoints, isValid flag, waypoint_count (4 bytes)
        reserved, maxwps, valid, count = struct.unpack('<BBBB', data)
        self.board.WP_INFO = {
            'reserved':            reserved,
            'NAV_MAX_WAYPOINTS':   maxwps,
            'isWaypointListValid': bool(valid),
            'WaypointCount':       count
        }


    # in yamspy, but unimplemented

    def process_MSP_SET_TRANSPONDER_CONFIG(self, data):
        logging.info("Transponder config saved")

    def process_MSP_SET_MODE_RANGE(self, data):
        logging.info('Mode range saved')

    def process_MSP_SET_ADJUSTMENT_RANGE(self, data):
        logging.info('Adjustment range saved')
        
    def process_MSP_SET_BOARD_ALIGNMENT_CONFIG(self, data):
        logging.info('Board alignment saved')
        
    def process_MSP_PID_CONTROLLER(self, data):
        self.board.PID['controller'] = self.board.readbytes(data, size=8, unsigned=True)
        
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