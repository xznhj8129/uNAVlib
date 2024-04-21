
import logging
import struct
import time
import sys
import serial
from select import select

# WHY DO THESE SUCK SO MUCH

class fastMSP():
    def __init__(self,mspy_instance):
        self.mspy = mspy_instance

    def fast_read_altitude(self):
        # Request altitude
        if self.mspy.send_RAW_msg(self.mspy.MSPCodes['MSP_ALTITUDE']):
            # dataHandler = self.mspy.receive_msg()
            # self.mspy.process_recv_data(dataHandler)
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            data_length = 4
            msg = self.mspy.receive_raw_msg(size = (6+data_length))[5:]
            converted_msg = struct.unpack('<i', msg[:-1])[0]
            self.mspy.SENSOR_DATA['altitude'] = round((converted_msg / 100.0), 2) # correct scale factor
            return self.mspy.SENSOR_DATA['altitude']

    def fast_read_imu(self):
        # Request IMU values
        if self.mspy.send_RAW_msg(self.mspy.MSPCodes['MSP_RAW_IMU']):
            # WHY NOT THIS???!!! HOW SLOW IS IT?!
            #dataHandler = self.mspy.receive_msg()
            #self.mspy.process_recv_data(dataHandler)
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            # data_length: 9 x 2 = 18 bytes

            data_length = 18
            msg = self.mspy.receive_raw_msg(size = (6+data_length))
            msg = msg[5:]
            converted_msg = struct.unpack('<%dh' % (data_length/2) , msg[:-1])

            # /512 for mpu6050, /256 for mma
            # currently we are unable to differentiate between the sensor types, so we are going with 512
            # And what about SENSOR_CONFIG???
            self.mspy.SENSOR_DATA['accelerometer'][0] = converted_msg[0]
            self.mspy.SENSOR_DATA['accelerometer'][1] = converted_msg[1]
            self.mspy.SENSOR_DATA['accelerometer'][2] = converted_msg[2]

            # properly scaled (INAV and BF use the same * (4 / 16.4))
            # but this is supposed to be RAW, so raw it is!
            self.mspy.SENSOR_DATA['gyroscope'][0] = converted_msg[3]
            self.mspy.SENSOR_DATA['gyroscope'][1] = converted_msg[4]
            self.mspy.SENSOR_DATA['gyroscope'][2] = converted_msg[5]

            # no clue about scaling factor (/1090), so raw
            self.mspy.SENSOR_DATA['magnetometer'][0] = converted_msg[6]
            self.mspy.SENSOR_DATA['magnetometer'][1] = converted_msg[7]
            self.mspy.SENSOR_DATA['magnetometer'][2] = converted_msg[8]
            return {"accelerometer": self.mspy.SENSOR_DATA['accelerometer'],
                    "gyroscope": self.mspy.SENSOR_DATA['gyroscope'],
                    "magnetometer": self.mspy.SENSOR_DATA['magnetometer']}

            """
            failure happened here, catch it
            Traceback (most recent call last):
            File "/uNAVlib/examples/test_msp2.py", line 30, in <module>
                print('imu:',board.fast_read_imu())
                            ^^^^^^^^^^^^^^^^^^^^^
            File "/uNAVlib/unavlib/__init__.py", line 285, in fast_read_imu
                converted_msg = struct.unpack('<%dh' % (data_length/2) , msg[:-1])
                                ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            struct.error: unpack requires a buffer of 18 bytes"""


    def fast_read_attitude(self): 
        """Request, read and process the ATTITUDE (Roll, Pitch and Yaw in degrees)
        """

        # Request ATTITUDE values
        if self.mspy.send_RAW_msg(self.mspy.MSPCodes['MSP_ATTITUDE']):
            # dataHandler = self.mspy.receive_msg()
            # self.mspy.process_recv_data(dataHandler)
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            # data_length: 3 x 2 = 6 bytes
            data_length = 6
            msg = self.mspy.receive_raw_msg(size = (6+data_length))[5:]
            converted_msg = struct.unpack('<%dh' % (data_length/2) , msg[:-1])
            self.mspy.SENSOR_DATA['kinematics'][0] = converted_msg[0] / 10.0 # x ROLL
            self.mspy.SENSOR_DATA['kinematics'][1] = converted_msg[1] / 10.0 # y PITCH
            self.mspy.SENSOR_DATA['kinematics'][2] = converted_msg[2] # z YAW
            return {"pitch": self.mspy.SENSOR_DATA['kinematics'][1],
                    "yaw": self.mspy.SENSOR_DATA['kinematics'][2],
                    "roll": self.mspy.SENSOR_DATA['kinematics'][0]}
    
    
    def fast_read_analog(self):
        """Request, read and process the ANALOG message
        """

        # Request ANALOG values
        if self.mspy.send_RAW_msg(self.mspy.MSPCodes['MSP_ANALOG']):
            # dataHandler = self.mspy.receive_msg()
            # self.mspy.process_recv_data(dataHandler)
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            if not self.mspy.INAV:
                # data_length: 1 + 2 + 2 + 2 + 2 = 9 bytes
                data_length = 9
                msg = self.mspy.receive_raw_msg(size = (6+data_length))[5:]
                converted_msg = struct.unpack('<B2HhH', msg[:-1])

            else:
                # data_length: 1 + 2 + 2 + 2 = 7 bytes
                data_length = 7
                msg = self.mspy.receive_raw_msg(size = (6+data_length))[5:]
                converted_msg = struct.unpack('<B2Hh', msg[:-1])

            self.mspy.ANALOG['voltage'] = converted_msg[0] / 10 # iNAV uses a MSP2 message to get a precise value.
            self.mspy.ANALOG['mAhdrawn'] = converted_msg[1]
            self.mspy.ANALOG['rssi'] = converted_msg[2] # 0-1023
            self.mspy.ANALOG['amperage'] = converted_msg[3] / 100 # A
            self.mspy.ANALOG['last_received_timestamp'] = int(time.time()) # why not monotonic? where is time synchronized?
            if not self.mspy.INAV:
                self.mspy.ANALOG['voltage'] = converted_msg[4] / 100 # BF has this 2 bytes value here
            return self.mspy.ANALOG


    def fast_msp_rc_cmd(self, cmds):
        """Send, read and process the RAW RC considering the MSP_RX

        Parameters
        ----------
        cmds : list
            List with RC values to be sent
            * The number of values is 4 + number of AUX channels enabled (max 14) 
        """
        cmds = [int(cmd) for cmd in cmds]
        data = struct.pack('<%dH' % len(cmds), *cmds)
        if self.mspy.send_RAW_msg(self.mspy.MSPCodes['MSP_SET_RAW_RC'], data):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length

            # The FC will send a code 0 message until it received enough RC msgs, then it
            # will return a code 200. However, the message is always empty (data_length = 0).
            _ = self.mspy.receive_raw_msg(size = 6)
