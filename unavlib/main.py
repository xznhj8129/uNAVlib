# Unavlib main.py 
# msp_ctrl and boardconn condensed again
# Here the main MSPY class will be using Facade Inheritance 
import logging
import struct
import time
import sys
import serial
from select import select
from threading import Lock, RLock

from .modules import msp_ctrl
from .enums import inav_enums
from .enums import msp_codes
from .enums import msp_vars
from .modules.utils import dict_reverse
from .modules.utils import TCPSocket
from .modules.process import processMSP
from .modules.fast_functions import fastMSP
from .modules.utils import inavutil

from serial import SerialException

if "linux" in sys.platform:
    import ctypes
    ffs = ctypes.cdll.LoadLibrary('libc.so.6').ffs # this is only for ffs... it should be directly implemented.
else:
    def ffs(x): # modified from https://stackoverflow.com/a/36059264
        return (x&-x).bit_length()

import time

JUMBO_FRAME_SIZE_LIMIT = 255


dataHandler_init = {
    'msp_version':                1,
    'state':                      0,
    'message_direction':          -1,
    'code':                       0,
    'dataView':                   [],
    'message_length_expected':    0,
    'message_length_received':    0,
    'message_buffer':             [],
    'message_buffer_uint8_view':  [],
    'message_checksum':           0,
    'messageIsJumboFrame':        False,
    'crcError':                   False,
    'packet_error':               0,
    'unsupported':                0,
    'last_received_timestamp':   None
}


read_buffer = b''

def mspctrl_read(local_read):
    def read(size=None, buffer=None):
        global read_buffer
        if buffer:
            read_buffer = buffer
            return
            
        output = b''
        if size:
            while True:
                output += read_buffer[:size]
                read_buffer = read_buffer[size:]
                size -= len(output)
                if size > 0:
                    read_buffer += local_read() # read (try) everything in the serial/socket buffer
                else:
                    break
        else:
            if len(read_buffer)==0:
                read_buffer += local_read() # read (try) everything in the serial/socket buffer
            output += read_buffer
            read_buffer = b''
        return output
    return read

def mspctrl_receive_raw_msg(local_read, logging, timeout_exception, size, timeout = 10):
    """Receive multiple bytes at once when it's not a jumbo frame.
    Returns
    -------
    bytes
        data received
    """
    local_read = mspctrl_read(local_read)
    msg_header = b''
    timeout = time.time() + timeout
    while True:
        if time.time() >= timeout:
            logging.warning("Timeout occured when receiving a message")
            raise timeout_exception("receive_raw_msg timeout")
        msg_header = local_read(size=1)
        if msg_header:
            if ord(msg_header) == 36: # $
                break
    msg = local_read(size=(size - 1)) # -1 to compensate for the $
    return msg_header + msg

def mspctrl_receive_msg(local_read, logging, output_raw_bytes=False):
    """Receive an MSP message from the serial port
    Based on betaflight-configurator (https://git.io/fjRAz)

    Returns
    -------
    dict
        dataHandler with the received data pre-parsed
    """
    local_read = mspctrl_read(local_read)
    dataHandler = dataHandler_init.copy()
    if output_raw_bytes:
        raw_bytes = b''

    di = 0
    while True:
        try:
            if di == 0:
                received_bytes = memoryview(local_read()) # it will read everything from the buffer
                if received_bytes:
                    dataHandler['last_received_timestamp'] = time.time()
                    data = received_bytes[di]
                    if output_raw_bytes:
                        raw_bytes += received_bytes[di:di+1]
                else:
                    dataHandler['packet_error'] = 1
                    break
            else:
                data = received_bytes[di]
                if output_raw_bytes:
                    raw_bytes += received_bytes[di:di+1]

            di += 1
            logging.debug(f"State: {dataHandler['state']} - byte received (at {dataHandler['last_received_timestamp']}): {data}")
        except IndexError:
            logging.debug('IndexError detected on state: {}'.format(dataHandler['state']))
            di = 0 # reads more data
            continue

        # it will always fall in the first state by default
        if dataHandler['state'] == 0: # sync char 1
            if (data == 36): # $ - a new MSP message begins with $
                dataHandler['state'] = 1

        elif dataHandler['state'] == 1: # sync char 2
            if (data == 77): # M - followed by an M => MSP V1
                dataHandler['msp_version'] = 1
                dataHandler['state'] = 2
            elif (data == 88): # X => MSP V2
                dataHandler['msp_version'] = 2
                dataHandler['state'] = 2
            else: # something went wrong, no M received...
                logging.debug('Something went wrong, no M received.')
                dataHandler['packet_error'] = 1
                break # sends it to the error state

        elif dataHandler['state'] == 2: # direction (should be >)
            dataHandler['unsupported'] = 0
            if (data == 33): # !
                # FC reports unsupported message error
                logging.debug('FC reports unsupported message error.')
                dataHandler['unsupported'] = 1
                dataHandler['packet_error'] = 1
                break # sends it to the error state
            else:
                if (data == 62): # > FC to PC
                    dataHandler['message_direction'] = 1
                elif (data == 60): # < PC to FC
                    dataHandler['message_direction'] = 0
                    
                if dataHandler['msp_version'] == 1:
                    dataHandler['state'] = 3

                elif dataHandler['msp_version'] == 2:
                    dataHandler['state'] = 2.1


        elif dataHandler['state'] == 2.1: # MSP V2: flag (ignored)
            dataHandler['flags'] = data # 4th byte 
            dataHandler['state'] = 2.2

        elif dataHandler['state'] == 2.2: # MSP V2: code LOW
            dataHandler['code'] = data
            dataHandler['state'] = 2.3

        elif dataHandler['state'] == 2.3: # MSP V2: code HIGH
            dataHandler['code'] |= data << 8
            dataHandler['state'] = 3.1

        elif dataHandler['state'] == 3:
            dataHandler['message_length_expected'] = data # 4th byte
            if dataHandler['message_length_expected'] == JUMBO_FRAME_SIZE_LIMIT:
                logging.debug("JumboFrame received.")
                dataHandler['messageIsJumboFrame'] = True

            # start the checksum procedure
            dataHandler['message_checksum'] = data
            dataHandler['state'] = 4

        elif dataHandler['state'] == 3.1: # MSP V2: msg length LOW
            dataHandler['message_length_expected'] = data
            dataHandler['state'] = 3.2

        elif dataHandler['state'] == 3.2: # MSP V2: msg length HIGH
            dataHandler['message_length_expected'] |= data << 8
            # setup buffer according to the message_length_expected
            dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
            if dataHandler['message_length_expected'] > 0:
                dataHandler['state'] = 7
            else:
                dataHandler['state'] = 9

        elif dataHandler['state'] == 4:
            dataHandler['code'] = data
            dataHandler['message_checksum'] ^= data

            if dataHandler['message_length_expected'] > 0:
                # process payload
                if dataHandler['messageIsJumboFrame']:
                    dataHandler['state'] = 5
                else:
                    dataHandler['state'] = 7
            else:
                # no payload
                dataHandler['state'] = 9

        elif dataHandler['state'] == 5:
            # this is a JumboFrame
            dataHandler['message_length_expected'] = data

            dataHandler['message_checksum'] ^= data

            dataHandler['state'] = 6

        elif dataHandler['state'] == 6:
            # calculates the JumboFrame size
            dataHandler['message_length_expected'] +=  256 * data
            logging.debug("JumboFrame message_length_expected: {}".format(dataHandler['message_length_expected']))
            # There's no way to check for transmission errors here...
            # In the worst scenario, it will try to read 255 + 256*255 = 65535 bytes
            dataHandler['message_checksum'] ^= data
            dataHandler['state'] = 7

        elif dataHandler['state'] == 7:
            # setup buffer according to the message_length_expected
            dataHandler['message_buffer'] = bytearray(dataHandler['message_length_expected'])
            dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code

            # payload
            dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
            dataHandler['message_checksum'] ^= data
            dataHandler['message_length_received'] += 1

            if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                dataHandler['state'] = 9
            else:
                dataHandler['state'] = 8

        elif dataHandler['state'] == 8:
            # payload
            dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
            dataHandler['message_checksum'] ^= data
            dataHandler['message_length_received'] += 1

            if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                dataHandler['state'] = 9

        elif dataHandler['state'] == 9:
                if dataHandler['msp_version'] == 1:
                    if dataHandler['message_checksum'] == data:
                        # checksum is correct, message received, store dataview
                        logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                        dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                        break
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        dataHandler['packet_error'] = 1 # sends it to the error state
                        break 
                elif dataHandler['msp_version'] == 2:
                    dataHandler['message_checksum'] = 0
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], 0) # flag
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['code'] & 0xFF) # code LOW
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['code'] & 0xFF00) >> 8) # code HIGH
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_length_expected'] & 0xFF) #  HIGH
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['message_length_expected'] & 0xFF00) >> 8) #  HIGH
                    for si in range(dataHandler['message_length_received']):
                        dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_buffer'][si])
                    if dataHandler['message_checksum'] == data:
                        # checksum is correct, message received, store dataview
                        logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                        dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                        break
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        dataHandler['packet_error'] = 1 # sends it to the error state
                        break

    if dataHandler['packet_error'] == 1:
        # it means an error occurred
        logging.debug('Error detected on state: {}'.format(dataHandler['state']))
    
    if len(received_bytes[di:]):
        local_read(buffer=bytes(received_bytes[di:])) # regurgitates unread stuff :)

    if output_raw_bytes:
        return dataHandler, raw_bytes
    else:
        return dataHandler 


def mspctrl_prepare_raw_msg(mspv, code, data=[]):
    """Send a RAW MSP message through the serial port
    Based on betaflight-configurator (https://git.io/fjRxz)

    Parameters
    ----------
    code : int
        MSP Code
    
    data: list or bytearray, optional
        Data to be sent (default is [])
        
    Returns
    -------
    int
        number of bytes of data actually written (including 6 bytes header)
    """

    res = -1

    # Always reserve 6 bytes for protocol overhead
    # $ + M + < + data_length + msg_code + data + msg_crc
    len_data = len(data)

    if len_data > 256: # this shouldn't be needed...
        mspv = 2

    if mspv==1: # MSP V1
        size = len_data + 6
        checksum = 0

        bufView = bytearray(size)

        bufView[0] = 36 #$
        bufView[1] = 77 #M
        bufView[2] = 60 #<
        bufView[3] = len_data
        bufView[4] = code

        checksum = bufView[3] ^ bufView[4]

        for i in range(len_data):
            bufView[i + 5] = data[i]
            checksum ^= bufView[i + 5]

        bufView[-1] = checksum

    elif mspv==2: # MSP V2
        size = len_data + 9
        checksum = 0
        bufView = bytearray(size)
        bufView[0] = 36 #$ 
        bufView[1] = 88 #X
        bufView[2] = 60 #<
        bufView[3] = 0 #flag: reserved, set to 0
        bufView[4] = code & 0xFF #code lower byte
        bufView[5] = (code & 0xFF00) >> 8 #code upper byte
        bufView[6] = len_data & 0xFF #len_data lower byte
        bufView[7] = (len_data & 0xFF00) >> 8 #len_data upper byte
        bufView[8:8+len_data] = data
        for si in range(3, size-1):
            checksum = _crc8_dvb_s2(checksum, bufView[si])
        bufView[-1] = checksum

    else:
        return []

    return bufView


# def _crc8_dvb_s2(crc, ch):
#     """CRC for MSPV2
#     *copied from inav-configurator
#     """
#     crc ^= ch
#     for _ in range(8):
#         if (crc & 0x80):
#             crc = ((crc << 1) & 0xFF) ^ 0xD5
#         else:
#             crc = (crc << 1) & 0xFF
#     return crc


# from https://github.com/fishpepper/openTCO/blob/master/crc8.h
crc8_table = [
    0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
    0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
    0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
    0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
    0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
    0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
    0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
    0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
    0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
    0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
    0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
    0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
    0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
    0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
    0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
    0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9,
]

def _crc8_dvb_s2(crc, ch):
    return crc8_table[crc^ch]

class connMSP():
    def __init__(self, mspy_instance):
        self.board = mspy_instance
    
    def connect(self, trials=100, delay=0.5):
        """Opens the serial connection with the board

        Parameters
        ----------
        trials : int
            Number of times it should try openning the serial port before giving up (default is 1)

        delay : float
            Time between trials
            
        Returns
        -------
        int
            0 if successful or 1 if not
        """

        for _ in range(trials):
            try:
                if self.board.use_tcp:
                    self.board.start(port=int(self.board.device), timeout=self.board.timeout)
                else:
                    self.board.start()

                self.basic_info()
                return 0
                
            except SerialException as err:
                logging.warning(f"Error opening the serial port ({self.board.device}): {err}")
            
            except FileNotFoundError as err:
                logging.warning(f"Port ({self.board.device}) not found: {err}")

            time.sleep(delay)
        
        return 1


    def basic_info(self):
        """Basic info about the flight controller to distinguish between the many flavours.
        """
        msg = inavutil.msp.MSP_API_VERSION
        while self.board.CONFIG['apiVersion'] == '0.0.0':
            print(f"Waiting for {msg} reply...")
            sent = 0
            while sent<=0:
                sent = self.send_RAW_msg(msg, data=[])
            dataHandler = self.receive_msg()
            self.process_recv_data(dataHandler)

        msg = inavutil.msp.MSP_FC_VARIANT
        while self.board.CONFIG['flightControllerIdentifier'] == '':
            print(f"Waiting for {msg} reply...")
            sent = 0
            while sent<=0:
                sent = self.send_RAW_msg(msg, data=[])
            dataHandler = self.receive_msg()
            self.process_recv_data(dataHandler)

        if 'INAV' in self.board.CONFIG['flightControllerIdentifier']:
            self.board.INAV = True

        basic_info_cmd_list = [
            inavutil.msp.MSP_FC_VERSION, 
            inavutil.msp.MSP_BUILD_INFO, 
            inavutil.msp.MSP_BOARD_INFO, 
            inavutil.msp.MSP_UID, 
            inavutil.msp.MSP_ACC_TRIM, 
            inavutil.msp.MSP_NAME, 
            inavutil.msp.MSP_STATUS, 
            inavutil.msp.MSP_STATUS_EX, 
            inavutil.msp.MSP_ANALOG
        ]
        if self.board.INAV:
            basic_info_cmd_list.append(inavutil.msp.MSP2_INAV_ANALOG)
            basic_info_cmd_list.append(inavutil.msp.MSP_VOLTAGE_METER_CONFIG)
            basic_info_cmd_list.append(inavutil.msp.MSP2_INAV_STATUS)

        for msg in basic_info_cmd_list:
            sent = 0
            while sent<=0:
                sent = self.send_RAW_msg(msg, data=[])
            dataHandler = self.receive_msg()
            self.process_recv_data(dataHandler)
    
        print(self.board.CONFIG)

        
    def receive_raw_msg(self, size, timeout = 10):
        current_write = time.time()
        if (current_write-self.board.last_write) < self.board.min_time_between_writes:
            time.sleep(max(self.board.min_time_between_writes-(current_write-self.board.last_write),0))
        with self.board.port_read_lock:
            return mspctrl_receive_raw_msg(self.board.read, logging, self.board.timeout_exception, size, timeout)


    def receive_msg(self):
        current_write = time.time()
        if (current_write-self.board.last_write) < self.board.min_time_between_writes:
            time.sleep(max(self.board.min_time_between_writes-(current_write-self.board.last_write),0))
        with self.board.port_read_lock:
            return mspctrl_receive_msg(self.board.read, logging)


    @staticmethod
    def readbytes(data, size=8, unsigned=False, read_as_float=False):
        """Unpack bytes according to size / type

        Parameters
        ----------
        data : bytearray
            Data to be unpacked
        size : int, optional
            Number of bits (8, 16 or 32) (default is 8)
        unsigned : bool, optional
            Indicates if data is unsigned or not (default is False)
        read_as_float: bool, optional
            Indicates if data is read as float or not (default is False)
            
        Returns
        -------
        int
            unpacked bytes according to input options
        """
        buffer = bytearray()

        for _ in range(int(size/8)):
            buffer.append(data.pop(0))
        
        if size==8:
            unpack_format = 'b'
        elif size==16:
            if read_as_float: # for special situations like MSP2_INAV_DEBUG
                unpack_format = 'e'
            else:   
                unpack_format = 'h'
        elif size==32:
            if read_as_float: # for special situations like MSP2_INAV_DEBUG
                unpack_format = 'f'
            else:
                unpack_format = 'i'
        else:
            raise ValueError('size must be 8, 16 or 32')
        
        if unsigned:
            unpack_format = unpack_format.upper()

        return struct.unpack('<' + unpack_format, buffer)[0]


    def process_armingDisableFlags(self, flags):
        result = []
        while (flags):
            bitpos = ffs(flags) - 1
            flags &= ~(1 << bitpos)
            result.append(bitpos)
        return result

    # why does this not show active modes?
    def process_mode(self, flag):
        """Translate the value from CONFIG['mode']
        """
        result = []
        for i in range(len(self.board.AUX_CONFIG_IDS)):
            if (self.bit_check(flag, i)):
                result.append(self.board.AUX_CONFIG_IDS[i])
        return result


    @staticmethod
    def bit_check(mask, bit):
        return ((mask>>bit)%2) != 0


    def serialPortFunctionMaskToFunctions(self, functionMask):
        functions = []

        keys = self.board.SERIAL_PORT_FUNCTIONS.keys()
        for key in keys:
            bit = self.board.SERIAL_PORT_FUNCTIONS[key]
            if (self.bit_check(functionMask, bit)):
                functions.append(key)
        return functions


    @staticmethod
    def convert(val_list, n=16): 
        """Convert to n*bits (8 multiple) list

        Parameters
        ----------
        val_list : list
            List with values to be converted
        
        n: int, optional
            Number of bits (multiple of 8) (default is 16)
            
        Returns
        -------
        list
            List where each item is the equivalent byte value
        """ 
        buffer = []
        for val in val_list:
            for i in range(int(n/8)): 
                buffer.append((int(val)>>i*8) & 255) 
        return buffer 

    def save2eprom(self):
        logging.info("Save to EPROM requested") # some configs also need reboot to be applied (not online).
        return self.send_RAW_msg(inavutil.msp.MSP_EEPROM_WRITE, data=[])


    def reboot(self):
        logging.info("Reboot requested")
        rebooting = True
        while rebooting:
            if self.send_RAW_msg(inavutil.msp.MSP_REBOOT, data=[]):
                dataHandler = self.receive_msg()
                if dataHandler['code'] == inavutil.msp.MSP_REBOOT and dataHandler['packet_error'] == 0:
                    rebooting = False


    def set_ARMING_DISABLE(self, armingDisabled=0, runawayTakeoffPreventionDisabled=0):
        """Disable Arming or runaway takeoff prevention
        
        runawayTakeoffPreventionDisabled will be ignored if armingDisabled is true
        https://github.com/betaflight/betaflight/wiki/Runaway-Takeoff-Prevention
        """
        data = bytearray([armingDisabled, runawayTakeoffPreventionDisabled])
        return self.send_RAW_msg(inavutil.msp.MSP_ARMING_DISABLE, data)


    def set_RX_MAP(self, new_rc_map):
        assert(type(new_rc_map)==list)
        assert(len(new_rc_map)==8)

        return self.send_RAW_msg(inavutil.msp.MSP_SET_RX_MAP, new_rc_map)


    def set_FEATURE_CONFIG(self, mask):
        assert(type(mask)==int)

        data = self.convert([mask], 32)
        return self.send_RAW_msg(inavutil.msp.MSP_SET_FEATURE_CONFIG, data)


    def send_RAW_MOTORS(self, data=[]):
        assert(type(data)==list)
        assert(len(data)==8)

        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                      # RC and Motor commands go from 0 to 2000.

        return self.send_RAW_msg(inavutil.msp.MSP_SET_MOTOR, data)


    def send_RAW_RC(self, data=[]):
        """
        When using RX_SERIAL:
        [roll, pitch, yaw, throttle, aux1, aux2,...,aux10]

        When using RX_MSP:
        [roll, pitch, yaw, throttle, aux1, aux2,...,aux14]

        Considering RC_MAP==[0, 1, 3, 2, 4, 5, 6, 7]
        """
        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                      # RC and Motor commands go from 0 to 2000. (900 to 2100 ackshully)

        return self.send_RAW_msg(inavutil.msp.MSP_SET_RAW_RC, data)


    def send_RAW_msg(self, code, data=[], blocking=None, timeout=None, flush=False):
        mspv = 1 if code <= 255 else 2  # choose MSP protocol version
        bufView = mspctrl_prepare_raw_msg(mspv, code, data)
        with self.board.port_write_lock:
            current_write = time.time()
            delta = current_write - self.board.last_write
            if delta < self.board.min_time_between_writes:
                sleeptime = self.board.min_time_between_writes - delta
                time.sleep(sleeptime)

            res = self.board.write(bufView)
            if flush:
                self.board.flush()
            self.board.last_write = current_write
            logging.debug(f"RAW message sent: {bufView}")
            return res



    def process_recv_data(self, dataHandler):
        """Process the dataHandler from receive_msg consuming (pop!) dataHandler['dataView'] as it goes.
        Based on betaflight-configurator (https://git.io/fjRAV)

        Parameters
        ----------
        dataHandler : dict
            Dictionary generated by receive_msg
            
        Returns
        -------
        int
            len(data) when successful or -(error type) if not
        """

        data = dataHandler['dataView'] # DataView (allowing us to view arrayBuffer as struct/union)
        code = dataHandler['code']
        if code == 0: # code==0 means nothing was received...
            logging.debug("Nothing was received - Code 0")
            return -1
        elif dataHandler['crcError']:
            logging.debug("dataHandler has a crcError.")
            return -2
        elif dataHandler['packet_error']:
            logging.debug("dataHandler has a packet_error.")
            return -3
        else:
            if (not dataHandler['unsupported']):
                processor = getattr(self.board, "process_" + inavutil.msp.get(code), None)
                if processor: # if nothing is found, should be None
                    try:
                        if data:
                            processor(data) # use it..
                            return len(data)
                        else:
                            return 0 # because a valid message may contain no data...
                    except IndexError as err:
                        logging.debug('Received data processing error: {}'.format(err))
                        return -4
                else:
                    logging.debug('Unknown code received: {}'.format(code))
                    return -5
            else:
                logging.debug('FC reports unsupported message error - Code {}'.format(code))
                return -6

class MSPy:

    SIGNATURE_LENGTH = 32

    def __init__(self, device, baudrate=115200, trials=1, 
                 logfilename='MSPy.log', logfilemode='a', loglevel='INFO', timeout=1/100,
                 use_tcp=False, min_time_between_writes = 1/100):

        """
        Parameters
        ----------
        device : str
            The location of the serial device (e.g. "/dev/ttyACM0") or TCP port when use_tcp==True
        baudrate : int, optional
            Serial connection speed (default is 115200)
        trials : int, optional
            Number of times it should try openning the serial port before giving up (default is 1)
        logfilename : str, optional
            Name of the file where the log is saved (default is 'MSPy.log').
            If logfilename=None, it will use stdout instead of a file.
        logfilemode : str, optional
            Use 'a' for append and 'w' for overriding (default is 'a'). 
        loglevel : str, optional
            The loglevel passed to logging (default is 'DEBUG')
        timeout : float, optional
            Value used for serial port and TCP timeouts.
        use_tcp : bool, optional
            Uses TCP instead of the serial port.
        min_time_between_writes : float, optional
            Adds delays to make sure messages aren't sent faster than this value.
        """

        self._modules = [processMSP(self), connMSP(self), fastMSP(self)]

        # set as attributes all the hardcoded MSP constants that were once here
        for key, value in vars(msp_vars).items():
            if not key.startswith('_'):
                setattr(self, key, value)


        if logfilename:
            logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                                filename=logfilename, 
                                filemode=logfilemode,
                                level=getattr(logging, loglevel.upper()))
        else:
            logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                                level=getattr(logging, loglevel.upper()),
                                stream=sys.stdout)
        self.min_time_between_writes = min_time_between_writes # it will add a sleep if trying to write / read too fast
        self.use_tcp = use_tcp
        self.timeout = timeout
        self.device = device
        
        self.read_buffer = b''

        if self.use_tcp is False:
            self.conn = serial.Serial()
            self.conn.port = self.device
            self.conn.baudrate = baudrate
            self.conn.bytesize = serial.EIGHTBITS
            self.conn.parity = serial.PARITY_NONE
            self.conn.stopbits = serial.STOPBITS_ONE
            self.conn.timeout = self.timeout
            self.conn.xonxoff = False
            self.conn.rtscts = False
            self.conn.dsrdtr = False
            self.conn.writeTimeout = self.timeout
            self.write = self.conn.write
            self.flush = self.conn.flush
            self.timeout_exception = serial.SerialTimeoutException

            def ser_read():
                _,_,_ = select([self.conn],[],[])  # wait for data
                data = self.conn.read(self.conn.in_waiting) # blocking
                return data
                
            self.read = ser_read
            
            self.start = self.conn.open
        
        else :
            
            socket = TCPSocket()
            self.conn = socket
            self.write = self.conn.send
            self.read = self.conn.receive
            self.start = self.conn.connect
            self.flush = lambda: None
            self.timeout_exception = socket.timeout_exception

        self.ser_trials = trials

        self.port_read_lock = RLock()
        self.port_write_lock = Lock()

        self.INAV = True #changed from False to True!

        self.last_write = time.time()

        # Preparing the method map
        self.method_map = {}
        for module in self._modules:
            for method_name in dir(module):
                # Ensure we only map methods that are callable
                if callable(getattr(module, method_name)) and not method_name.startswith('__'):
                    if method_name not in self.method_map:  # Avoid overriding methods
                        self.method_map[method_name] = getattr(module, method_name)

    def __getattr__(self, name):
        try:
            return self.method_map[name]
        except KeyError:
            raise AttributeError(f"MSPy object has no attribute '{name}'")

    def __enter__(self):
        is_connection_open = not self.connect(trials=self.ser_trials)

        if is_connection_open:
            return self
        else:
            logging.warning(f"{self.device} is not ready/available")
            return 1


    def __exit__(self, exc_type, exc_value, traceback):
        if not self.conn.closed:
            self.conn.close()
