
from ..modules import msp_ctrl
from ..enums import msp_codes
from ..enums import msp_vars
from ..enums import inav_enums
from ..modules.utils import inavutil

from serial import SerialException
import logging
import struct
import time
import sys
import serial
from select import select

if "linux" in sys.platform:
    import ctypes
    ffs = ctypes.cdll.LoadLibrary('libc.so.6').ffs # this is only for ffs... it should be directly implemented.
else:
    def ffs(x): # modified from https://stackoverflow.com/a/36059264
        return (x&-x).bit_length()

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
            return msp_ctrl.receive_raw_msg(self.board.read, logging, self.board.timeout_exception, size, timeout)


    def receive_msg(self):
        current_write = time.time()
        if (current_write-self.board.last_write) < self.board.min_time_between_writes:
            time.sleep(max(self.board.min_time_between_writes-(current_write-self.board.last_write),0))
        with self.board.port_read_lock:
            return msp_ctrl.receive_msg(self.board.read, logging)


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
        mspv = 1 if code <= 255 else 2 # should i set to 2 always?
        bufView = msp_ctrl.prepare_RAW_msg(mspv, code, data)
        with self.board.port_write_lock:
            current_write = time.time()
            if (current_write-self.board.last_write) < self.board.min_time_between_writes:
                sleeptime = max(self.board.min_time_between_writes-(current_write-self.board.last_write))
                time.sleep(sleeptime,0)
                #print('SLEEPING',sleeptime)

            res = self.board.write(bufView)
            if flush:
                self.board.flush()
            self.board.last_write = current_write
            logging.debug("RAW message sent: {}".format(bufView))
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