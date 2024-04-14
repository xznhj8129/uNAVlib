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
from .modules.boardconn import connMSP
from .modules.process import processMSP
from .modules.fast_functions import fastMSP


class MSPy:

    SIGNATURE_LENGTH = 32
    MSPCodes = msp_codes.MSPCodes
    R_MSPCodes = dict_reverse(MSPCodes)

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

        self.INAV = False

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
            raise AttributeError(f"'Facade' object has no attribute '{name}'")

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

for attr_name in dir(inav_enums):
    if not attr_name.startswith("__"):
        attr = getattr(inav_enums, attr_name)
        if isinstance(attr, dict): 
            reversed_dict = dict_reverse(attr)
            setattr(MSPy, f"R_{attr_name}", reversed_dict)
            setattr(MSPy, attr_name, attr)