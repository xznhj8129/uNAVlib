import socket
from ..enums import inav_enums, msp_codes

def dict_index(d, target_value):
    return [key for key, value in d.items() if value == target_value]

def dict_reverse(d):
    return {v: i for i,v in d.items()}

class TCPSocket:
    """TCP Socket for Software In The Loop (SITL)
    """

    def close(self):
        if not self.sock:
            raise Exception("Cannot close, socket never created")
        self.closed = True
        self.sock.close()

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            self.buffersize = self.sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
        else:
            self.sock = sock
        self.closed = False
        self.timeout_exception = socket.timeout

    def connect(self, host='127.0.0.1', port=54320, timeout=1/500):
        self.sock.connect((host, port))
        self.sock.settimeout(timeout)
        self.closed = False

    def send(self, msg):
        sent = self.sock.send(msg)
        if not sent:
            raise RuntimeError("socket connection broken")
        return sent

    def receive(self, size = None):
        recvbuffer = b''
        try:
            if size:
                recvbuffer = self.sock.recv(size)
            else:
                recvbuffer = self.sock.recv(self.buffersize)
        except socket.timeout:
            return recvbuffer
        if not recvbuffer:
            raise RuntimeError("socket connection broken")

        return recvbuffer
class MSPContainer:
    """Container for MSP-related enums"""
    pass

class EnumContainer:
    """Container for other enums"""
    pass

# Function to convert a dict to a class and add it to a container class
def dict_2_class_add_to_class(container_class, enum_name, enum_dict):
    reverse_dict = {v: k for k, v in enum_dict.items()}

    class EnumClass:
        pass

    for var_name, data in enum_dict.items():
        vn = var_name.replace(' ', '_')
        try:
            dat = int(data)
        except:
            dat = data
        setattr(EnumClass, vn, dat)

    def get(cls, value):
        return reverse_dict.get(value)
    
    setattr(EnumClass, 'get', classmethod(get))
    EnumClass.__name__ = enum_name
    enum_class = EnumClass

    setattr(container_class, enum_name, enum_class)

inavutil = EnumContainer()
for attr_name in dir(inav_enums):
    if not attr_name.startswith("__"):
        attr = getattr(inav_enums, attr_name)
        if isinstance(attr, dict): 
            dict_2_class_add_to_class(inavutil, attr_name, attr)

dict_2_class_add_to_class(inavutil, "msp", msp_codes.MSPCodes)
#setattr(inavutil, 'msp', msp)
