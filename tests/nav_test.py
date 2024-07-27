import struct
from unavlib import MSPy
from unavlib.modules.utils import dict_reverse

R_MSPCodes = dict_reverse(MSPy.MSPCodes)

# Define a function to pack data into the MSP waypoint structure
def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
    msp_wp = struct.pack('<BBiiihhhB', wp_no, action, lat, lon, altitude, p1, p2, p3, flag)
    return msp_wp

def unpack_msp_wp(msp_wp):
    unpacked_data = struct.unpack('<BBiiihhhB', msp_wp)
    return unpacked_data

def send(board, msg_code, data=[]):
    if board.send_RAW_msg(MSPy.MSPCodes[msg_code], data=data):
        dataHandler = board.receive_msg()
        board.process_recv_data(dataHandler)
        return dataHandler

wp_no = 1
action = 1 
lat = int(55.732110 * 1e7)  
lon = int(-17.119426 * 1e7)
alt = 5000  # Altitude in cm as per INAV's handling (if 42 meters, it's 4200 cm).
p1 = 1200   
p2 = 0     
p3 = 0      
flag = 0    
print(lat,lon)
packed_wp = pack_msp_wp(wp_no, action, lat, lon, alt, p1, p2, p3, flag)
hex_string = ' '.join(f"{byte:02x}" for byte in packed_wp)
print(hex_string)
ret = {}

#####
# DO NOT CONNECT TO THE HITL PORT
#####
with MSPy(device='/dev/ttyUSB0', loglevel='DEBUG', baudrate=115200) as board:
    if board == 1: # an error occurred...
        raise Exception('Connection error')

    try:
        print()
        try1 = True
        if try1:
            print('MSP_SET_WP')
            ret = send(board, 'MSP_SET_WP', data=packed_wp)
            print(ret)
            if len(ret['dataView'])>0:
                print(unpack_msp_wp(ret['dataView']))
            else:
                print('return len 0')

        print()
        print('MSP_WP_GETINFO')
        # https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c#L1423
        board.send_RAW_msg(MSPy.MSPCodes['MSP_WP_GETINFO'], data=[])
        ret = board.receive_msg()
        print(ret)
        data = struct.unpack('<BBBB', ret['dataView'])
        jsondat = {
            "reserved": data[0],
            "NAV_MAX_WAYPOINTS": data[1],
            "isWaypointListValid": data[2],
            "WaypointCount": data[3]
        }
        for key,value in jsondat.items():
            print(key,value)

        print()
        print('MSP_WP')
        # Special waypoints are 0, 254, and 255. #0 returns the RTH (Home) position, #254 returns the current desired position (e.g. target waypoint), #255 returns the current position.
        for i in range(1,jsondat['WaypointCount']+1):
            board.send_RAW_msg(MSPy.MSPCodes['MSP_WP'],  data=struct.pack('B', i))
            ret = board.receive_msg()
            print(ret)
            b = unpack_msp_wp(ret['dataView'])
            print(i, b)

        #print(send(board, 'MSP_WP_MISSION_SAVE', data=[])) #unsupported

    except struct.error as e:
        print(e, ret['dataView'])