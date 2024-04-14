import struct
from unavlib import MSPy

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
lat = int(55.737110 * 1e7)  
lon = int(-5.119426 * 1e7)
alt = 4200  # Altitude in cm as per INAV's handling (if 42 meters, it's 4200 cm).
p1 = 1200   
p2 = 0     
p3 = 0      
flag = 0    
print(lat,lon)
packed_wp = pack_msp_wp(wp_no, action, lat, lon, alt, p1, p2, p3, flag)
hex_string = ' '.join(f"{byte:02x}" for byte in packed_wp)
print(hex_string)

with MSPy(device='/dev/ttyACM0', loglevel='DEBUG', baudrate=115200) as board:
    if board == 1: # an error occurred...
        raise Exception('Connection error')

    print()
    #print('MSP_SET_WP')
    #a = send(board, 'MSP_SET_WP', data=packed_wp)
    #print(a)
    #print(unpack_msp_wp(a['dataView']))

    print()
    print('MSP_WP_GETINFO')
    # https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c#L1423
    ret = send(board, 'MSP_WP_GETINFO', data=[])
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
        b = send(board, 'MSP_WP', data=struct.pack('B', i))
        ret = unpack_msp_wp(b['dataView'])
        print(i, ret)

    #print(send(board, 'MSP_WP_MISSION_SAVE', data=[])) #unsupported

