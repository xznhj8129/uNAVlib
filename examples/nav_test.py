import struct

# Define a function to pack data into the MSP waypoint structure
def pack_msp_wp(wp_no, action, lat, lon, altitude, p1, p2, p3, flag):
    msp_wp = struct.pack('<BBIIIHHHB', wp_no, action, lat, lon, altitude, p1, p2, p3, flag)
    return msp_wp

# Example usage:
wp_no = 1
action = 209 
lat = int(54.137110 * 1e7)  
lon = int(-4.719426 * 1e7)
alt = 4200  # Altitude in cm as per INAV's handling (if 42 meters, it's 4200 cm).
p1 = 1200   
p2 = 0     
p3 = 0      
flag = 0    
print(lat,lon)
packed_wp = pack_msp_wp(wp_no, action, lat, lon, alt, p1, p2, p3, flag)
hex_string = ' '.join(f"{byte:02x}" for byte in packed_wp)
print(hex_string)
