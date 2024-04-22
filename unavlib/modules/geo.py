from geographiclib.geodesic import Geodesic
import mgrs
import math
import geojson

class GPSposition():
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def __str__(self):
        s = "Lattitude: {:.8f} Longitude: {:.8f} Altitude: {:.3f}".format(self.lat, self.lon, self.alt)
        return s

class PosVector():
    def __init__(self, distance, azimuth, elevation):
        self.az = azimuth
        self.dist = distance
        self.elev = elevation
    def __str__(self):
        s = "Distance: {:.3f} Azimuth: {:.3f} Elevation: {:.3f}".format(self.dist, self.az, self.elev)
        return s
    
def latlon_to_mgrs(latlon):
    milobj = mgrs.MGRS()
    return milobj.toMGRS(latlon.lat,latlon.lon,0)

def mgrs_to_latlon(milgrid):
    milobj = mgrs.MGRS()
    return milobj.toLatLon(milgrid)

def gps_to_vector(latlon1, latlon2):
    geod = Geodesic.WGS84
    g = geod.Inverse(latlon1.lat, latlon1.lon, latlon2.lat, latlon2.lon)
    az = g['azi1']
    dist = g['s12']
    if az<0:
        az = az+360
    if latlon1.alt > latlon2.alt:
        relalt = latlon1.alt - latlon2.alt
        elev = math.degrees( math.atan( relalt / dist ) ) * -1
    else:
        relalt = latlon2.alt - latlon1.alt
        elev = math.degrees( math.atan( relalt / dist ) ) 

    return PosVector(dist, az, elev) #dist, azimuth, elev

def vector_to_gps(latlon, dist, az):
    geod = Geodesic.WGS84
    g = geod.Direct(latlon.lat, latlon.lon, az, dist)
    return GPSposition(float(g['lat2']),float(g['lon2']),float(0))

def vector_to_gps_air(latlon, az, ang): #only valid if both points are at same altitude
    geod = Geodesic.WGS84
    truerange = math.tan(math.radians(ang)) * latlon.alt
    slantrange = latlon.alt / math.cos(math.radians(ang))
    #print("SLANT:",slantrange)
    #print("TRUE:",truerange)
    g = geod.Direct(latlon.lat, latlon.lon, az, truerange)
    return GPSposition(float(g['lat2']),float(g['lon2']),float(0))

def vector_rangefinder_to_gps_air(latlon, az, ang, slantrange):
    geod = Geodesic.WGS84
    truerange = math.cos(math.radians(ang))*slantrange
    g = geod.Direct(latlon.lat, latlon.lon, az, truerange)
    return GPSposition(float(g['lat2']),float(g['lon2']),float(0))