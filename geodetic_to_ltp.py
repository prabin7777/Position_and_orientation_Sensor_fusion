import numpy as np

def geodetic_to_cartesian(lat, lon, alt):
    # Convert latitude and longitude from degrees to radians
    ref_lat = lat[0]
    ref_lon = lon[0]
    ref_alt = alt[0]
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    ref_lat_rad = np.radians(ref_lat)
    ref_lon_rad = np.radians(ref_lon)
    
    # Constants
    a = 6378137.0  # Semi-major axis of WGS84 ellipsoid (meters)
    f = 1 / 298.257223563  # Flattening of WGS84 ellipsoid 
    e_sq = f * (2 - f)  # Eccentricity squared
    
    # Radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e_sq * np.sin(ref_lat_rad)**2)
    
    # Convert reference point to Cartesian coordinates
    ref_x = (N + ref_alt) * np.cos(ref_lat_rad) * np.cos(ref_lon_rad)
    ref_y = (N + ref_alt) * np.cos(ref_lat_rad) * np.sin(ref_lon_rad)
    ref_z = (N * (1 - e_sq) + ref_alt) * np.sin(ref_lat_rad)
    
    # Convert geodetic coordinates to Cartesian coordinates
    x = []
    y = []
    z = []
    for i in range(len(lat)):
        # Convert current point to Cartesian coordinates relative to reference point
        xi = (N + alt[i]) * np.cos(lat_rad[i]) * np.cos(lon_rad[i]) - ref_x
        yi = (N + alt[i]) * np.cos(lat_rad[i]) * np.sin(lon_rad[i]) - ref_y
        zi = (N * (1 - e_sq) + alt[i]) * np.sin(lat_rad[i]) - ref_z
        
        x.append(xi)
        y.append(yi)
        z.append(zi)
    
    return np.array(x), np.array(y), np.array(z)
