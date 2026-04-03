#!/usr/bin/env python3
import serial
import time
import pynmea2
import csv
import os
from datetime import datetime, timezone
import signal
import sys

# Configure the serial port
serial_port = "/dev/ttyTHS1"  # Confirmed from minicom
baud_rate = 9600

# Output path for GPS logging
output_dir = "/home/a/Projects/ros2_ws/data/gps/"
os.makedirs(output_dir, exist_ok=True)

# File rotation settings
FILE_ROTATION_INTERVAL = 1 * 60  # 1 minute in seconds
current_file = None
writer = None
file_start_time = None

# Store latest GPS state
gps_state = {
    'latitude': None,
    'longitude': None,
    'altitude_m': None,
    'speed_knots': None,
    'speed_kmh': None,
    'speed_mph': None,
    'course_degrees': None,
    'num_satellites': None,
    'hdop': None,
    'fix_quality': None,
    'gps_time': None,
    'date': None
}

def create_new_file():
    """Create a new CSV file with timestamp in filename"""
    global current_file, writer, file_start_time
    if current_file:
        current_file.close()
    
    filename = os.path.join(output_dir, f"gps_{datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S_%f')[:-3]}.csv")
    current_file = open(filename, mode='w', newline='')
    writer = csv.writer(current_file)
    
    # Clear, descriptive column names
    writer.writerow([
        "utc_timestamp",           # System UTC timestamp
        "gps_date",                # Date from GPS
        "gps_time",                # Time from GPS
        "latitude_deg",            # Latitude in decimal degrees
        "longitude_deg",           # Longitude in decimal degrees
        "altitude_m",              # Altitude in meters
        "speed_knots",             # Speed in knots
        "speed_kmh",               # Speed in km/h
        "speed_mph",               # Speed in mph
        "course_degrees",          # Direction of travel (0-360)
        "num_satellites",          # Number of satellites in view
        "fix_quality",             # GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
        "hdop",                    # Horizontal dilution of precision
        "latitude_direction",      # N or S
        "longitude_direction",     # E or W
    ])
    file_start_time = time.time()
    print(f"\n✅ Created new GPS log file: {filename}\n")

def should_rotate_file():
    """Check if it's time to rotate to a new file"""
    return time.time() - file_start_time >= FILE_ROTATION_INTERVAL

def cleanup_and_exit(signum, frame):
    """Clean up resources and exit gracefully"""
    global current_file
    print("\n\n🛑 Stopping GPS logging...")
    if current_file:
        current_file.close()
        print("✅ GPS file closed gracefully")
    ser.close()
    sys.exit(0)

def convert_speed(speed_knots):
    """Convert speed from knots to other units"""
    if speed_knots is None or speed_knots == "":
        return None, None
    try:
        speed_knots = float(speed_knots)
        speed_kmh = speed_knots * 1.852
        speed_mph = speed_knots * 1.15078
        return round(speed_kmh, 2), round(speed_mph, 2)
    except:
        return None, None

def get_direction(value, pos_char, neg_char):
    """Get direction character (N/S or E/W)"""
    if value is None or value == "":
        return ""
    try:
        return pos_char if float(value) >= 0 else neg_char
    except:
        return ""

def print_gps_status():
    """Print formatted GPS status to terminal"""
    print("\n" + "="*70)
    print("📡 GPS STATUS")
    print("="*70)
    
    if gps_state['latitude'] and gps_state['longitude']:
        print(f"📍 Position:")
        print(f"   Latitude:  {gps_state['latitude']:.6f}° {get_direction(gps_state['latitude'], 'N', 'S')}")
        print(f"   Longitude: {gps_state['longitude']:.6f}° {get_direction(gps_state['longitude'], 'E', 'W')}")
    else:
        print(f"📍 Position: NO FIX")
    
    if gps_state['altitude_m']:
        print(f"🏔️  Altitude: {gps_state['altitude_m']:.1f} m ({gps_state['altitude_m']*3.28084:.1f} ft)")
    
    if gps_state['speed_knots']:
        print(f"🚀 Speed:")
        print(f"   {gps_state['speed_knots']:.1f} knots | {gps_state['speed_kmh']:.1f} km/h | {gps_state['speed_mph']:.1f} mph")
    
    if gps_state['course_degrees']:
        print(f"🧭 Course: {gps_state['course_degrees']:.1f}°")
    
    if gps_state['num_satellites']:
        print(f"🛰️  Satellites: {gps_state['num_satellites']}")
    
    if gps_state['hdop']:
        print(f"📊 HDOP: {gps_state['hdop']} (accuracy indicator)")
    
    if gps_state['fix_quality'] is not None:
        quality_text = {0: "Invalid", 1: "GPS Fix", 2: "DGPS Fix"}
        print(f"✅ Fix Quality: {quality_text.get(gps_state['fix_quality'], 'Unknown')}")
    
    if gps_state['gps_time']:
        print(f"⏰ GPS Time: {gps_state['gps_time']} UTC")
    
    print("="*70 + "\n")

# Set up signal handlers for graceful shutdown
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Initialize the serial connection
print("🔌 Connecting to GPS module...")
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"✅ Connected to {serial_port} at {baud_rate} baud")
except Exception as e:
    print(f"❌ Error connecting to GPS: {e}")
    sys.exit(1)

# Create the first file
create_new_file()

last_status_print = time.time()
STATUS_PRINT_INTERVAL = 5  # Print status every 5 seconds

try:
    print("📡 Listening for GPS data...\n")
    while True:
        # Check if we need to rotate to a new file
        if should_rotate_file():
            print(f"🔄 Rotating GPS file after {FILE_ROTATION_INTERVAL/60:.1f} minute(s)")
            create_new_file()
        
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line and line.startswith('$'):
            try:
                msg = pynmea2.parse(line)
                utc_timestamp = datetime.now(timezone.utc).isoformat()
                
                # Parse GGA - Position and altitude
                if msg.sentence_type == 'GGA':
                    gps_state['latitude'] = float(msg.latitude) if msg.latitude else None
                    gps_state['longitude'] = float(msg.longitude) if msg.longitude else None
                    gps_state['altitude_m'] = float(msg.altitude) if msg.altitude else None
                    gps_state['num_satellites'] = int(msg.num_sats) if hasattr(msg, 'num_sats') and msg.num_sats else None
                    gps_state['hdop'] = float(msg.horizontal_dil) if hasattr(msg, 'horizontal_dil') and msg.horizontal_dil else None
                    gps_state['fix_quality'] = int(msg.gps_qual) if hasattr(msg, 'gps_qual') and msg.gps_qual else None
                    gps_state['gps_time'] = str(msg.timestamp) if msg.timestamp else None
                
                # Parse RMC - Speed and course
                elif msg.sentence_type == 'RMC':
                    gps_state['latitude'] = float(msg.latitude) if msg.latitude else None
                    gps_state['longitude'] = float(msg.longitude) if msg.longitude else None
                    gps_state['speed_knots'] = float(msg.spd_over_grnd) if msg.spd_over_grnd else None
                    gps_state['course_degrees'] = float(msg.true_course) if msg.true_course else None
                    gps_state['gps_time'] = str(msg.timestamp) if msg.timestamp else None
                    gps_state['date'] = str(msg.datestamp) if hasattr(msg, 'datestamp') and msg.datestamp else None
                    
                    # Convert speed to different units
                    if gps_state['speed_knots'] is not None:
                        gps_state['speed_kmh'], gps_state['speed_mph'] = convert_speed(gps_state['speed_knots'])
                
                # Write to CSV with all available data
                writer.writerow([
                    utc_timestamp,
                    gps_state.get('date', ''),
                    gps_state.get('gps_time', ''),
                    gps_state.get('latitude', ''),
                    gps_state.get('longitude', ''),
                    gps_state.get('altitude_m', ''),
                    gps_state.get('speed_knots', ''),
                    gps_state.get('speed_kmh', ''),
                    gps_state.get('speed_mph', ''),
                    gps_state.get('course_degrees', ''),
                    gps_state.get('num_satellites', ''),
                    gps_state.get('fix_quality', ''),
                    gps_state.get('hdop', ''),
                    get_direction(gps_state.get('latitude'), 'N', 'S'),
                    get_direction(gps_state.get('longitude'), 'E', 'W'),
                ])
                current_file.flush()
                
                # Print status periodically
                if time.time() - last_status_print >= STATUS_PRINT_INTERVAL:
                    print_gps_status()
                    last_status_print = time.time()
                
            except pynmea2.ParseError as e:
                print(f"⚠️  Parse error: {e}")
            except Exception as e:
                print(f"⚠️  Error processing GPS data: {e}")
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    cleanup_and_exit(None, None)
finally:
    if current_file:
        current_file.close()
    ser.close()

