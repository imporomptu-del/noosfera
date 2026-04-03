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

def create_new_file():
    """Create a new CSV file with timestamp in filename"""
    global current_file, writer, file_start_time
    if current_file:
        current_file.close()
    
    filename = os.path.join(output_dir, f"gps_{datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S_%f')[:-3]}.csv")
    current_file = open(filename, mode='w', newline='')
    writer = csv.writer(current_file)
    writer.writerow([
        "timestamp", "sentence_type", "latitude", "longitude", "altitude_m",
        "speed_knots", "course_degrees", "num_satellites", "hdop", "raw_nmea"
    ])
    file_start_time = time.time()
    print(f"Created new GPS log file: {filename}")

def should_rotate_file():
    """Check if it's time to rotate to a new file"""
    return time.time() - file_start_time >= FILE_ROTATION_INTERVAL

def cleanup_and_exit(signum, frame):
    """Clean up resources and exit gracefully"""
    global current_file
    if current_file:
        current_file.close()
        print("GPS file closed gracefully")
    ser.close()
    sys.exit(0)

# Set up signal handlers for graceful shutdown
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Create the first file
create_new_file()

try:
    while True:
        # Check if we need to rotate to a new file
        if should_rotate_file():
            print(f"Rotating GPS file after {FILE_ROTATION_INTERVAL/60:.1f} minute(s)")
            create_new_file()
        
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            print(f"Raw NMEA: {line}")
            if line.startswith('$'):
                try:
                    msg = pynmea2.parse(line)
                    timestamp = datetime.now(timezone.utc).isoformat()
                    
                    # Initialize variables
                    latitude = longitude = altitude = speed = course = ""
                    num_sats = hdop = ""
                    
                    if msg.sentence_type == 'GGA':
                        latitude = msg.latitude if msg.latitude else ""
                        longitude = msg.longitude if msg.longitude else ""
                        altitude = msg.altitude if msg.altitude else ""
                        num_sats = msg.num_sats if hasattr(msg, 'num_sats') else ""
                        hdop = msg.horizontal_dil if hasattr(msg, 'horizontal_dil') else ""
                        print(f"Time: {msg.timestamp}, Lat: {latitude}, Lon: {longitude}, Alt: {altitude} m")
                    elif msg.sentence_type == 'RMC':
                        latitude = msg.latitude if msg.latitude else ""
                        longitude = msg.longitude if msg.longitude else ""
                        speed = msg.spd_over_grnd if msg.spd_over_grnd else ""
                        course = msg.true_course if msg.true_course else ""
                        print(f"Time: {msg.timestamp}, Speed: {speed} knots, Course: {course}°")
                    
                    # Write to CSV
                    writer.writerow([
                        timestamp, msg.sentence_type, latitude, longitude, altitude,
                        speed, course, num_sats, hdop, line
                    ])
                    current_file.flush()
                    
                except pynmea2.ParseError:
                    pass
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping...")
finally:
    if current_file:
        current_file.close()
    ser.close()
