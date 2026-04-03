import time
import csv
from datetime import datetime, timezone
import os
import signal
import sys
from smbus2 import SMBus
from math import pi

# ——— Configuration ———
I2C_BUS = 1
ADDR    = 0x28

# BNO055 register map
_OPR_MODE            = 0x3D
_EULER_LSB           = 0x1A
_QUAT_LSB            = 0x20
_GYRO_LSB            = 0x14
_ACCEL_LSB           = 0x08
_MAG_LSB             = 0x0E
_LIN_ACC_LSB         = 0x28
_GRAVITY_LSB         = 0x2E
_TEMP_REG            = 0x34

# Scaling factors
SF_EULER      = 1/16.0
SF_QUAT       = 1/16384.0
SF_GYRO       = (1/16.0)*(pi/180)
SF_ACCEL      = 1/100.0
SF_MAG        = 1/16.0
SF_LIN_ACC    = 1/100.0
SF_GRAVITY    = 1/100.0

def twos_complement(val, bits=16):
    if val & (1 << (bits-1)):
        val -= (1 << bits)
    return val

# Output path for SSD logging
output_dir = "/home/a/Projects/ros2_ws/data/imu/"
os.makedirs(output_dir, exist_ok=True)

# File rotation settings
FILE_ROTATION_INTERVAL = 5 * 60  # 5 minutes in seconds
current_file = None
writer = None
file_start_time = None

def create_new_file():
    """Create a new CSV file with timestamp in filename"""
    global current_file, writer, file_start_time
    if current_file:
        current_file.close()
    
    filename = os.path.join(output_dir, f"imu_bno0555_{datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S_%f')[:-3]}.csv")
    current_file = open(filename, mode='w', newline='')
    writer = csv.writer(current_file)
    writer.writerow([
        "timestamp", "heading", "roll", "pitch",
        "qw", "qx", "qy", "qz",
        "gx", "gy", "gz",
        "ax", "ay", "az",
        "mx", "my", "mz",
        "lax", "lay", "laz",
        "gxv", "gyv", "gzv",
        "temp"
    ])
    file_start_time = time.time()
    print(f"Created new log file: {filename}")

def should_rotate_file():
    """Check if it's time to rotate to a new file"""
    return time.time() - file_start_time >= FILE_ROTATION_INTERVAL

def cleanup_and_exit(signum, frame):
    """Clean up resources and exit gracefully"""
    global current_file
    if current_file:
        current_file.close()
        print("File closed gracefully")
    sys.exit(0)

# Set up signal handlers for graceful shutdown
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

with SMBus(I2C_BUS) as bus:
    # Switch to NDOF mode
    bus.write_byte_data(ADDR, _OPR_MODE, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, _OPR_MODE, 0x0C)
    time.sleep(0.05)

    # Create the first file
    create_new_file()

    while True:
        # Check if we need to rotate to a new file
        if should_rotate_file():
            print(f"Rotating file after {FILE_ROTATION_INTERVAL/60:.1f} minutes")
            create_new_file()

        euler = bus.read_i2c_block_data(ADDR, _EULER_LSB, 6)
        quat  = bus.read_i2c_block_data(ADDR, _QUAT_LSB, 8)
        gyro  = bus.read_i2c_block_data(ADDR, _GYRO_LSB, 6)
        accel = bus.read_i2c_block_data(ADDR, _ACCEL_LSB, 6)
        mag   = bus.read_i2c_block_data(ADDR, _MAG_LSB, 6)
        lin   = bus.read_i2c_block_data(ADDR, _LIN_ACC_LSB, 6)
        grav  = bus.read_i2c_block_data(ADDR, _GRAVITY_LSB, 6)
        temp  = bus.read_byte_data(ADDR, _TEMP_REG)

        heading = twos_complement(euler[1]<<8 | euler[0]) * SF_EULER
        roll    = twos_complement(euler[3]<<8 | euler[2]) * SF_EULER
        pitch   = twos_complement(euler[5]<<8 | euler[4]) * SF_EULER

        qw = twos_complement(quat[1]<<8 | quat[0]) * SF_QUAT
        qx = twos_complement(quat[3]<<8 | quat[2]) * SF_QUAT
        qy = twos_complement(quat[5]<<8 | quat[4]) * SF_QUAT
        qz = twos_complement(quat[7]<<8 | quat[6]) * SF_QUAT

        gx = twos_complement(gyro[1]<<8 | gyro[0]) * SF_GYRO
        gy = twos_complement(gyro[3]<<8 | gyro[2]) * SF_GYRO
        gz = twos_complement(gyro[5]<<8 | gyro[4]) * SF_GYRO

        ax = twos_complement(accel[1]<<8 | accel[0]) * SF_ACCEL
        ay = twos_complement(accel[3]<<8 | accel[2]) * SF_ACCEL
        az = twos_complement(accel[5]<<8 | accel[4]) * SF_ACCEL

        mx = twos_complement(mag[1]<<8 | mag[0]) * SF_MAG
        my = twos_complement(mag[3]<<8 | mag[2]) * SF_MAG
        mz = twos_complement(mag[5]<<8 | mag[4]) * SF_MAG

        lax = twos_complement(lin[1]<<8 | lin[0]) * SF_LIN_ACC
        lay = twos_complement(lin[3]<<8 | lin[2]) * SF_LIN_ACC
        laz = twos_complement(lin[5]<<8 | lin[4]) * SF_LIN_ACC

        gxv = twos_complement(grav[1]<<8 | grav[0]) * SF_GRAVITY
        gyv = twos_complement(grav[3]<<8 | grav[2]) * SF_GRAVITY
        gzv = twos_complement(grav[5]<<8 | grav[4]) * SF_GRAVITY

        timestamp = datetime.now(timezone.utc).isoformat()

        writer.writerow([
            timestamp, heading, roll, pitch,
            qw, qx, qy, qz,
            gx, gy, gz,
            ax, ay, az,
            mx, my, mz,
            lax, lay, laz,
            gxv, gyv, gzv,
            temp
        ])
        current_file.flush()
        time.sleep(1)  # Log once per second
