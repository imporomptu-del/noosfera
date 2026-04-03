#!/usr/bin/env python3

import rclpy
import csv
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
from collections import defaultdict
import subprocess
import time
import os
from datetime import datetime, timezone


class ADSBDump1090Listener(Node):
    def __init__(self):
        super().__init__('adsb_dump1090_listener')
        self.housing_name = "prtype"  # change this to your actual housing name
        self.data_dir = "/home/a/Projects/ros2_ws/data/adsb/"  # or wherever you want to save
        os.makedirs(self.data_dir, exist_ok=True)

        self.csv_lock = threading.Lock()
        self.collected_rows = []  # Temporary storage for the last 5 min


        #self.dump1090_path = '/home/a/Projects/dump1090/'
        self.dump1090_path = '/usr/bin/dump1090-fa' #gahome/a/Projects/dump1090/'


        self.publisher_ = self.create_publisher(String, '/adsb_data', 10)
        self.get_logger().info("ADSB listener started. Connecting to dump1090...")

        self.aircraft_state = defaultdict(dict)  # hex_ident -> latest known values
        
        # Check if dump1090 service is running
        self.check_dump1090_service()

        self.thread = threading.Thread(target=self.listen_to_socket, daemon=True)
        self.thread.start()
        self.create_timer(30.0, self.periodic_tasks)  # Every 30 seconds

    def periodic_tasks(self):
        self.save_csv_snapshot()
        self.publish_snapshot()
    
    def check_dump1090_available(self):
        """Check if dump1090 is available and executable"""
        if os.path.isfile(self.dump1090_path):
            if os.access(self.dump1090_path, os.X_OK):
                self.get_logger().info(f"Found dump1090 at: {self.dump1090_path}")
                return True
            else:
                self.get_logger().error(f"dump1090 exists but is not executable: {self.dump1090_path}")
                self.get_logger().info("Run: chmod +x /home/a/ros2/dump1090/dump1090")
                return False
        else:
            self.get_logger().error(f"dump1090 not found at: {self.dump1090_path}")
            return False
    
    def check_dump1090_service(self):
        """Check if dump1090 service is running"""
        try:
            # Check if dump1090 is running as a system service
            result = subprocess.run(['systemctl', 'is-active', 'dump1090-fa'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout.strip() == 'active':
                self.get_logger().info("✓ dump1090-fa service is running")
                return True
            else:
                self.get_logger().warn("dump1090-fa service is not active")
                self.get_logger().info("Please start it with: sudo systemctl start dump1090-fa")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timeout checking dump1090 service status")
            return False
        except FileNotFoundError:
            self.get_logger().warn("systemctl not found, assuming dump1090 is running")
            return True
        except Exception as e:
            self.get_logger().warn(f"Error checking dump1090 service: {e}")
            return True  # Assume it's running and try to connect    

    def listen_to_socket(self):
        host = 'localhost'
        port = 30003  # SBS-1 TCP port

        # Wait for dump1090 to start
        time.sleep(2)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)  # 10 second timeout
            sock.connect((host, port))
            self.get_logger().info("Connected to dump1090 on port 30003")

            while True:
                try:
                    data = sock.recv(4096)
                    if not data:
                        break

                    decoded = data.decode('utf-8').strip()
                    for line in decoded.splitlines():
                        if line.startswith('MSG'):
                            self.handle_adsb_message(line)

                except socket.timeout:
                    self.get_logger().warn("Socket timeout, retrying...")
                    continue
                except Exception as e:
                    self.get_logger().error(f"Socket read error: {e}")
                    break

        except ConnectionRefusedError:
            self.get_logger().error("Connection refused. Is dump1090 running?")
            self.get_logger().info(f"Try running: {self.dump1090_path} --device-index 0 --gain auto --net")
        except Exception as e:
            self.get_logger().error(f"Socket connection error: {e}")

    def handle_adsb_message(self, msg_line):
        fields = msg_line.split(',')
        if len(fields) < 22:
            return

        msg_type = fields[0]
        transmission_type = fields[1]
        hex_ident = fields[4]

        if not hex_ident:
            return

        # Update the aircraft state with available fields
        ac = self.aircraft_state[hex_ident]
        ac['hex'] = hex_ident
        ac['type'] = transmission_type
        # Use UTC timestamp instead of local time
        ac['timestamp'] = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

        if transmission_type == '1':  # callsign
            if fields[10]:
                ac['callsign'] = fields[10].strip()
        elif transmission_type == '2':  # surface position
            if fields[14] and fields[15]:
                ac['lat'] = fields[14]
                ac['lon'] = fields[15]
            if fields[11]:
                ac['alt'] = fields[11]
        elif transmission_type == '3':  # airborne position
            if fields[14] and fields[15]:
                ac['lat'] = fields[14]
                ac['lon'] = fields[15]
            if fields[11]:
                ac['alt'] = fields[11]
        elif transmission_type == '4':  # airborne velocity
            if fields[12]:
                ac['spd'] = fields[12]
            if fields[13]:
                ac['trk'] = fields[13]
            if fields[16]:
                ac['vrt'] = fields[16]
        elif transmission_type == '5':  # surveillance alt
            if fields[11]:
                ac['alt'] = fields[11]

        # Format output
        msg_str = (
            f"ICAO: {ac.get('hex', '')}, "
            f"Callsign: {ac.get('callsign', '')}, "
            f"Alt: {ac.get('alt', '')} ft, "
            f"Speed: {ac.get('spd', '')} kt, "
            f"Track: {ac.get('trk', '')}°, "
            f"VertRate: {ac.get('vrt', '')} fpm, "
            f"Lat: {ac.get('lat', '')}, "
            f"Lon: {ac.get('lon', '')}, "
            f"Time: {ac.get('timestamp', '')}"
        )

        with self.csv_lock:
            self.collected_rows.append({
                "timestamp": ac.get('timestamp', ''),
                "icao": ac.get('hex', ''),
                "callsign": ac.get('callsign', ''),
                "altitude_ft": ac.get('alt', ''),
                "speed_kt": ac.get('spd', ''),
                "track_deg": ac.get('trk', ''),
                "vertical_rate_fpm": ac.get('vrt', ''),
                "latitude": ac.get('lat', ''),
                "longitude": ac.get('lon', ''),
            })


    def publish_snapshot(self):
        for hex_id, ac in self.aircraft_state.items():
            msg_str = (
                f"ICAO: {ac.get('hex', '')}, "
                f"Callsign: {ac.get('callsign', '')}, "
                f"Alt: {ac.get('alt', '')} ft, "
                f"Speed: {ac.get('spd', '')} kt, "
                f"Track: {ac.get('trk', '')}°, "
                f"VertRate: {ac.get('vrt', '')} fpm, "
                f"Lat: {ac.get('lat', '')}, "
                f"Lon: {ac.get('lon', '')}, "
                f"Time: {ac.get('timestamp', '')}"
            )

            msg = String()
            msg.data = msg_str
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg_str}")  



    def save_csv_snapshot(self):
        utc_now = datetime.now(timezone.utc)
        timestamp_str = utc_now.strftime("%Y_%m_%d_T%H_%M_%S_%f")[:-3]  # Remove last 3 digits to get milliseconds
        filename = f"adsb_{timestamp_str}.csv"
        filepath = os.path.join(self.data_dir, filename)

        with self.csv_lock:
            rows_to_write = self.collected_rows.copy()
            self.collected_rows.clear()

        if not rows_to_write:
            self.get_logger().info("No ADS-B data to write.")
            return

        try:
            with open(filepath, mode='w', newline='') as csvfile:
                fieldnames = [
                    "timestamp", "icao", "callsign", "altitude_ft",
                    "speed_kt", "track_deg", "vertical_rate_fpm",
                    "latitude", "longitude"
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows_to_write)

            self.get_logger().info(f"✓ ADS-B data saved to {filepath}")
        except Exception as e:
            self.get_logger().error(f"✗ Failed to write ADS-B CSV: {e}")              

    def stop_dump1090(self):
        """Stop the dump1090 service (if we started it)"""
        # Since we're using systemctl service, we don't need to stop anything
        # The systemctl service will handle this
        self.get_logger().info("Using systemctl dump1090-fa service - no need to stop manually")


def main(args=None):
    rclpy.init(args=args)
    node = ADSBDump1090Listener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received stop signal")
    finally:
        node.stop_dump1090()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
