#!/usr/bin/env python3

import serial
import utm
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from datetime import datetime
import csv
import os

class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('log_directory', 'gps_logs')
        
        serial_port_addr = self.get_parameter('port').get_parameter_value().string_value
        self.enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value
        log_dir = self.get_parameter('log_directory').get_parameter_value().string_value
        
        # Create publisher for standard GPS message
        self.publisher = self.create_publisher(NavSatFix, '/gps_data', 10)
        
        # Setup serial port
        self.serial_port = serial.Serial(serial_port_addr, 4800, timeout=0.1)
        self.get_logger().info(f'GPS Driver started on port {serial_port_addr}')
        
        # Setup logging if enabled
        self.csv_file = None
        self.csv_writer = None
        if self.enable_logging:
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_filename = os.path.join(log_dir, f'gps_log_{timestamp}.csv')
            
            self.csv_file = open(log_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            self.csv_writer.writerow([
                'Timestamp',
                'Latitude',
                'Longitude',
                'Altitude_m',
                'UTM_Easting',
                'UTM_Northing',
                'UTM_Zone',
                'UTM_Letter',
                'HDOP',
                'Fix_Quality',
                'Raw_GPGGA'
            ])
            self.csv_file.flush()
            
            self.get_logger().info(f'Logging GPS data to: {log_filename}')
        else:
            self.get_logger().info('GPS logging disabled')
        
        # Create timer to read GPS data periodically
        self.timer = self.create_timer(0.1, self.read_gps_data)
    
    def read_gps_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                gpgga_read = self.serial_port.readline().decode('utf-8').strip()
                
                if gpgga_read and self.isGPGGAinString(gpgga_read):
                    self.process_gpgga_string(gpgga_read)
                    
            except Exception as e:
                self.get_logger().error(f'Error reading GPS data: {e}')
    
    def process_gpgga_string(self, gpgga_read):
        # Parse the string using your existing functions
        gpgga_split = gpgga_read.split(',')
        
        if len(gpgga_split) >= 15:
            try:
                # Extract data
                utc = float(gpgga_split[1]) if gpgga_split[1] else 0.0
                latitude = float(gpgga_split[2]) if gpgga_split[2] else 0.0
                latitude_dir = gpgga_split[3]
                longitude = float(gpgga_split[4]) if gpgga_split[4] else 0.0
                longitude_dir = gpgga_split[5]
                fix_quality = int(gpgga_split[6]) if gpgga_split[6] else 0
                altitude = float(gpgga_split[9]) if gpgga_split[9] else 0.0
                hdop = float(gpgga_split[8]) if gpgga_split[8] else 0.0
                
                # Apply your conversions
                latitude_signed = self.degMinstoDegDec(latitude)
                longitude_signed = self.degMinstoDegDec(longitude)
                latitude_signed = self.LatLongSignConvetion(latitude_signed, latitude_dir)
                longitude_signed = self.LatLongSignConvetion(longitude_signed, longitude_dir)
                
                # Calculate UTM
                try:
                    utm_vals = utm.from_latlon(latitude_signed, longitude_signed)
                    utm_easting = utm_vals[0]
                    utm_northing = utm_vals[1]
                    utm_zone = utm_vals[2]
                    utm_letter = utm_vals[3]
                except:
                    utm_easting = utm_northing = 0.0
                    utm_zone = 0
                    utm_letter = ''
                
                # Create standard NavSatFix message
                gps_msg = NavSatFix()
                
                # Header
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'
                
                # GPS Status
                gps_msg.status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
                gps_msg.status.service = NavSatStatus.SERVICE_GPS
                
                # Position
                gps_msg.latitude = latitude_signed
                gps_msg.longitude = longitude_signed
                gps_msg.altitude = altitude
                
                # Position covariance (estimate based on HDOP)
                position_variance = (hdop * 5.0) ** 2
                gps_msg.position_covariance = [
                    position_variance, 0.0, 0.0,
                    0.0, position_variance, 0.0,
                    0.0, 0.0, position_variance
                ]
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                
                # Publish the message
                self.publisher.publish(gps_msg)
                self.get_logger().info(
                    f'GPS: Lat={latitude_signed:.6f}, Lon={longitude_signed:.6f}, '
                    f'Alt={altitude:.2f}m, HDOP={hdop:.1f}'
                )
                
                # Log to CSV if enabled
                if self.enable_logging and self.csv_writer:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    self.csv_writer.writerow([
                        timestamp,
                        f'{latitude_signed:.8f}',
                        f'{longitude_signed:.8f}',
                        f'{altitude:.2f}',
                        f'{utm_easting:.2f}',
                        f'{utm_northing:.2f}',
                        f'{utm_zone}',
                        f'{utm_letter}',
                        f'{hdop:.2f}',
                        f'{fix_quality}',
                        gpgga_read
                    ])
                    self.csv_file.flush()
                
            except Exception as e:
                self.get_logger().error(f'Error processing GPGGA string: {e}')

    # Your existing parsing functions (now as class methods)
    def isGPGGAinString(self, inputString):
        return '$GPGGA' in inputString

    def degMinstoDegDec(self, LatOrLong):
        deg = LatOrLong // 100
        mins = LatOrLong % 100
        degDec = mins / 60
        return deg + degDec

    def LatLongSignConvetion(self, LatOrLong, LatOrLongDir):
        if LatOrLongDir == 'S' or LatOrLongDir == 'W':
            LatOrLong *= -1
        return LatOrLong
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info('GPS log file closed')

def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPSDriver()
    
    try:
        rclpy.spin(gps_driver)
    except KeyboardInterrupt:
        pass
    finally:
        gps_driver.serial_port.close()
        gps_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
