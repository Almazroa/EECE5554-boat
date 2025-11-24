#!/usr/bin/env python3

# summary: topic name is /water_quality/tds
# msg contains tds_value

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re
from datetime import datetime
import csv
import os

class TDSDriver(Node):
    def __init__(self):
        super().__init__('tds_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('log_directory', 'tds_logs')
        
        serial_port_addr = self.get_parameter('port').get_parameter_value().string_value
        self.enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value
        log_dir = self.get_parameter('log_directory').get_parameter_value().string_value
        
        # Publisher
        self.tds_pub = self.create_publisher(Float32, '/water_quality/tds', 10)
        
        # Serial connection to arduino
        self.serial_port = serial.Serial(
            port=serial_port_addr,
            baudrate=115200,
            timeout=1.0
        )
        
        # Setup logging if enabled
        self.csv_file = None
        self.csv_writer = None
        if self.enable_logging:
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_filename = os.path.join(log_dir, f'tds_log_{timestamp}.csv')
            
            self.csv_file = open(log_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            self.csv_writer.writerow([
                'Timestamp',
                'TDS_ppm',
                'Raw_Reading'
            ])
            self.csv_file.flush()
            
            self.get_logger().info(f'Logging TDS data to: {log_filename}')
        else:
            self.get_logger().info('TDS logging disabled')
        
        # Timer to read serial
        self.timer = self.create_timer(0.1, self.read_tds)  # 10 Hz
        
        self.get_logger().info(f'TDS Driver started on port {serial_port_addr}')
    
    def read_tds(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Parse "TDS:X.XX"
                match = re.match(r'TDS:([\d.]+)', line)
                if match:
                    tds_value = float(match.group(1))
                    
                    # Publish
                    msg = Float32()
                    msg.data = tds_value
                    self.tds_pub.publish(msg)
                    
                    self.get_logger().info(f'TDS: {tds_value:.2f} ppm')
                    
                    # Log to CSV if enabled
                    if self.enable_logging and self.csv_writer:
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        self.csv_writer.writerow([
                            timestamp,
                            f'{tds_value:.2f}',
                            line
                        ])
                        self.csv_file.flush()
                        
        except Exception as e:
            self.get_logger().error(f'Error reading TDS: {e}')
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info('TDS log file closed')

def main(args=None):
    rclpy.init(args=args)
    node = TDSDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
