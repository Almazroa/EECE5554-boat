#!/usr/bin/env python3


# summary: topic name is /water_quality/tds
# msg contains tds_value

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class TDSDriver(Node):
    def __init__(self):
        super().__init__('tds_driver')
        
        # Publisher
        self.tds_pub = self.create_publisher(Float32, '/water_quality/tds', 10)
        
        # Serial connection to arduino
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',  # arduino port#
            baudrate=115200,
            timeout=1.0
        )
        
        # Timer to read serial
        self.timer = self.create_timer(0.1, self.read_tds)  # 10 Hz
        
        self.get_logger().info('TDS Driver started')
    
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
        except Exception as e:
            self.get_logger().error(f'Error reading TDS: {e}')
    
    def __del__(self):
        if hasattr(self, 'serial_port'):
            self.serial_port.close()

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