#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from customgps.msg import Customgps
from datetime import datetime
import csv
import os
import threading
import time

# Import the standalone webcam recorder
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from webcam_recorder import auto_record_webcam


class BoatMeasurementDriver(Node):
    def __init__(self):
        super().__init__('boat_measurement_driver')
        
        # Parameters
        self.declare_parameter('microsd_path', '/media/microsd')
        self.declare_parameter('measurement_duration', 300)  # 5 minutes
        
        self.microsd_path = self.get_parameter('microsd_path').value
        self.duration = self.get_parameter('measurement_duration').value
        
        # Subscribe to TDS data
        self.tds_sub = self.create_subscription(
            Float32,
            '/water_quality/tds',
            self.tds_callback,
            10
        )
        
        # Subscribe to GPS data
        self.gps_sub = self.create_subscription(
            Customgps,
            '/gps_data',
            self.gps_callback,
            10
        )
        
        # Measurement state
        self.is_measuring = False
        self.csv_file = None
        self.csv_writer = None
        self.measurement_start_time = None
        self.tds_readings = []
        
        # Latest GPS data
        self.latest_gps = None
        self.gps_lock = threading.Lock()
        
        self.get_logger().info('Boat Measurement Driver initialized')
        self.get_logger().info('Waiting for TDS and GPS data...')
    
    def gps_callback(self, msg):
        """Store latest GPS position"""
        with self.gps_lock:
            self.latest_gps = msg
    
    def tds_callback(self, msg):
        """Receive TDS readings and log to SD card with GPS location"""
        if not self.is_measuring:
            return
            
        try:
            # Get timestamp
            elapsed = time.time() - self.measurement_start_time
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # Get GPS data (thread-safe)
            with self.gps_lock:
                if self.latest_gps:
                    latitude = self.latest_gps.latitude
                    longitude = self.latest_gps.longitude
                    altitude = self.latest_gps.altitude
                    utm_easting = self.latest_gps.utm_easting
                    utm_northing = self.latest_gps.utm_northing
                    hdop = self.latest_gps.hdop
                else:
                    # No GPS fix yet
                    latitude = longitude = altitude = 0.0
                    utm_easting = utm_northing = hdop = 0.0
            
            # Write to CSV
            self.csv_writer.writerow([
                timestamp,
                f'{elapsed:.2f}',
                f'{msg.data:.2f}',
                f'{latitude:.8f}',
                f'{longitude:.8f}',
                f'{altitude:.2f}',
                f'{utm_easting:.2f}',
                f'{utm_northing:.2f}',
                f'{hdop:.2f}'
            ])
            self.csv_file.flush()  # Ensure data is written immediately
            
            # Store for statistics
            self.tds_readings.append(msg.data)
            
            # Log every 10 readings
            if len(self.tds_readings) % 10 == 0:
                self.get_logger().info(
                    f'Sample #{len(self.tds_readings)}: TDS={msg.data:.2f} ppm, '
                    f'GPS=({latitude:.6f}, {longitude:.6f}), '
                    f'Elapsed={elapsed:.1f}s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error logging data: {e}')
    
    def start_measurement(self):
        """Start measurement run - triggers webcam and begins TDS/GPS logging"""
        if self.is_measuring:
            self.get_logger().warn('Measurement already in progress!')
            return False
        
        # Check if we have GPS fix
        if self.latest_gps is None:
            self.get_logger().warn('No GPS fix yet! Waiting for GPS data...')
            # Allow measurement to start anyway, will log 0,0 until GPS locks
            
        try:
            # Create data directory on microSD
            data_dir = os.path.join(self.microsd_path, 'tds_data')
            os.makedirs(data_dir, exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_filename = os.path.join(data_dir, f'tds_log_{timestamp}.csv')
            
            # Open CSV file
            self.csv_file = open(csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header with GPS columns
            self.csv_writer.writerow([
                'Timestamp',
                'Elapsed_Seconds',
                'TDS_ppm',
                'Latitude',
                'Longitude',
                'Altitude_m',
                'UTM_Easting',
                'UTM_Northing',
                'HDOP'
            ])
            self.csv_file.flush()
            
            # Reset state
            self.is_measuring = True
            self.measurement_start_time = time.time()
            self.tds_readings = []
            
            # Start webcam recording
            self.get_logger().info('Starting webcam recording...')
            auto_record_webcam(duration_minutes=self.duration / 60)
            
            # Start auto-stop timer
            stop_thread = threading.Thread(
                target=self._auto_stop_measurement,
                args=(self.duration,)
            )
            stop_thread.start()
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('MEASUREMENT RUN STARTED')
            self.get_logger().info(f'Recording TDS + GPS data to: {csv_filename}')
            self.get_logger().info(f'Webcam recording for {self.duration}s ({self.duration/60:.1f} min)')
            self.get_logger().info('=' * 60)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start measurement: {e}')
            self.stop_measurement()
            return False
    
    def _auto_stop_measurement(self, duration):
        """Auto-stop measurement after duration"""
        time.sleep(duration)
        self.stop_measurement()
    
    def stop_measurement(self):
        """Stop measurement run and finalize data"""
        if not self.is_measuring:
            return
            
        self.is_measuring = False
        
        # Close CSV file
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            
        # Calculate statistics
        if self.tds_readings:
            avg_tds = sum(self.tds_readings) / len(self.tds_readings)
            min_tds = min(self.tds_readings)
            max_tds = max(self.tds_readings)
            
            elapsed = time.time() - self.measurement_start_time
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('MEASUREMENT RUN COMPLETE')
            self.get_logger().info(f'Duration: {elapsed:.1f}s ({elapsed/60:.1f} min)')
            self.get_logger().info(f'Total samples: {len(self.tds_readings)}')
            self.get_logger().info(f'Average TDS: {avg_tds:.2f} ppm')
            self.get_logger().info(f'Min TDS: {min_tds:.2f} ppm')
            self.get_logger().info(f'Max TDS: {max_tds:.2f} ppm')
            self.get_logger().info('Data saved to microSD card')
            self.get_logger().info('=' * 60)
        else:
            self.get_logger().warn('No TDS readings recorded!')


def main(args=None):
    rclpy.init(args=args)
    
    boat_driver = BoatMeasurementDriver()
    
    # Create a separate thread for user input
    def wait_for_start():
        input("Press ENTER to start measurement run...")
        boat_driver.start_measurement()
    
    input_thread = threading.Thread(target=wait_for_start)
    input_thread.daemon = True
    input_thread.start()
    
    try:
        rclpy.spin(boat_driver)
    except KeyboardInterrupt:
        boat_driver.get_logger().info('Keyboard interrupt - stopping measurement')
    finally:
        boat_driver.stop_measurement()
        boat_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()