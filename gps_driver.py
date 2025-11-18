import serial
import utm
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from customgps.msg import Customgps

class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        
        # Create publisher for custom GPS message
        self.publisher = self.create_publisher(Customgps, '/gps_data', 10)
        
        # Get serial port parameter (with default)
        self.declare_parameter('port', '/dev/ttyACM0')
        serial_port_addr = self.get_parameter('port').get_parameter_value().string_value
        
        # Setup serial port
        self.serial_port = serial.Serial(serial_port_addr, 4800, timeout=0.1)
        self.get_logger().info(f'GPS Driver started on port {serial_port_addr}')
        
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
                altitude = float(gpgga_split[9]) if gpgga_split[9] else 0.0
                hdop = float(gpgga_split[8]) if gpgga_split[8] else 0.0
                
                # Apply your conversions
                latitude_signed = self.degMinstoDegDec(latitude)
                longitude_signed = self.degMinstoDegDec(longitude)
                latitude_signed = self.LatLongSignConvetion(latitude_signed, latitude_dir)
                longitude_signed = self.LatLongSignConvetion(longitude_signed, longitude_dir)
                
                # Convert to UTM
                utm_vals = self.convertToUTM(latitude_signed, longitude_signed)
                
                # Convert time
                time_vals = self.UTCtoUTCEpoch(utc)
                
                # Create and populate the custom message
                gps_msg = Customgps()
                gps_msg.header.frame_id = 'GPS1_Frame'
                gps_msg.header.stamp.sec = time_vals[0]
                gps_msg.header.stamp.nanosec = time_vals[1]
                gps_msg.latitude = latitude_signed
                gps_msg.longitude = longitude_signed
                gps_msg.altitude = altitude
                gps_msg.utm_easting = utm_vals[0]
                gps_msg.utm_northing = utm_vals[1]
                gps_msg.zone = utm_vals[2]
                gps_msg.letter = utm_vals[3]
                gps_msg.hdop = hdop
                gps_msg.gpgga_read = gpgga_read
                
                # Publish the message
                self.publisher.publish(gps_msg)
                self.get_logger().info(f'Published GPS data: Lat={latitude_signed:.6f}, Lon={longitude_signed:.6f}')
                
            except Exception as e:
                self.get_logger().error(f'Error processing GPGGA string: {e}')

    # Your existing parsing functions (now as class methods)
    def isGPGGAinString(self, inputString):
        if '$GPGGA' in inputString:
            return True
        else:
            return False

    def degMinstoDegDec(self, LatOrLong):
        deg = LatOrLong // 100
        mins = LatOrLong % 100
        degDec = mins / 60
        return deg + degDec

    def LatLongSignConvetion(self, LatOrLong, LatOrLongDir):
        if LatOrLongDir == 'S' or LatOrLongDir == 'W':
            LatOrLong *= -1
        return LatOrLong

    def convertToUTM(self, LatitudeSigned, LongitudeSigned):
        UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
        UTMEasting = UTMVals[0]
        UTMNorthing = UTMVals[1]
        UTMZone = UTMVals[2]
        UTMLetter = UTMVals[3]
        return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

    def UTCtoUTCEpoch(self, UTC):
        # Convert UTC to seconds
        hours = UTC // 10000
        minutes = (UTC // 100) % 100
        seconds = UTC % 100
        UTCinSecs = hours * 3600 + minutes * 60 + seconds
        
        # Get current epoch time and beginning of day
        TimeSinceEpoch = time.time()
        TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400)
        
        # Calculate final time
        CurrentTime = TimeSinceEpochBOD + UTCinSecs
        CurrentTimeSec = int(CurrentTime)
        CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1000000000)
        
        return [CurrentTimeSec, CurrentTimeNsec]

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