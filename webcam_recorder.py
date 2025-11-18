import cv2
import threading
import time
from datetime import datetime
import os

class WebcamRecorder:
    def __init__(self):
        self.is_recording = False
        self.video_writer = None
        self.capture = None
        self.record_thread = None
        
    def start_recording(self, duration_seconds=300, output_dir="recordings", 
                       fps=20.0, resolution=(640, 480)):
        """
        Start webcam recording for specified duration.
        
        Args:
            duration_seconds: Recording duration (default: 300 = 5 minutes)
            output_dir: Directory to save recordings
            fps: Frames per second for recording
            resolution: Video resolution as (width, height)
        """
        if self.is_recording:
            print("Already recording!")
            return False
            
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate filename with timestamp (MP4 format)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(output_dir, f"recording_{timestamp}.mp4")
        
        # Start recording in separate thread
        self.record_thread = threading.Thread(
            target=self._record,
            args=(output_file, duration_seconds, fps, resolution)
        )
        self.record_thread.start()
        
        print(f"Started recording to: {output_file}")
        print(f"Will record for {duration_seconds} seconds ({duration_seconds/60:.1f} minutes)")
        return True
    
    def _record(self, output_file, duration, fps, resolution):
        """Internal method to handle the actual recording."""
        try:
            # Initialize webcam
            self.capture = cv2.VideoCapture(0)
            
            if not self.capture.isOpened():
                print("Error: Could not open webcam")
                return
            
            # Set resolution
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
            
            # Try different codecs for MP4 (in order of preference)
            codecs = ['mp4v', 'MP4V', 'avc1', 'H264']
            self.video_writer = None
            
            for codec in codecs:
                try:
                    fourcc = cv2.VideoWriter_fourcc(*codec)
                    temp_writer = cv2.VideoWriter(
                        output_file, fourcc, fps, resolution
                    )
                    if temp_writer.isOpened():
                        self.video_writer = temp_writer
                        print(f"Using codec: {codec}")
                        break
                    else:
                        temp_writer.release()
                except:
                    continue
            
            if self.video_writer is None:
                print("Error: Could not initialize video writer with any codec")
                return
            
            self.is_recording = True
            start_time = time.time()
            
            # Record until duration is reached
            while self.is_recording and (time.time() - start_time) < duration:
                ret, frame = self.capture.read()
                
                if ret:
                    # Resize frame to match output resolution
                    frame = cv2.resize(frame, resolution)
                    self.video_writer.write(frame)
                else:
                    print("Error: Failed to capture frame")
                    break
                
                # Small delay to control frame rate
                time.sleep(0.01)
            
            # Cleanup
            self.stop_recording()
            print(f"Recording complete: {output_file}")
            
        except Exception as e:
            print(f"Error during recording: {e}")
            self.stop_recording()
    
    def stop_recording(self):
        """Stop the current recording."""
        self.is_recording = False
        
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            
        if self.capture:
            self.capture.release()
            self.capture = None
    
    def wait_for_completion(self):
        """Wait for recording thread to finish."""
        if self.record_thread and self.record_thread.is_alive():
            self.record_thread.join()


# Global recorder instance
recorder = WebcamRecorder()


def auto_record_webcam(duration_minutes=5):
    """
    Convenience function to start webcam recording.
    This can be called from your other driver code.
    
    Args:
        duration_minutes: How long to record (default: 5 minutes)
    """
    duration_seconds = duration_minutes * 60
    return recorder.start_recording(duration_seconds=duration_seconds)


# Example usage
if __name__ == "__main__":
    # Quick test: 10 second recording
    print("Starting 10-second test recording...")
    auto_record_webcam(duration_minutes=0.167)  # 10 seconds
    
    # The recording runs in background
    print("Recording in progress... (10 seconds)")
    
    # Wait for it to complete
    time.sleep(12)
    print("Test complete! Check the recordings folder.")
