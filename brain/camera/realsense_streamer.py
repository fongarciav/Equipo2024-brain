"""
RealSense Streamer Module - Manages Intel RealSense camera capture (RGB + Depth).

Following SRP: This module handles RealSense capture, alignment, and data retrieval.
"""

import cv2
import threading
import numpy as np
import time

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

class RealSenseStreamer:
    """Manages Intel RealSense camera capture and provides aligned RGB-D frames."""
    
    def __init__(self):
        """Initialize the RealSense streamer."""
        self.pipeline = None
        self.config = None
        self.align = None
        self.profile = None
        
        self.lock = threading.Lock()
        self.is_running = False
        
        self.current_color_frame = None
        self.current_depth_frame = None  # Aligned depth frame
        self.current_frame_jpeg = None   # For MJPEG streaming
        self.frame_ready = threading.Event()
        
        # Intrinsic parameters (updated on start)
        self.depth_scale = 0.001  # Default 1mm
        
    def initialize(self) -> bool:
        """
        Initialize RealSense pipeline.
        
        Returns:
            True if initialized successfully, False otherwise
        """
        if rs is None:
            print("[RealSenseStreamer] Error: pyrealsense2 not installed.")
            return False
            
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable streams
            # Adjust resolution/FPS as needed
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            self.profile = self.pipeline.start(self.config)
            
            # Get depth scale
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"[RealSenseStreamer] Depth Scale: {self.depth_scale}")
            
            # Create alignment object (align depth to color)
            self.align = rs.align(rs.stream.color)
            
            self.is_running = True
            print("[RealSenseStreamer] RealSense initialized successfully")
            
            # Start capture thread
            self.start_capture()
            return True
            
        except Exception as e:
            print(f"[RealSenseStreamer] Error initializing RealSense: {e}")
            return False
    
    def start_capture(self):
        """Start capturing frames in a background thread."""
        if not self.is_running or self.pipeline is None:
            return
        
        def capture_loop():
            while self.is_running:
                try:
                    # Wait for a coherent pair of frames: depth and color
                    frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                    
                    # Align the depth frame to color frame
                    aligned_frames = self.align.process(frames)
                    
                    aligned_depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()
                    
                    if not aligned_depth_frame or not color_frame:
                        continue
                        
                    # Convert to numpy arrays
                    depth_image = np.asanyarray(aligned_depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # Encode for MJPEG
                    ret_jpeg, buffer_jpeg = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    jpeg_bytes = buffer_jpeg.tobytes() if ret_jpeg else None
                    
                    with self.lock:
                        self.current_color_frame = color_image
                        self.current_depth_frame = depth_image
                        self.current_frame_jpeg = jpeg_bytes
                        self.frame_ready.set()
                        
                except Exception as e:
                    print(f"[RealSenseStreamer] Error capturing frame: {e}")
                    time.sleep(0.1)
        
        thread = threading.Thread(target=capture_loop, daemon=True)
        thread.start()
    
    def get_frame(self, timeout=0.05):
        """
        Get the current color frame (compatible with VideoStreamer).
        """
        if self.lock.acquire(timeout=timeout):
            try:
                if self.current_color_frame is not None:
                    return self.current_color_frame.copy()
            finally:
                self.lock.release()
        return None
    
    def get_distance(self, x: int, y: int) -> float:
        """
        Get distance in meters at pixel coordinates (x, y).
        """
        if self.lock.acquire(timeout=0.05):
            try:
                if self.current_depth_frame is not None:
                    height, width = self.current_depth_frame.shape
                    if 0 <= x < width and 0 <= y < height:
                        # Get raw depth value (uint16)
                        depth_value = self.current_depth_frame[y, x]
                        # Convert to meters
                        distance = depth_value * self.depth_scale
                        # print(f"[RealSense] Distance at ({x}, {y}): {distance:.3f}m (Raw: {depth_value})")
                        return distance
                    else:
                        print(f"[RealSense] Coordinates ({x}, {y}) out of bounds ({width}x{height})")
                else:
                    print("[RealSense] No depth frame available")
            except Exception as e:
                print(f"[RealSenseStreamer] Error getting distance: {e}")
            finally:
                self.lock.release()
        return None
        
    def generate_mjpeg(self):
        """Generate MJPEG stream frames."""
        # Reuse logic similar to VideoStreamer or simplified
        import time
        last_frame_time = 0
        frame_interval = 0.033
        
        while self.is_running:
            current_time = time.time()
            if current_time - last_frame_time >= frame_interval:
                jpeg_bytes = None
                if self.lock.acquire(timeout=0.01):
                    try:
                        jpeg_bytes = self.current_frame_jpeg
                    finally:
                        self.lock.release()
                
                if jpeg_bytes:
                    try:
                        last_frame_time = current_time
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
                    except:
                        pass
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.005)

    def stop(self):
        """Stop streaming."""
        self.is_running = False
        if self.pipeline:
            self.pipeline.stop()
            print("[RealSenseStreamer] Pipeline stopped")

