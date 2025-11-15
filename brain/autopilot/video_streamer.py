"""
Video Streamer Module - Manages camera capture and MJPEG streaming.

Following SRP: This module only handles video capture and streaming.
"""

import cv2
import threading
import sys
import os

# Import camera_config from parent directory
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)
from camera_config import choose_camera_by_OS


class VideoStreamer:
    """Manages camera capture and provides frames for streaming."""
    
    def __init__(self):
        """Initialize the video streamer."""
        self.camera = None
        self.camera_path = None
        self.lock = threading.Lock()
        self.is_running = False
        self.current_frame = None
        self.frame_ready = threading.Event()
    
    def initialize(self) -> bool:
        """
        Initialize camera capture.
        
        Returns:
            True if camera initialized successfully, False otherwise
        """
        try:
            self.camera_path = choose_camera_by_OS()
            self.camera = cv2.VideoCapture(self.camera_path)
            
            if not self.camera.isOpened():
                print(f"[VideoStreamer] Error: Could not open camera at {self.camera_path}")
                return False
            
            # Set camera properties for better performance
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            self.is_running = True
            print(f"[VideoStreamer] Camera initialized at {self.camera_path}")
            # Start capture thread
            self.start_capture()
            return True
            
        except Exception as e:
            print(f"[VideoStreamer] Error initializing camera: {e}")
            return False
    
    def start_capture(self):
        """Start capturing frames in a background thread."""
        if not self.is_running or self.camera is None:
            return
        
        def capture_loop():
            while self.is_running:
                try:
                    ret, frame = self.camera.read()
                    if ret:
                        with self.lock:
                            self.current_frame = frame.copy()
                            self.frame_ready.set()
                    else:
                        print("[VideoStreamer] Warning: Failed to read frame")
                except Exception as e:
                    print(f"[VideoStreamer] Error capturing frame: {e}")
        
        thread = threading.Thread(target=capture_loop, daemon=True)
        thread.start()
    
    def get_frame(self):
        """
        Get the current frame.
        
        Returns:
            Current frame (numpy array) or None if no frame available
        """
        with self.lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def generate_mjpeg(self):
        """
        Generate MJPEG stream frames.
        
        Yields:
            JPEG-encoded frames as bytes
        """
        import io
        
        while self.is_running:
            frame = self.get_frame()
            if frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            else:
                # Wait a bit if no frame available
                import time
                time.sleep(0.033)  # ~30 FPS
    
    def stop(self):
        """Stop camera capture and release resources."""
        self.is_running = False
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        print("[VideoStreamer] Camera stopped")

