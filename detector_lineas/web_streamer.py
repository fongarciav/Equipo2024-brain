#!/usr/bin/env python3
"""
Web Streamer for Lane Detector

Provides HTTP streaming of lane detection video feeds using MJPEG streaming.
Allows remote access to the lane detector windows via web browser.
"""

import threading
import time
import cv2
import numpy as np
from typing import Optional, Callable
from flask import Flask, Response, render_template_string
import io


class WebStreamer:
    """
    Web streamer that provides HTTP endpoints for MJPEG video streaming.
    
    This class handles the composition of video streaming with Flask web server,
    following the Single Responsibility Principle.
    """
    
    def __init__(self, port: int = 5000, host: str = '0.0.0.0'):
        """
        Initialize the web streamer.
        
        Args:
            port: Port number for the web server (default: 5000)
            host: Host address to bind to (default: '0.0.0.0' for all interfaces)
        """
        self.port = port
        self.host = host
        self.app = Flask(__name__)
        self.latest_frame = None
        self.latest_canny = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        
        # Setup routes
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup Flask routes for web streaming."""
        
        # HTML template for the web interface
        html_template = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Lane Detector - Live Stream</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    background-color: #1a1a1a;
                    color: #ffffff;
                    margin: 0;
                    padding: 20px;
                }
                .container {
                    max-width: 1400px;
                    margin: 0 auto;
                }
                h1 {
                    text-align: center;
                    color: #4CAF50;
                }
                .video-container {
                    display: flex;
                    gap: 20px;
                    flex-wrap: wrap;
                    justify-content: center;
                }
                .video-box {
                    background-color: #2a2a2a;
                    padding: 10px;
                    border-radius: 8px;
                    box-shadow: 0 4px 6px rgba(0,0,0,0.3);
                }
                .video-box h2 {
                    margin-top: 0;
                    color: #4CAF50;
                }
                img {
                    max-width: 100%;
                    height: auto;
                    border: 2px solid #4CAF50;
                    border-radius: 4px;
                }
                .info {
                    text-align: center;
                    margin-top: 20px;
                    padding: 15px;
                    background-color: #2a2a2a;
                    border-radius: 8px;
                }
                .status {
                    display: inline-block;
                    padding: 5px 15px;
                    background-color: #4CAF50;
                    border-radius: 4px;
                    margin: 5px;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🚗 Lane Detector - Live Stream</h1>
                <div class="video-container">
                    <div class="video-box">
                        <h2>Lane Detection</h2>
                        <img src="{{ url_for('video_feed') }}" alt="Lane Detection">
                    </div>
                    <div class="video-box">
                        <h2>Canny Edge Detection</h2>
                        <img src="{{ url_for('canny_feed') }}" alt="Canny Detection">
                    </div>
                </div>
                <div class="info">
                    <div class="status">Streaming Active</div>
                    <p>Access this page from any device on your network</p>
                    <p>To access from internet, configure port forwarding on your router</p>
                </div>
            </div>
        </body>
        </html>
        """
        
        @self.app.route('/')
        def index():
            """Main page with video streams."""
            return render_template_string(html_template)
        
        @self.app.route('/video_feed')
        def video_feed():
            """MJPEG stream for lane detection video."""
            return Response(
                self._generate_frames('lane'),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/canny_feed')
        def canny_feed():
            """MJPEG stream for Canny edge detection video."""
            return Response(
                self._generate_frames('canny'),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/health')
        def health():
            """Health check endpoint."""
            return {'status': 'ok', 'streaming': self.running}
    
    def _generate_frames(self, stream_type: str):
        """
        Generator function for MJPEG streaming.
        
        Args:
            stream_type: 'lane' or 'canny'
        
        Yields:
            JPEG-encoded frames
        """
        while self.running:
            with self.frame_lock:
                if stream_type == 'lane' and self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                elif stream_type == 'canny' and self.latest_canny is not None:
                    frame = self.latest_canny.copy()
                else:
                    # Create a placeholder frame if no frame is available
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, 'Waiting for frames...', (150, 240),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            # Small delay to control frame rate (approximately 30 FPS)
            time.sleep(1/30.0)
    
    def update_frame(self, lane_frame: np.ndarray, canny_frame: Optional[np.ndarray] = None):
        """
        Update the latest frames for streaming.
        
        Args:
            lane_frame: The lane detection frame (BGR format)
            canny_frame: Optional Canny edge detection frame (grayscale or BGR)
        """
        with self.frame_lock:
            self.latest_frame = lane_frame.copy()
            if canny_frame is not None:
                # Convert grayscale to BGR if needed
                if len(canny_frame.shape) == 2:
                    self.latest_canny = cv2.cvtColor(canny_frame, cv2.COLOR_GRAY2BGR)
                else:
                    self.latest_canny = canny_frame.copy()
    
    def start(self):
        """Start the web server in a separate thread."""
        if self.running:
            return
        
        self.running = True
        
        def run_server():
            self.app.run(host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False)
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        
        print(f"🌐 Web streamer started!")
        print(f"   Access at: http://{self._get_local_ip()}:{self.port}")
        print(f"   Or from network: http://<raspberry-pi-ip>:{self.port}")
        print(f"   Press Ctrl+C to stop")
    
    def stop(self):
        """Stop the web server."""
        self.running = False
        # Flask doesn't have a clean shutdown, but the thread will exit when app stops
    
    def _get_local_ip(self) -> str:
        """Get local IP address."""
        import socket
        try:
            # Connect to a remote address to determine local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"

