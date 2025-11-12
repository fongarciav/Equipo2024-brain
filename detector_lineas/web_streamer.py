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
from flask import Flask, Response, render_template_string, request, jsonify
import io
import json


class WebStreamer:
    """
    Web streamer that provides HTTP endpoints for MJPEG video streaming.
    
    This class handles the composition of video streaming with Flask web server,
    following the Single Responsibility Principle.
    """
    
    def __init__(self, port: int = 5000, host: str = '0.0.0.0', detector=None):
        """
        Initialize the web streamer.
        
        Args:
            port: Port number for the web server (default: 5000)
            host: Host address to bind to (default: '0.0.0.0' for all interfaces)
            detector: Optional reference to MarcosLaneDetector for PID control
        """
        self.port = port
        self.host = host
        self.app = Flask(__name__)
        self.latest_frame = None
        self.latest_canny = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        self.detector = detector  # Reference to lane detector for PID control
        
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
                .controls {
                    background-color: #2a2a2a;
                    padding: 20px;
                    border-radius: 8px;
                    margin-top: 20px;
                }
                .control-group {
                    margin-bottom: 20px;
                }
                .control-group label {
                    display: block;
                    margin-bottom: 8px;
                    color: #4CAF50;
                    font-weight: bold;
                }
                .slider-container {
                    display: flex;
                    align-items: center;
                    gap: 15px;
                }
                .slider {
                    flex: 1;
                    height: 8px;
                    border-radius: 5px;
                    background: #444;
                    outline: none;
                    -webkit-appearance: none;
                }
                .slider::-webkit-slider-thumb {
                    -webkit-appearance: none;
                    appearance: none;
                    width: 20px;
                    height: 20px;
                    border-radius: 50%;
                    background: #4CAF50;
                    cursor: pointer;
                }
                .slider::-moz-range-thumb {
                    width: 20px;
                    height: 20px;
                    border-radius: 50%;
                    background: #4CAF50;
                    cursor: pointer;
                    border: none;
                }
                .value-display {
                    min-width: 80px;
                    text-align: right;
                    color: #fff;
                    font-family: monospace;
                }
                button {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 10px 20px;
                    border-radius: 4px;
                    cursor: pointer;
                    font-size: 14px;
                    margin-top: 10px;
                }
                button:hover {
                    background-color: #45a049;
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
                <div class="controls" id="pidControls">
                    <h2 style="color: #4CAF50; margin-top: 0;">PID Parameters (Real-time)</h2>
                    <div class="control-group">
                        <label for="kp">Kp (Proportional): <span class="value-display" id="kpValue">0.02</span></label>
                        <div class="slider-container">
                            <input type="range" id="kp" class="slider" min="0" max="0.1" step="0.001" value="0.02">
                        </div>
                    </div>
                    <div class="control-group">
                        <label for="ki">Ki (Integral): <span class="value-display" id="kiValue">0.001</span></label>
                        <div class="slider-container">
                            <input type="range" id="ki" class="slider" min="0" max="0.1" step="0.0001" value="0.001">
                        </div>
                    </div>
                    <div class="control-group">
                        <label for="kd">Kd (Derivative): <span class="value-display" id="kdValue">0.001</span></label>
                        <div class="slider-container">
                            <input type="range" id="kd" class="slider" min="0" max="0.1" step="0.0001" value="0.001">
                        </div>
                    </div>
                    <div class="control-group">
                        <label for="tolerance">Tolerance: <span class="value-display" id="toleranceValue">60</span></label>
                        <div class="slider-container">
                            <input type="range" id="tolerance" class="slider" min="0" max="200" step="1" value="60">
                        </div>
                    </div>
                    <button onclick="resetPID()">Reset to Default</button>
                </div>
                <div class="info">
                    <div class="status">Streaming Active</div>
                    <p>Access this page from any device on your network</p>
                    <p>Adjust PID parameters in real-time using the sliders above</p>
                </div>
            </div>
            <script>
                // Update PID parameters when sliders change
                document.getElementById('kp').addEventListener('input', function(e) {
                    document.getElementById('kpValue').textContent = parseFloat(e.target.value).toFixed(3);
                    updatePID();
                });
                document.getElementById('ki').addEventListener('input', function(e) {
                    document.getElementById('kiValue').textContent = parseFloat(e.target.value).toFixed(4);
                    updatePID();
                });
                document.getElementById('kd').addEventListener('input', function(e) {
                    document.getElementById('kdValue').textContent = parseFloat(e.target.value).toFixed(4);
                    updatePID();
                });
                document.getElementById('tolerance').addEventListener('input', function(e) {
                    document.getElementById('toleranceValue').textContent = e.target.value;
                    updatePID();
                });
                
                let updateTimeout;
                function updatePID() {
                    clearTimeout(updateTimeout);
                    updateTimeout = setTimeout(() => {
                        const params = {
                            kp: parseFloat(document.getElementById('kp').value),
                            ki: parseFloat(document.getElementById('ki').value),
                            kd: parseFloat(document.getElementById('kd').value),
                            tolerance: parseInt(document.getElementById('tolerance').value)
                        };
                        
                        fetch('/api/pid', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify(params)
                        }).then(response => response.json())
                          .then(data => {
                              if (data.status === 'ok') {
                                  console.log('PID updated:', params);
                              }
                          })
                          .catch(error => console.error('Error:', error));
                    }, 100); // Debounce: wait 100ms after last change
                }
                
                function resetPID() {
                    document.getElementById('kp').value = 0.02;
                    document.getElementById('ki').value = 0.001;
                    document.getElementById('kd').value = 0.001;
                    document.getElementById('tolerance').value = 60;
                    document.getElementById('kpValue').textContent = '0.02';
                    document.getElementById('kiValue').textContent = '0.001';
                    document.getElementById('kdValue').textContent = '0.001';
                    document.getElementById('toleranceValue').textContent = '60';
                    updatePID();
                }
                
                // Load current PID values on page load
                fetch('/api/pid')
                    .then(response => response.json())
                    .then(data => {
                        if (data.status === 'ok') {
                            document.getElementById('kp').value = data.kp;
                            document.getElementById('ki').value = data.ki;
                            document.getElementById('kd').value = data.kd;
                            document.getElementById('tolerance').value = data.tolerance;
                            document.getElementById('kpValue').textContent = parseFloat(data.kp).toFixed(3);
                            document.getElementById('kiValue').textContent = parseFloat(data.ki).toFixed(4);
                            document.getElementById('kdValue').textContent = parseFloat(data.kd).toFixed(4);
                            document.getElementById('toleranceValue').textContent = data.tolerance;
                        }
                    })
                    .catch(error => console.error('Error loading PID:', error));
            </script>
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
        
        @self.app.route('/api/pid', methods=['GET'])
        def get_pid():
            """Get current PID parameters."""
            if self.detector:
                params = self.detector.get_pid_parameters()
                return jsonify({'status': 'ok', **params})
            return jsonify({'status': 'error', 'message': 'Detector not available'}), 500
        
        @self.app.route('/api/pid', methods=['POST'])
        def set_pid():
            """Set PID parameters."""
            if not self.detector:
                return jsonify({'status': 'error', 'message': 'Detector not available'}), 500
            
            try:
                data = request.get_json()
                kp = data.get('kp')
                ki = data.get('ki')
                kd = data.get('kd')
                tolerance = data.get('tolerance')
                
                self.detector.update_pid_parameters(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    tolerance=tolerance
                )
                
                return jsonify({'status': 'ok', 'message': 'PID parameters updated'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
    
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
    
    def is_running(self) -> bool:
        """Check if the web server is running."""
        return self.running
    
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

