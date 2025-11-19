
#!/usr/bin/env python3
"""
Flask server to serve the ESP32 Car Control Dashboard.
This allows accessing the dashboard via IP address from any device on the network.
"""

from flask import Flask, send_from_directory, request, jsonify, Response, stream_with_context
from flask_cors import CORS
import os
import sys
import threading
import time
import queue

# Import auto-pilot modules
import sys
import os
# Add parent directory to path to import autopilot modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)
try:
    from autopilot.video_streamer import VideoStreamer
    from autopilot.autopilot_controller import AutoPilotController
    from autopilot.command_sender import CommandSender
except ImportError as e:
    print(f"Warning: Could not import autopilot modules: {e}")
    print("Auto-pilot features will be disabled.")
    VideoStreamer = None
    AutoPilotController = None
    CommandSender = None

# Try to import serial
try:
    import serial
    from serial.tools import list_ports as serial_list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Install with: pip install pyserial", file=sys.stderr)

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# UART Configuration
UART_BAUD_RATE = 115200

# Global serial connection
serial_conn = None
serial_lock = threading.Lock()
serial_message_queue = queue.Queue()
serial_reader_thread = None
serial_reader_running = False
serial_read_buffer = ""  # Buffer for incomplete serial lines

# SSE client connections - list of queues, one per connected client
sse_clients = []
sse_clients_lock = threading.Lock()

# System state tracking
system_state = {'mode': 'MANUAL', 'state': 'ARMED'}
system_state_lock = threading.Lock()

# Auto-pilot components
video_streamer = None
autopilot_controller = None
command_sender = None

app = Flask(__name__, static_folder=SCRIPT_DIR)
CORS(app)  # Enable CORS for all routes


def write_uart_command(command: str):
    """Write a UART command to ESP32."""
    global serial_conn
    if not serial_conn or not serial_conn.is_open:
        return False, "Serial port not connected"

    try:
        with serial_lock:
            line = (command.strip() + "\n").encode("utf-8")
            serial_conn.write(line)
            serial_conn.flush()
            return True, "OK"
    except Exception as e:
        return False, str(e)


def translate_http_to_uart(endpoint: str, args: dict):
    """Translate HTTP endpoint to UART command."""
    SERVO_CENTER = 105
    SERVO_RIGHT = 50   # Right turn (lower value)
    SERVO_LEFT = 160   # Left turn (higher value)
    MOTOR_SPEED_MAX = 255
    FORWARD_SPEED = 210

    if endpoint == 'arm':
        return True, "ARM", "M:SYS_ARM:0"
    elif endpoint == 'disarm':
        return True, "DISARM", "M:SYS_DISARM:0"
    elif endpoint == 'mode':
        value = args.get('value', '').upper()
        if value == 'AUTO':
            return True, "MODE: AUTO", "M:SYS_MODE:1"
        elif value == 'MANUAL':
            return True, "MODE: MANUAL", "M:SYS_MODE:0"
        else:
            return False, "Invalid mode value", None
    elif endpoint == 'brake':
        return True, "BRAKE", "E:BRAKE_NOW:0"
    elif endpoint == 'forward':
        return True, "FORWARD", f"C:SET_SPEED:{FORWARD_SPEED}"
    elif endpoint == 'back':
        return True, "BACKWARD", f"C:SET_SPEED:{MOTOR_SPEED_MAX}"
    elif endpoint == 'driveStop':
        return True, "STOP", "C:SET_SPEED:0"
    elif endpoint == 'changeSpeed':
        speed_str = args.get('speed', '0')
        direction_str = args.get('direction', 'forward')
        try:
            speed = int(speed_str)
            if speed < 0 or speed > MOTOR_SPEED_MAX:
                return False, f"Speed must be 0-{MOTOR_SPEED_MAX}", None
            if speed == 0:
                return True, "STOP", "C:SET_SPEED:0"
            else:
                return True, f"SPEED: {speed} ({direction_str})", f"C:SET_SPEED:{speed}"
        except ValueError:
            return False, "Invalid speed value", None
    elif endpoint == 'steer' or endpoint == 'changeSteer':
        angle_str = args.get('angle', '105')
        try:
            angle = int(angle_str)
            # SERVO_RIGHT (50) is minimum, SERVO_LEFT (135) is maximum
            if angle < SERVO_RIGHT or angle > SERVO_LEFT:
                return False, f"Angle must be {SERVO_RIGHT}-{SERVO_LEFT}", None
            return True, f"STEER: {angle}°", f"C:SET_STEER:{angle}"
        except ValueError:
            return False, "Invalid angle value", None
    elif endpoint == 'left':
        return True, "LEFT", f"C:SET_STEER:{SERVO_LEFT}"
    elif endpoint == 'right':
        return True, "RIGHT", f"C:SET_STEER:{SERVO_RIGHT}"
    elif endpoint == 'steerStop':
        return True, "CENTER", f"C:SET_STEER:{SERVO_CENTER}"
    elif endpoint == 'LightsOn':
        return True, "LIGHTS ON", "M:LIGHTS_ON:0"
    elif endpoint == 'LightsOff':
        return True, "LIGHTS OFF", "M:LIGHTS_OFF:0"
    elif endpoint == 'LightsAuto':
        return True, "LIGHTS AUTO", "M:LIGHTS_AUTO:0"
    else:
        return False, f"Unknown endpoint: {endpoint}", None


@app.route('/')
def index():
    """Serve the dashboard HTML file."""
    return send_from_directory(SCRIPT_DIR, 'dashboard.html')


@app.route('/<path:endpoint>')
def handle_command(endpoint):
    """Handle all dashboard commands and translate to UART."""
    args = request.args.to_dict()

    success, message, uart_cmd = translate_http_to_uart(endpoint, args)

    if not success:
        return jsonify({'error': message}), 400

    if uart_cmd is None:
        return jsonify({'status': 'ok', 'message': message}), 200

    cmd_success, cmd_message = write_uart_command(uart_cmd)

    if cmd_success:
        return jsonify({'status': 'ok', 'message': message, 'uart_command': uart_cmd}), 200
    else:
        return jsonify({'error': f'Failed to send UART command: {cmd_message}'}), 500


@app.route('/proxy/<path:endpoint>')
def proxy_request(endpoint):
    """
    Proxy requests to ESP32 to avoid CORS issues.
    Usage: /proxy/arm, /proxy/status, etc.
    """
    esp32_ip = request.args.get('ip', '192.168.4.1')
    url = f'http://{esp32_ip}/{endpoint}'

    try:
        import requests
        response = requests.get(url, timeout=2)
        return response.text, response.status_code
    except ImportError:
        return jsonify({'error': 'requests library not installed. Install with: pip install requests'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


def open_serial(port: str, baud: int = UART_BAUD_RATE):
    """Open serial port with appropriate settings."""
    if not SERIAL_AVAILABLE:
        raise RuntimeError("pyserial not installed")

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,  # Shorter timeout for more responsive reading
        write_timeout=1.0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_available_lines(ser):
    """Read all available lines from serial port, buffering incomplete lines."""
    global serial_read_buffer
    lines = []
    if not ser or not ser.is_open:
        return lines

    try:
        # Read all available data first
        available = ser.in_waiting
        if available > 0:
            # Read in chunks to avoid blocking
            data = ser.read(available)
            # Decode and append to buffer
            new_data = data.decode('utf-8', errors='ignore')
            serial_read_buffer += new_data
            
            # Process complete lines (ending with \n or \r\n)
            # Keep processing until no more complete lines in buffer
            while True:
                # Check for \r\n first (Windows line ending)
                if '\r\n' in serial_read_buffer:
                    line, serial_read_buffer = serial_read_buffer.split('\r\n', 1)
                    line = line.strip()
                    if line:
                        lines.append(line)
                # Then check for \n (Unix line ending)
                elif '\n' in serial_read_buffer:
                    line, serial_read_buffer = serial_read_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        lines.append(line)
                else:
                    # No more complete lines, break
                    break
        else:
            # If no data available, try readline with short timeout
            # This helps catch any remaining complete lines
            try:
                data = ser.readline()
                if data:
                    line = data.decode('utf-8', errors='ignore').rstrip()
                    if line:
                        lines.append(line)
            except:
                pass  # Timeout is expected when no data
    except Exception as e:
        print(f"[DEBUG] Read error: {e}", file=sys.stderr)
        pass

    return lines


def parse_system_events(line: str):
    """Parse system state and mode events from serial messages."""
    global system_state

    if line.startswith('EVENT:STATE_CHANGED:'):
        state = line.replace('EVENT:STATE_CHANGED:', '').strip()
        with system_state_lock:
            system_state['state'] = state
    elif line.startswith('EVENT:MODE_CHANGED:'):
        mode = line.replace('EVENT:MODE_CHANGED:', '').strip()
        with system_state_lock:
            system_state['mode'] = mode


def serial_reader_worker():
    """Background thread that reads from serial port."""
    global serial_conn, serial_reader_running, sse_clients

    while serial_reader_running:
        if serial_conn and serial_conn.is_open:
            try:
                lines = read_available_lines(serial_conn)
                for line in lines:
                    if line:
                        # Parse system events for state tracking
                        parse_system_events(line)

                        # Broadcast message to all connected SSE clients
                        message_data = {
                            'type': 'serial',
                            'message': line,
                            'timestamp': time.time()
                        }
                        with sse_clients_lock:
                            # Send to all connected clients
                            # Copy list to avoid modification during iteration
                            for client_queue in sse_clients[:]:
                                try:
                                    client_queue.put(message_data, block=False)
                                except queue.Full:
                                    # Client queue is full, remove it (client might be disconnected)
                                    try:
                                        sse_clients.remove(client_queue)
                                    except ValueError:
                                        pass  # Already removed
                if not lines:
                    time.sleep(0.01)
            except Exception as e:
                if serial_reader_running:
                    print(f"[SerialReader] Error: {e}", file=sys.stderr)
                    time.sleep(0.1)
        else:
            time.sleep(0.1)


def start_serial_reader():
    """Start the serial reader thread."""
    global serial_reader_thread, serial_reader_running

    if serial_reader_thread is None or not serial_reader_thread.is_alive():
        serial_reader_running = True
        serial_reader_thread = threading.Thread(
            target=serial_reader_worker, daemon=True)
        serial_reader_thread.start()


def stop_serial_reader():
    """Stop the serial reader thread."""
    global serial_reader_running, serial_reader_thread

    serial_reader_running = False
    if serial_reader_thread:
        serial_reader_thread.join(timeout=1.0)
        serial_reader_thread = None


@app.route('/uart/ports', methods=['GET'])
def list_ports():
    """List available serial ports."""
    if not SERIAL_AVAILABLE:
        return jsonify({'error': 'pyserial not installed', 'ports': []}), 500

    try:
        ports_list = list(serial_list_ports.comports())
        ports = []
        for p in ports_list:
            ports.append({
                'device': p.device,
                'description': p.description or 'No description',
                'hwid': getattr(p, 'hwid', 'Unknown')
            })
        return jsonify({'ports': ports, 'count': len(ports)}), 200
    except Exception as e:
        return jsonify({'error': str(e), 'ports': []}), 500


def initialize_autopilot_if_needed():
    """Initialize autopilot controller if conditions are met."""
    global autopilot_controller, command_sender, video_streamer, serial_conn
    
    # Only initialize if not already initialized
    if autopilot_controller is not None:
        return True
    
    # Check if we have all required components
    if CommandSender is None or AutoPilotController is None:
        print("[UART] Autopilot modules not available", file=sys.stderr)
        return False
    
    if not serial_conn or not serial_conn.is_open:
        print("[UART] Serial port not connected", file=sys.stderr)
        return False
    
    # Initialize command sender
    if command_sender is None:
        command_sender = CommandSender(write_uart_command)
        print("[UART] Command sender initialized", file=sys.stderr)
    
    # Initialize video streamer if not already done
    if video_streamer is None and VideoStreamer is not None:
        print("[UART] Initializing video streamer...", file=sys.stderr)
        video_streamer = VideoStreamer()
        if not video_streamer.initialize():
            print("[UART] Video streamer initialization failed", file=sys.stderr)
            video_streamer = None
            return False
        print("[UART] Video streamer initialized", file=sys.stderr)
    
    # Initialize autopilot controller
    if video_streamer is not None:
        print("[UART] Initializing autopilot controller...", file=sys.stderr)
        # Initialize autopilot controller with default parameters
        autopilot_controller = AutoPilotController(
            video_streamer=video_streamer,
            command_sender=command_sender,
            threshold=180,
            pid_kp=0.43,
            pid_ki=0.002,
            pid_kd=0.12,
            max_angle=30.0,
            deadband=6.0
        )
        print("[UART] Autopilot controller initialized (not started - use /autopilot/start)", file=sys.stderr)
        return True
    else:
        print("[UART] Autopilot controller not initialized - video streamer not available", file=sys.stderr)
        return False

@app.route('/uart/connect', methods=['POST'])
def uart_connect():
    """Connect to ESP32 via UART."""
    global serial_conn, serial_read_buffer

    if not SERIAL_AVAILABLE:
        return jsonify({'error': 'pyserial not installed'}), 500

    data = request.get_json() or {}
    port = data.get('port')

    if not port:
        return jsonify({'error': 'Port not specified'}), 400

    try:
        if serial_conn and serial_conn.is_open:
            stop_serial_reader()
            serial_conn.close()
            serial_read_buffer = ""  # Clear buffer on reconnect

        serial_conn = open_serial(port, UART_BAUD_RATE)
        serial_read_buffer = ""  # Clear buffer on new connection
        time.sleep(0.1)
        start_serial_reader()
        
        # Try to initialize autopilot if modules are available
        autopilot_initialized = initialize_autopilot_if_needed()
        
        response = {
            'status': 'ok',
            'message': f'Connected to {port}',
            'port': port,
            'baudrate': UART_BAUD_RATE,
            'autopilot_available': autopilot_initialized
        }
        
        if not autopilot_initialized:
            response['warning'] = 'Autopilot not initialized - check camera connection and module availability'

        return jsonify(response)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/uart/disconnect', methods=['POST'])
def uart_disconnect():
    """Disconnect from ESP32."""
    global serial_conn, serial_read_buffer, autopilot_controller

    stop_serial_reader()
    
    # Stop autopilot if running
    if autopilot_controller:
        autopilot_controller.stop()

    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        serial_conn = None
    
    serial_read_buffer = ""  # Clear buffer on disconnect

    return jsonify({'status': 'ok', 'message': 'Disconnected'})


@app.route('/uart/stream')
def uart_stream():
    """Server-Sent Events stream for serial messages."""
    def generate():
        # Create a dedicated queue for this client
        client_queue = queue.Queue(maxsize=100)

        # Register this client
        with sse_clients_lock:
            sse_clients.append(client_queue)

        try:
            while True:
                try:
                    # Get message from this client's queue
                    msg = client_queue.get(timeout=1.0)
                    message = msg.get('message', '')
                    if message and len(message.strip()) > 0:
                        data = f"data: {message}\n\n"
                        yield data
                except queue.Empty:
                    yield ": heartbeat\n\n"
        except GeneratorExit:
            pass  # Client disconnected
        except Exception as e:
            print(f"[SSE] Error: {e}", file=sys.stderr)
        finally:
            # Unregister this client
            with sse_clients_lock:
                try:
                    sse_clients.remove(client_queue)
                except ValueError:
                    pass  # Already removed

    response = Response(stream_with_context(generate()),
                        mimetype='text/event-stream')
    response.headers['Cache-Control'] = 'no-cache'
    response.headers['X-Accel-Buffering'] = 'no'  # Disable buffering for nginx
    return response


@app.route('/status')
def get_status():
    """Get current system status (mode and state)."""
    global system_state
    with system_state_lock:
        return jsonify({
            'mode': system_state['mode'],
            'state': system_state['state']
        })


# Global debug mode flag
debug_mode_enabled = False

@app.route('/video_stream')
def video_stream():
    """MJPEG video stream endpoint."""
    global video_streamer
    if video_streamer is None:
        return "Video streamer not initialized", 503
    
    try:
        return Response(
            stream_with_context(video_streamer.generate_mjpeg()),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )
    except Exception as e:
        return f"Error generating video stream: {e}", 500


def generate_debug_mjpeg(image_key):
    """Generate MJPEG stream from debug images."""
    import cv2
    import time
    import numpy as np
    
    global autopilot_controller, video_streamer
    
    # Create a placeholder black image
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, 'Waiting for auto-pilot...', (50, 240), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    while True:
        if autopilot_controller is None:
            # Send placeholder if autopilot not initialized
            ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.1)
            continue
        
        # Check if autopilot is running
        status = autopilot_controller.get_status()
        if not status.get('is_running', False):
            # Send placeholder if autopilot not running
            placeholder_text = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder_text, 'Auto-pilot not running', (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', placeholder_text, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.1)
            continue
        
        debug_image = autopilot_controller.get_debug_image(image_key)
        if debug_image is not None:
            # Make sure image is valid (not empty)
            try:
                if debug_image.size > 0 and len(debug_image.shape) >= 2:
                    # Convert grayscale ROI to BGR if needed for display
                    if len(debug_image.shape) == 2:  # Grayscale image (like 'roi' or 'binary')
                        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
                    
                    ret, buffer = cv2.imencode('.jpg', debug_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    else:
                        # Encoding failed, send placeholder
                        ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if ret:
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                else:
                    # Empty or invalid image, send placeholder
                    ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            except Exception as e:
                # Error processing image, send placeholder
                print(f"[Debug Stream] Error processing {image_key}: {e}")
                ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            # No debug image available, try to show regular video frame as fallback
            if video_streamer is not None:
                frame = video_streamer.get_frame()
                if frame is not None:
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                else:
                    ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            else:
                ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS


@app.route('/debug/bird_view_lines')
def debug_bird_view_lines():
    """MJPEG stream for bird view with lines."""
    return Response(
        stream_with_context(generate_debug_mjpeg('bird_view_lines')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/debug/sliding_windows')
def debug_sliding_windows():
    """MJPEG stream for sliding windows."""
    return Response(
        stream_with_context(generate_debug_mjpeg('sliding_windows')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/debug/final_result')
def debug_final_result():
    """MJPEG stream for final result."""
    return Response(
        stream_with_context(generate_debug_mjpeg('final_result')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/debug/toggle', methods=['POST'])
def toggle_debug_mode():
    """Toggle debug mode on/off."""
    global debug_mode_enabled
    debug_mode_enabled = not debug_mode_enabled
    return jsonify({
        'debug_mode': debug_mode_enabled,
        'message': 'Debug mode ' + ('enabled' if debug_mode_enabled else 'disabled')
    })


@app.route('/debug/status', methods=['GET'])
def debug_status():
    """Get current debug mode status."""
    global debug_mode_enabled
    return jsonify({'debug_mode': debug_mode_enabled})


@app.route('/autopilot/start', methods=['POST'])
def autopilot_start():
    """Start the auto-pilot controller."""
    global autopilot_controller
    
    # Try to initialize if not already initialized
    if autopilot_controller is None:
        initialized = initialize_autopilot_if_needed()
        if not initialized:
            # Get detailed status for error message
            status = {
                'error': 'Auto-pilot controller not initialized',
                'details': {
                    'modules_available': {
                        'CommandSender': CommandSender is not None,
                        'AutoPilotController': AutoPilotController is not None,
                        'VideoStreamer': VideoStreamer is not None
                    },
                    'serial_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
                    'video_streamer_initialized': video_streamer is not None
                },
                'suggestions': []
            }
            
            if not status['details']['serial_connected']:
                status['suggestions'].append('Connect UART port first')
            if not status['details']['video_streamer_initialized']:
                if not status['details']['modules_available']['VideoStreamer']:
                    status['suggestions'].append('Install autopilot modules: pip install opencv-python numpy')
                else:
                    status['suggestions'].append('Connect camera USB device')
            if not status['details']['modules_available']['AutoPilotController']:
                status['suggestions'].append('Autopilot modules not found - check that you are in brain/dashboard directory')
            
            return jsonify(status), 503
    
    success = autopilot_controller.start()
    if success:
        return jsonify({'status': 'ok', 'message': 'Auto-pilot started'})
    else:
        return jsonify({'error': 'Auto-pilot already running'}), 400


@app.route('/autopilot/stop', methods=['POST'])
def autopilot_stop():
    """Stop the auto-pilot controller."""
    global autopilot_controller
    if autopilot_controller is None:
        return jsonify({'error': 'Auto-pilot controller not initialized'}), 503
    
    success = autopilot_controller.stop()
    if success:
        return jsonify({'status': 'ok', 'message': 'Auto-pilot stopped'})
    else:
        return jsonify({'error': 'Auto-pilot not running'}), 400


@app.route('/autopilot/init-status', methods=['GET'])
def autopilot_init_status():
    """Get initialization status of autopilot components."""
    global autopilot_controller, command_sender, video_streamer, serial_conn
    
    status = {
        'autopilot_controller_initialized': autopilot_controller is not None,
        'command_sender_initialized': command_sender is not None,
        'video_streamer_initialized': video_streamer is not None,
        'serial_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
        'modules_available': {
            'CommandSender': CommandSender is not None,
            'AutoPilotController': AutoPilotController is not None,
            'VideoStreamer': VideoStreamer is not None
        }
    }
    
    # Add reasons if not initialized
    if not status['autopilot_controller_initialized']:
        reasons = []
        if not status['modules_available']['AutoPilotController']:
            reasons.append('AutopilotController module not imported')
        if not status['modules_available']['CommandSender']:
            reasons.append('CommandSender module not imported')
        if not status['serial_connected']:
            reasons.append('Serial port not connected')
        if not status['video_streamer_initialized']:
            if not status['modules_available']['VideoStreamer']:
                reasons.append('VideoStreamer module not imported')
            else:
                reasons.append('Video streamer initialization failed (check camera)')
        status['initialization_issues'] = reasons
    
    return jsonify(status)

@app.route('/autopilot/status', methods=['GET'])
def autopilot_status():
    """Get auto-pilot controller status."""
    global autopilot_controller
    if autopilot_controller is None:
        return jsonify({'error': 'Auto-pilot controller not initialized'}), 503
    
    status = autopilot_controller.get_status()
    pid_params = autopilot_controller.get_pid_parameters()
    status['pid_parameters'] = pid_params
    return jsonify(status)

@app.route('/autopilot/pid', methods=['POST'])
def autopilot_update_pid():
    """Update PID parameters."""
    global autopilot_controller
    if autopilot_controller is None:
        return jsonify({'error': 'Auto-pilot controller not initialized'}), 503
    
    data = request.get_json() or {}
    kp = data.get('kp')
    ki = data.get('ki')
    kd = data.get('kd')
    max_angle = data.get('max_angle')
    deadband = data.get('deadband')
    
    # Validate parameters
    if kp is not None and (not isinstance(kp, (int, float)) or kp < 0):
        return jsonify({'error': 'Invalid kp value'}), 400
    if ki is not None and (not isinstance(ki, (int, float)) or ki < 0):
        return jsonify({'error': 'Invalid ki value'}), 400
    if kd is not None and (not isinstance(kd, (int, float)) or kd < 0):
        return jsonify({'error': 'Invalid kd value'}), 400
    if max_angle is not None and (not isinstance(max_angle, (int, float)) or max_angle < 0):
        return jsonify({'error': 'Invalid max_angle value'}), 400
    if deadband is not None and (not isinstance(deadband, (int, float)) or deadband < 0):
        return jsonify({'error': 'Invalid deadband value'}), 400
    
    # Update parameters
    autopilot_controller.update_pid_parameters(kp=kp, ki=ki, kd=kd, max_angle=max_angle, deadband=deadband)
    
    # Return updated parameters
    updated_params = autopilot_controller.get_pid_parameters()
    return jsonify({
        'status': 'ok',
        'message': 'PID parameters updated',
        'pid_parameters': updated_params
    })

@app.route('/autopilot/pid', methods=['GET'])
def autopilot_get_pid():
    """Get current PID parameters."""
    global autopilot_controller
    if autopilot_controller is None:
        return jsonify({'error': 'Auto-pilot controller not initialized'}), 503
    
    pid_params = autopilot_controller.get_pid_parameters()
    return jsonify(pid_params)


@app.route('/health')
def health():
    """Health check endpoint."""
    global serial_conn, serial_reader_running, video_streamer, autopilot_controller
    port_name = None
    if serial_conn and serial_conn.is_open:
        port_name = serial_conn.port
    
    autopilot_status_info = None
    if autopilot_controller is not None:
        autopilot_status_info = autopilot_controller.get_status()
    
    return jsonify({
        'status': 'ok',
        'message': 'Dashboard server is running',
        'uart_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
        'uart_port': port_name,
        'serial_available': SERIAL_AVAILABLE,
        'serial_reader_running': serial_reader_running,
        'video_streamer_initialized': video_streamer is not None,
        'autopilot_status': autopilot_status_info
    })


def select_port_interactive() -> str:
    """Interactive port selection from available serial ports."""
    if not SERIAL_AVAILABLE:
        print("Warning: pyserial not installed. Cannot select port.")
        return None

    ports = list(serial_list_ports.comports())
    if not ports:
        print("No serial ports found. Connect the ESP32 and try again.")
        return None
    if len(ports) == 1:
        selected = ports[0].device
        print(f"Only one port found, auto-selecting: {selected}")
        return selected
    print("\nAvailable serial ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx}] {p.device} - {p.description}")
    while True:
        sel = input("\nSelect port index (or press Enter to skip): ").strip()
        if sel == "":
            return None  # Skip selection
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Try again.")


def get_local_ip():
    """Get the local IP address of this machine."""
    import socket
    try:
        # Connect to a remote address to determine local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='ESP32 Car Control Dashboard Server')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Host to bind to (default: 0.0.0.0 for all interfaces)')
    parser.add_argument('--port', type=int, default=5000,
                        help='Port to bind to (default: 5000)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode')
    parser.add_argument('--auto-connect', action='store_true',
                        help='Automatically connect to a serial port on startup')
    parser.add_argument('--port-name', type=str, default=None,
                        help='Specific serial port to connect to (e.g., /dev/ttyUSB0)')
    parser.add_argument('--threshold', type=int, default=180,
                        help='Image processing threshold (default: 180)')
    parser.add_argument('--pid-kp', type=float, default=0.43,
                        help='PID proportional gain (default: 0.43)')
    parser.add_argument('--pid-ki', type=float, default=0.002,
                        help='PID integral gain (default: 0.002)')
    parser.add_argument('--pid-kd', type=float, default=0.12,
                        help='PID derivative gain (default: 0.12)')
    parser.add_argument('--max-angle', type=float, default=30.0,
                        help='Maximum steering angle in degrees (default: 30.0)')
    parser.add_argument('--deadband', type=float, default=6.0,
                        help='Deadband angle in degrees (default: 6.0)')

    args = parser.parse_args()

    local_ip = get_local_ip()

    print("=" * 60)
    print("ESP32 Car Control Dashboard Server")
    print("=" * 60)

    # Handle serial port connection
    if args.port_name:
        # Connect to specific port
        if SERIAL_AVAILABLE:
            try:
                serial_conn = open_serial(args.port_name, UART_BAUD_RATE)
                time.sleep(0.1)
                start_serial_reader()
                print(f"Connected to serial port: {args.port_name}")
            except Exception as e:
                print(f"Warning: Failed to connect to {args.port_name}: {e}")
                print("You can connect manually from the dashboard.")
        else:
            print("Warning: pyserial not installed. Cannot connect to serial port.")
    elif args.auto_connect:
        # Interactive port selection
        if SERIAL_AVAILABLE:
            selected_port = select_port_interactive()
            if selected_port:
                try:
                    serial_conn = open_serial(selected_port, UART_BAUD_RATE)
                    time.sleep(0.1)
                    start_serial_reader()
                    print(f"Connected to serial port: {selected_port}")
                except Exception as e:
                    print(
                        f"Warning: Failed to connect to {selected_port}: {e}")
                    print("You can connect manually from the dashboard.")
            else:
                print("No port selected. You can connect manually from the dashboard.")
        else:
            print("Warning: pyserial not installed. Cannot connect to serial port.")
    else:
        print("Serial port not connected. Use the dashboard to connect manually.")
        print("Tip: Use --auto-connect to select a port on startup, or --port-name to specify one.")

    # Initialize video streamer
    if VideoStreamer is not None:
        print("=" * 60)
        print("Initializing video streamer...")
        video_streamer = VideoStreamer()
        if video_streamer.initialize():
            print("✓ Video streamer initialized")
        else:
            print("⚠ Video streamer initialization failed - video features disabled")
            video_streamer = None
    else:
        print("⚠ Video streamer not available - autopilot modules not found")
        video_streamer = None

    # Initialize command sender (requires serial connection)
    if CommandSender is not None and serial_conn and serial_conn.is_open:
        command_sender = CommandSender(write_uart_command)
        print("✓ Command sender initialized")
        
        # Initialize and auto-start auto-pilot controller
        if AutoPilotController is not None:
            print("Initializing auto-pilot controller...")
            autopilot_controller = AutoPilotController(
                video_streamer=video_streamer,
                command_sender=command_sender,
                threshold=args.threshold,
                pid_kp=args.pid_kp,
                pid_ki=args.pid_ki,
                pid_kd=args.pid_kd,
                max_angle=args.max_angle,
                deadband=args.deadband
            )
            
            if video_streamer is not None:
                autopilot_controller.start()
                print("✓ Auto-pilot controller started")
            else:
                print("⚠ Auto-pilot controller not started - video streamer not available")
        else:
            print("⚠ Auto-pilot controller not available")
            autopilot_controller = None
    else:
        if CommandSender is None:
            print("⚠ Command sender not available - autopilot modules not found")
        else:
            print("⚠ Command sender not initialized - serial port not connected")
        print("⚠ Auto-pilot controller not initialized - serial port required")
        command_sender = None
        autopilot_controller = None

    print("=" * 60)
    print(f"Dashboard available at:")
    print(f"  Local:   http://127.0.0.1:{args.port}")
    print(f"  Network: http://{local_ip}:{args.port}")
    print("=" * 60)
    print("Press Ctrl+C to stop the server")
    print("=" * 60)

    try:
        app.run(host=args.host, port=args.port, debug=args.debug)
    finally:
        # Cleanup on shutdown
        if autopilot_controller:
            autopilot_controller.stop()
        if video_streamer:
            video_streamer.stop()
