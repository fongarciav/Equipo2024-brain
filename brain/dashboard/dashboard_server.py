
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
import json

# Import auto-pilot modules
import sys
import os
# Add parent directory to path to import brain modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

# Store import errors for debugging
import_errors = {}

try:
    from camera.video_streamer import VideoStreamer
except ImportError as e:
    print(f"Warning: Could not import VideoStreamer: {e}")
    VideoStreamer = None
    import_errors['VideoStreamer'] = str(e)

try:
    from camera.realsense_streamer import RealSenseStreamer
except ImportError as e:
    print(f"Warning: Could not import RealSenseStreamer: {e}")
    RealSenseStreamer = None
    import_errors['RealSenseStreamer'] = str(e)

try:
    from lane_detection.autopilot_controller import AutoPilotController
except ImportError as e:
    print(f"Warning: Could not import AutoPilotController: {e}")
    AutoPilotController = None
    import_errors['AutoPilotController'] = str(e)
except Exception as e:
    # Catch other exceptions (syntax errors, etc.)
    print(f"Warning: Error importing AutoPilotController: {e}")
    AutoPilotController = None
    import_errors['AutoPilotController'] = str(e)

try:
    from command_sender import CommandSender
except ImportError as e:
    print(f"Warning: Could not import CommandSender: {e}")
    CommandSender = None
    import_errors['CommandSender'] = str(e)

try:
    from lane_detection.lane_detector import MarcosLaneDetector_Advanced
except ImportError as e:
    print(f"Warning: Could not import MarcosLaneDetector_Advanced: {e}")
    MarcosLaneDetector_Advanced = None
    import_errors['MarcosLaneDetector_Advanced'] = str(e)

# Check if any critical modules are missing
if VideoStreamer is None or AutoPilotController is None or CommandSender is None or MarcosLaneDetector_Advanced is None:
    print("Warning: Some autopilot modules are not available. Auto-pilot features will be disabled.")

# Import sign vision modules
try:
    from sign_vision.sign_detector import SignDetector
except ImportError as e:
    print(f"Warning: Could not import SignDetector: {e}")
    SignDetector = None
    import_errors['SignDetector'] = str(e)
except Exception as e:
    # Catch other exceptions (syntax errors, etc.)
    print(f"Warning: Error importing SignDetector: {e}")
    SignDetector = None
    import_errors['SignDetector'] = str(e)

try:
    from sign_vision.sign_controller import SignController
except ImportError as e:
    print(f"Warning: Could not import SignController: {e}")
    SignController = None
    import_errors['SignController'] = str(e)
except Exception as e:
    print(f"Warning: Error importing SignController: {e}")
    SignController = None
    import_errors['SignController'] = str(e)

# Try to import serial
try:
    import serial
    from serial.tools import list_ports as serial_list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Install with: pip install pyserial", file=sys.stderr)

# Check CUDA availability
CUDA_AVAILABLE = False
CUDA_DEVICE_COUNT = 0
CUDA_ERROR = None
try:
    import cv2
    cuda_device_count = cv2.cuda.getCudaEnabledDeviceCount()
    CUDA_AVAILABLE = True
    CUDA_DEVICE_COUNT = cuda_device_count
except Exception as e:
    CUDA_AVAILABLE = False
    CUDA_DEVICE_COUNT = 0
    CUDA_ERROR = str(e)

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

# Heartbeat sender thread
heartbeat_sender_thread = None
heartbeat_sender_running = False

# Global event clients (combined stream)
event_clients = []
event_clients_lock = threading.Lock()

# SSE clients for separate streams (maintained for backward compatibility if needed, but we will consolidate)
# Note: We will route ALL events to event_clients as well

# SSE clients for autopilot status streaming (Deprecated - mapped to event_clients)
autopilot_status_broadcast_running = False
autopilot_status_broadcast_thread = None

# SSE clients for sign detection status streaming (Deprecated - mapped to event_clients)
sign_detection_status_broadcast_running = False
sign_detection_status_broadcast_thread = None

# ...

# System state tracking
# Deafult esp32 state is already armed and running, in manual mode
system_state = {'mode': 'MANUAL', 'state': 'RUNNING'}
system_state_lock = threading.Lock()

# Auto-pilot components
video_streamer = None
autopilot_controller = None
command_sender = None

# Sign vision components
sign_detector = None
sign_detection_controller = None

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
            # SERVO_RIGHT (50) is minimum, SERVO_LEFT (160) is maximum
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
                        # Fix for concatenated messages (e.g. EVENT:A...EVENT:B...)
                        # If a line contains multiple "EVENT:" markers after the start, split them
                        if 'EVENT:' in line[6:]:  # Skip checking the very start
                            parts = line.split('EVENT:')
                            # First part (might be empty if line started with EVENT:)
                            first = parts[0].strip()
                            if first:
                                lines.append(first)
                            
                            # Subsequent parts need "EVENT:" added back
                            for part in parts[1:]:
                                clean_part = f"EVENT:{part}".strip()
                                if clean_part:
                                    lines.append(clean_part)
                        else:
                             lines.append(line)

                # Then check for \n (Unix line ending)
                elif '\n' in serial_read_buffer:
                    line, serial_read_buffer = serial_read_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        # Fix for concatenated messages (duplicate logic)
                        if 'EVENT:' in line[6:]:
                            parts = line.split('EVENT:')
                            first = parts[0].strip()
                            if first:
                                lines.append(first)
                            for part in parts[1:]:
                                clean_part = f"EVENT:{part}".strip()
                                if clean_part:
                                    lines.append(clean_part)
                        else:
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
                        # Fix for concatenated messages in readline fallback
                        if 'EVENT:' in line[6:]:
                            parts = line.split('EVENT:')
                            first = parts[0].strip()
                            if first:
                                lines.append(first)
                            for part in parts[1:]:
                                clean_part = f"EVENT:{part}".strip()
                                if clean_part:
                                    lines.append(clean_part)
                        else:
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
        # Extract only the first word to avoid trailing garbage/extra messages
        raw_state = line.replace('EVENT:STATE_CHANGED:', '').strip()
        state = raw_state.split()[0] if raw_state else raw_state
        
        with system_state_lock:
            system_state['state'] = state
            print(f"[SystemState] State changed to: {state} (raw: {raw_state})")
    elif line.startswith('EVENT:MODE_CHANGED:'):
        # Extract only the first word
        raw_mode = line.replace('EVENT:MODE_CHANGED:', '').strip()
        mode = raw_mode.split()[0] if raw_mode else raw_mode
        
        with system_state_lock:
            system_state['mode'] = mode
            print(f"[SystemState] Mode changed to: {mode} (raw: {raw_mode})")


def serial_reader_worker():
    """Background thread that reads from serial port."""
    global serial_conn, serial_reader_running, event_clients

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
                        # Send to unified event clients
                        with event_clients_lock:
                            # Send to all connected clients
                            # Copy list to avoid modification during iteration
                            for client_queue in event_clients[:]:
                                try:
                                    client_queue.put(message_data, block=False)
                                except queue.Full:
                                    # Client queue is full, remove it (client might be disconnected)
                                    try:
                                        event_clients.remove(client_queue)
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


def heartbeat_worker():
    """Background thread that sends heartbeat to ESP32 when in AUTO mode."""
    global serial_conn, heartbeat_sender_running, system_state, command_sender, system_state_lock

    print("[Heartbeat] Worker thread started")

    while heartbeat_sender_running:
        if serial_conn and serial_conn.is_open and command_sender:
            # Check if we are in AUTO mode
            is_auto = False
            with system_state_lock:
                 is_auto = (system_state.get('mode') == 'AUTO')
                 current_mode = system_state.get('mode')
            
            # Debug log only when state changes or periodically (to avoid spam)
            # For now, let's print if we think we should be sending
            if is_auto:
                try:
                    command_sender.send_heartbeat()
                    # Log every ~1 second (20 cycles * 50ms) to avoid spam but confirm activity
                    if int(time.time()) % 2 == 0 and int(time.time() * 20) % 20 == 0:
                         print(".", end="", flush=True)
                except Exception as e:
                    print(f"[Heartbeat] Error sending heartbeat: {e}", file=sys.stderr)
            else:
                 pass
                 # print(f"[Heartbeat] Skipped (Mode: {current_mode})", file=sys.stderr)
        else:
             print(f"[Heartbeat] Not ready - Serial: {serial_conn and serial_conn.is_open}, Sender: {command_sender is not None}")
            
        time.sleep(0.05)  # Send every 50ms (Watchdog is 120ms)

    print("[Heartbeat] Worker thread stopped")


def start_heartbeat_sender():
    """Start the heartbeat sender thread."""
    global heartbeat_sender_thread, heartbeat_sender_running

    if heartbeat_sender_thread is None or not heartbeat_sender_thread.is_alive():
        heartbeat_sender_running = True
        heartbeat_sender_thread = threading.Thread(
            target=heartbeat_worker, daemon=True)
        heartbeat_sender_thread.start()


def stop_heartbeat_sender():
    """Stop the heartbeat sender thread."""
    global heartbeat_sender_running, heartbeat_sender_thread

    heartbeat_sender_running = False
    if heartbeat_sender_thread:
        heartbeat_sender_thread.join(timeout=1.0)
        heartbeat_sender_thread = None


def autopilot_status_broadcast_worker():
    """Background thread that broadcasts autopilot status to SSE clients."""
    global autopilot_status_broadcast_running, autopilot_controller, event_clients
    
    while autopilot_status_broadcast_running:
        try:
            status_data = None
            if autopilot_controller is not None:
                status = autopilot_controller.get_status()
                pid_params = autopilot_controller.get_pid_parameters()
                status_data = {
                    'type': 'autopilot_status',
                    'status': status,
                    'pid_parameters': pid_params,
                    'timestamp': time.time()
                }
            else:
                # Controller not initialized
                status_data = {
                    'type': 'autopilot_status',
                    'error': 'Auto-pilot controller not initialized',
                    'timestamp': time.time()
                }
            
            # Broadcast to unified event clients
            with event_clients_lock:
                for client_queue in event_clients[:]:
                    try:
                        client_queue.put(status_data, block=False)
                    except queue.Full:
                        # Client queue is full, remove it (client might be disconnected)
                        try:
                            event_clients.remove(client_queue)
                        except ValueError:
                            pass  # Already removed
            
            # Update every 0.5 seconds
            time.sleep(0.5)
        except Exception as e:
            if autopilot_status_broadcast_running:
                print(f"[AutopilotStatusBroadcast] Error: {e}", file=sys.stderr)
                time.sleep(0.5)


def start_autopilot_status_broadcast():
    """Start the autopilot status broadcast thread."""
    global autopilot_status_broadcast_thread, autopilot_status_broadcast_running
    
    if autopilot_status_broadcast_thread is None or not autopilot_status_broadcast_thread.is_alive():
        autopilot_status_broadcast_running = True
        autopilot_status_broadcast_thread = threading.Thread(
            target=autopilot_status_broadcast_worker, daemon=True)
        autopilot_status_broadcast_thread.start()


def stop_autopilot_status_broadcast():
    """Stop the autopilot status broadcast thread."""
    global autopilot_status_broadcast_running, autopilot_status_broadcast_thread
    
    autopilot_status_broadcast_running = False
    if autopilot_status_broadcast_thread:
        autopilot_status_broadcast_thread.join(timeout=1.0)
        autopilot_status_broadcast_thread = None


def sign_detection_status_broadcast_worker():
    """Background thread that broadcasts sign detection status to SSE clients."""
    global sign_detection_status_broadcast_running, sign_detection_controller, event_clients, sign_detector
    
    while sign_detection_status_broadcast_running:
        try:
            status_data = None
            if sign_detection_controller is not None:
                status = sign_detection_controller.get_status()
                
                # Get extra status from DETECTOR (frames, device, etc.)
                detector_status = {}
                detections = []
                if sign_detector:
                    detector_status = sign_detector.get_status()
                    detections = sign_detector.get_detections()
                
                # Merge detector status into main status object so frontend can see frame_count, device, etc.
                status.update(detector_status)
                
                status_data = {
                    'type': 'sign_detection_status',
                    'status': status,
                    'detections': detections,
                    'timestamp': time.time()
                }
            else:
                # Controller not initialized
                status_data = {
                    'type': 'sign_detection_status',
                    'error': 'Sign detection controller not initialized',
                    'timestamp': time.time()
                }
            
            # Broadcast to unified event clients
            with event_clients_lock:
                for client_queue in event_clients[:]:
                    try:
                        client_queue.put(status_data, block=False)
                    except queue.Full:
                        # Client queue is full, remove it (client might be disconnected)
                        try:
                            event_clients.remove(client_queue)
                        except ValueError:
                            pass  # Already removed
            
            # Update every 0.5 seconds
            time.sleep(0.5)
        except Exception as e:
            if sign_detection_status_broadcast_running:
                print(f"[SignDetectionStatusBroadcast] Error: {e}", file=sys.stderr)
                time.sleep(0.5)


def start_sign_detection_status_broadcast():
    """Start the sign detection status broadcast thread."""
    global sign_detection_status_broadcast_thread, sign_detection_status_broadcast_running
    
    if sign_detection_status_broadcast_thread is None or not sign_detection_status_broadcast_thread.is_alive():
        sign_detection_status_broadcast_running = True
        sign_detection_status_broadcast_thread = threading.Thread(
            target=sign_detection_status_broadcast_worker, daemon=True)
        sign_detection_status_broadcast_thread.start()


def stop_sign_detection_status_broadcast():
    """Stop the sign detection status broadcast thread."""
    global sign_detection_status_broadcast_running, sign_detection_status_broadcast_thread
    
    sign_detection_status_broadcast_running = False
    if sign_detection_status_broadcast_thread:
        sign_detection_status_broadcast_thread.join(timeout=1.0)
        sign_detection_status_broadcast_thread = None


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
    if CommandSender is None or AutoPilotController is None or MarcosLaneDetector_Advanced is None:
        print("Autopilot modules not available", file=sys.stderr)
        return False
    
    if not serial_conn or not serial_conn.is_open:
        print("Serial port not connected", file=sys.stderr)
        return False
    
    # Initialize command sender
    if command_sender is None:
        command_sender = CommandSender(write_uart_command)
    
    # Initialize video streamer if not already done
    if video_streamer is None:
        # Try RealSense first if available
        if RealSenseStreamer is not None:
            print(f"[Dashboard] Attempting to initialize RealSense camera using: {RealSenseStreamer}")
            rs_streamer = RealSenseStreamer()
            if rs_streamer.initialize():
                video_streamer = rs_streamer
                print("[Dashboard] RealSense camera initialized successfully.")
            else:
                print("[Dashboard] RealSense initialization failed, falling back to standard VideoStreamer.")
        else:
            print("[Dashboard] RealSenseStreamer class not available (import failed).")
        
        # Fallback to standard VideoStreamer if RealSense failed or not available
        if video_streamer is None and VideoStreamer is not None:
            video_streamer = VideoStreamer()
            if not video_streamer.initialize():
                return False
    
    # Initialize autopilot controller
    if video_streamer is not None:
        if MarcosLaneDetector_Advanced is None:
            return False
        
        # Instantiate lane detector strategy
        lane_detector = MarcosLaneDetector_Advanced(threshold=180)
        
        # Initialize autopilot controller with default parameters
        autopilot_controller = AutoPilotController(
            video_streamer=video_streamer,
            command_sender=command_sender,
            lane_detector=lane_detector,
            pid_kp=0.8380,
            pid_ki=0.0100,
            pid_kd=0.4300,
            max_angle=30.0,
            deadband=6.0
        )
        
        # If sign controller exists, update its reference to autopilot
        global sign_detection_controller
        if sign_detection_controller is not None:
             sign_detection_controller.autopilot_controller = autopilot_controller
             print("[Dashboard] Linked AutopilotController to SignController")
             
        return True
    else:
        return False

def on_sign_controller_event(event_type, data):
    """Callback to handle events from SignController."""
    global event_clients, event_clients_lock
    
    if event_type == "sign_detected":
        message = f"[SignController] {data.get('message', 'Sign detected')}"
        
        # Create log message for dashboard (use specialized log type)
        log_data = {
            'type': 'server_log',
            'message': message,
            'timestamp': time.time()
        }
        
        # Broadcast
        with event_clients_lock:
            for client_queue in event_clients[:]:
                try:
                    client_queue.put(log_data, block=False)
                except:
                    pass


def initialize_sign_detection_if_needed():
    """Initialize sign detection controller if conditions are met."""
    global sign_detection_controller, sign_detector, video_streamer, serial_conn, command_sender, autopilot_controller
    
    # Only initialize if not already initialized
    if sign_detection_controller is not None:
        return True
    
    # Check if we have all required components
    if SignDetector is None or SignController is None:
        print("Sign detection modules not available", file=sys.stderr)
        return False
    
    if not serial_conn or not serial_conn.is_open:
        print("Serial port not connected", file=sys.stderr)
        return False
        
    # Initialize command sender if needed
    if command_sender is None:
        command_sender = CommandSender(write_uart_command)
    
    # Initialize video streamer if not already done
    if video_streamer is None:
        # Try RealSense first if available
        if RealSenseStreamer is not None:
            print(f"[Dashboard] Attempting to initialize RealSense camera using: {RealSenseStreamer}")
            rs_streamer = RealSenseStreamer()
            if rs_streamer.initialize():
                video_streamer = rs_streamer
                print("[Dashboard] RealSense camera initialized successfully.")
            else:
                print("[Dashboard] RealSense initialization failed, falling back to standard VideoStreamer.")
        else:
            print("[Dashboard] RealSenseStreamer class not available (import failed).")
        
        # Fallback to standard VideoStreamer if RealSense failed or not available
        if video_streamer is None and VideoStreamer is not None:
            video_streamer = VideoStreamer()
            if not video_streamer.initialize():
                video_streamer = None
                return False
    
    # Create sign detection controller if video_streamer is available
    if video_streamer is not None:
        # Initialize the detector (sensor)
        if sign_detector is None:
            sign_detector = SignDetector(
                video_streamer=video_streamer,
                confidence_threshold=0.6
            )
            if not sign_detector.initialize():
                sign_detector = None
                return False
                
        # Initialize the controller (decision maker)
        sign_detection_controller = SignController(
            sign_detector=sign_detector,
            command_sender=command_sender,
            event_callback=on_sign_controller_event,
            autopilot_controller=autopilot_controller
        )
        
        return True
    else:
        return False

@app.route('/uart/connect', methods=['POST'])
def uart_connect():
    """Connect to ESP32 via UART."""
    global serial_conn, serial_read_buffer, command_sender

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
        
        # Initialize command sender immediately after connection
        if command_sender is None:
            command_sender = CommandSender(write_uart_command)
            
        time.sleep(0.1)
        start_serial_reader()
        print("[Dashboard] Started serial reader")
        start_heartbeat_sender()
        print("[Dashboard] Started heartbeat sender")
        
        response = {
            'status': 'ok',
            'message': f'Connected to {port}',
            'port': port,
            'baudrate': UART_BAUD_RATE,
        }
        return jsonify(response)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/uart/disconnect', methods=['POST'])
def uart_disconnect():
    """Disconnect from ESP32."""
    global serial_conn, serial_read_buffer, autopilot_controller, sign_detection_controller, sign_detector

    stop_serial_reader()
    stop_heartbeat_sender()
    
    # Stop autopilot if running
    if autopilot_controller:
        autopilot_controller.stop()
    
    # Stop sign detection if running
    if sign_detection_controller:
        sign_detection_controller.stop()
        # Also stop the detector (the thread)
        if sign_detector:
            sign_detector.stop()

    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        serial_conn = None
    
    serial_read_buffer = ""  # Clear buffer on disconnect

    return jsonify({'status': 'ok', 'message': 'Disconnected'})


@app.route('/events')
def events_stream():
    """Unified Server-Sent Events stream for all system events."""
    # Start all broadcast threads if not already running
    start_serial_reader()
    start_autopilot_status_broadcast()
    start_sign_detection_status_broadcast()
    
    def generate():
        # Create a dedicated queue for this client
        client_queue = queue.Queue(maxsize=100)

        # Register this client
        with event_clients_lock:
            event_clients.append(client_queue)

        try:
            while True:
                try:
                    # Get message from this client's queue
                    msg = client_queue.get(timeout=1.0)
                    
                    # If message is a dict (status update), serialize it
                    if isinstance(msg, dict):
                        if msg.get('type') == 'serial':
                            # Serial messages are raw strings in 'message' field
                            message = msg.get('message', '')
                            if message and len(message.strip()) > 0:
                                # Compatibility: send raw serial line as 'data' for backward compatibility with serial listener
                                # BUT we should wrap it in JSON ideally. 
                                # For now, let's check what the client expects.
                                # The new client will parse JSON.
                                json_data = json.dumps(msg)
                                yield f"data: {json_data}\n\n"
                        else:
                            # Status updates are dicts
                            json_data = json.dumps(msg)
                            yield f"data: {json_data}\n\n"
                    else:
                         # Fallback for string messages
                         yield f"data: {msg}\n\n"

                except queue.Empty:
                    yield ": heartbeat\n\n"
        except GeneratorExit:
            pass  # Client disconnected
        except Exception as e:
            print(f"[EventsSSE] Error: {e}", file=sys.stderr)
        finally:
            # Unregister this client
            with event_clients_lock:
                try:
                    event_clients.remove(client_queue)
                except ValueError:
                    pass  # Already removed

    response = Response(stream_with_context(generate()),
                        mimetype='text/event-stream')
    response.headers['Cache-Control'] = 'no-cache'
    response.headers['X-Accel-Buffering'] = 'no'  # Disable buffering for nginx
    return response


# Deprecated streams - kept for compatibility if user hasn't refreshed page
# We redirect them to the unified logic or just keep them working but with reduced functionality

@app.route('/uart/stream')
def uart_stream_deprecated():
    """Deprecated: Use /events instead."""
    return events_stream()

@app.route('/autopilot/status/stream')
def autopilot_status_stream_deprecated():
    """Deprecated: Use /events instead."""
    return events_stream()

@app.route('/sign_detection/status/stream')
def sign_detection_status_stream_deprecated():
    """Deprecated: Use /events instead."""
    return events_stream()


@app.route('/status')
def get_status():
    """Get current system status (mode and state)."""
    global system_state
    with system_state_lock:
        return jsonify({
            'mode': system_state['mode'],
            'state': system_state['state']
        })


def generate_debug_mjpeg(image_key=None, mode='autopilot', 
                         waiting_text='Waiting...', not_running_text='Not running'):
    """
    Generate MJPEG stream from debug images.
    
    Args:
        image_key: Key for autopilot debug images (e.g., 'bird_view_lines', 'sliding_windows', 'final_result')
        mode: 'autopilot' or 'sign_detector'
        waiting_text: Text to show when waiting
        not_running_text: Text to show when controller is not running
    """
    import cv2
    import time
    import numpy as np
    
    # Declare globals explicitly
    global autopilot_controller, sign_detector
    
    # Create a placeholder black image
    placeholder_base = np.zeros((480, 640, 3), dtype=np.uint8)
    
    while True:
        debug_image = None
        is_running = False
        
        try:
            if mode == 'autopilot':
                if autopilot_controller is not None:
                    # Check status
                    try:
                        status = autopilot_controller.get_status()
                        is_running = status.get('is_running', False)
                    except:
                        is_running = False
                        
                    # Get image only if running
                    if is_running:
                        debug_image = autopilot_controller.get_debug_image(image_key)
            
            elif mode == 'sign_detector':
                if sign_detector is not None:
                    # Check status
                    try:
                        status = sign_detector.get_status()
                        is_running = status.get('is_running', False)
                    except:
                        is_running = False
                        
                    # Get image only if running
                    if is_running:
                        debug_image = sign_detector.get_detection_image()
                    
        except Exception as e:
            # print(f"[Debug Stream] Error fetching data: {e}")
            debug_image = None
            
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
                        # Encoding failed
                        debug_image = None
                else:
                    # Invalid image
                    debug_image = None
            except Exception as e:
                # print(f"[Debug Stream] Error processing image: {e}")
                debug_image = None
        
        # If we have no image (either fetch failed, or processing failed, or just not available)
        if debug_image is None:
            # Create appropriate placeholder text
            placeholder_text = not_running_text if not is_running else waiting_text
            
            placeholder = placeholder_base.copy()
            cv2.putText(placeholder, placeholder_text, (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS


@app.route('/debug/bird_view_lines')
def debug_bird_view_lines():
    """MJPEG stream for bird view with lines."""
    return Response(
        stream_with_context(generate_debug_mjpeg(mode='autopilot', image_key='bird_view_lines', waiting_text='Waiting for lane detection...', not_running_text='Lane detection not running')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/debug/sliding_windows')
def debug_sliding_windows():
    """MJPEG stream for sliding windows."""
    return Response(
        stream_with_context(generate_debug_mjpeg(mode='autopilot', image_key='sliding_windows', waiting_text='Waiting for lane detection...', not_running_text='Lane detection not running')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/debug/final_result')
def debug_final_result():
    """MJPEG stream for final result."""
    return Response(
        stream_with_context(generate_debug_mjpeg(mode='autopilot', image_key='final_result', waiting_text='Waiting for lane detection...', not_running_text='Lane detection not running')),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/autopilot/start', methods=['POST'])
def autopilot_start():
    """Start the auto-pilot controller."""
    global autopilot_controller
    
    # Try to initialize if not already initialized
    if autopilot_controller is None:
        # Quick checks before potentially blocking initialization
        if CommandSender is None or AutoPilotController is None or MarcosLaneDetector_Advanced is None:
            response = {
                'error': 'Auto-pilot controller not initialized',
                'details': {
                    'modules_available': {
                        'CommandSender': CommandSender is not None,
                        'AutoPilotController': AutoPilotController is not None,
                        'VideoStreamer': VideoStreamer is not None,
                        'MarcosLaneDetector_Advanced': MarcosLaneDetector_Advanced is not None
                    }
                },
                'suggestions': ['Autopilot modules not found - check that you are in brain/dashboard directory']
            }
            # Add import errors if available
            if import_errors:
                response['details']['import_errors'] = import_errors
            return jsonify(response), 503
        
        if not serial_conn or not serial_conn.is_open:
            return jsonify({
                'error': 'Auto-pilot controller not initialized',
                'details': {'serial_connected': False},
                'suggestions': ['Connect UART port first']
            }), 503
        
        # Initialize (may block briefly if camera is slow, but optimized to be fast)
        initialized = initialize_autopilot_if_needed()
        if not initialized:
            # Get detailed status for error message
            status = {
                'error': 'Auto-pilot controller not initialized',
                'details': {
                    'modules_available': {
                        'CommandSender': CommandSender is not None,
                        'AutoPilotController': AutoPilotController is not None,
                        'VideoStreamer': VideoStreamer is not None,
                        'MarcosLaneDetector_Advanced': MarcosLaneDetector_Advanced is not None
                    },
                    'serial_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
                    'video_streamer_initialized': video_streamer is not None
                },
                'suggestions': []
            }
            
            # Add import errors if available
            if import_errors:
                status['details']['import_errors'] = import_errors
            
            if not status['details']['serial_connected']:
                status['suggestions'].append('Connect UART port first')
            if not status['details']['video_streamer_initialized']:
                if not status['details']['modules_available']['VideoStreamer']:
                    status['suggestions'].append('Install autopilot modules: pip install opencv-python numpy')
                else:
                    status['suggestions'].append('Connect camera USB device')
            if not status['details']['modules_available']['AutoPilotController'] or not status['details']['modules_available']['MarcosLaneDetector_Advanced']:
                status['suggestions'].append('Autopilot modules not found - check that you are in brain/dashboard directory')
            
            return jsonify(status), 503
    
    success = autopilot_controller.start()
    if success:
        # Automatically switch to AUTO mode when autopilot starts
        with system_state_lock:
            system_state['mode'] = 'AUTO'
        
        # Send mode command to ESP32
        if command_sender:
            command_sender.write_uart_command("M:SYS_MODE:1")
        
        # Restart sign detector if running to prevent stream freezing
        # This handles the case where starting Autopilot might interfere with an existing SignDetector loop
        if sign_detector is not None:
            try:
                status = sign_detector.get_status()
                if status.get('is_running', False):
                    print("Restarting SignDetector to prevent freeze...", file=sys.stderr)
                    sign_detector.stop()
                    time.sleep(0.1) # Brief pause to allow thread cleanup
                    sign_detector.start()
            except Exception as e:
                print(f"Error restarting SignDetector: {e}", file=sys.stderr)

        return jsonify({'status': 'ok', 'message': 'Auto-pilot started'})
    else:
        return jsonify({'error': 'Auto-pilot already running'}), 400

@app.route('/autopilot/stop', methods=['POST'])
def autopilot_stop():
    """Stop the auto-pilot controller."""
    global autopilot_controller, video_streamer, sign_detection_controller
    if autopilot_controller is None:
        return jsonify({'error': 'Auto-pilot controller not initialized'}), 503
    
    success = autopilot_controller.stop()
    if success:
        # Check if video_streamer should be stopped (only if sign_detection is also not running)
        if video_streamer is not None:
            sign_detection_running = False
            if sign_detection_controller is not None:
                sign_status = sign_detection_controller.get_status()
                sign_detection_running = sign_status.get('is_running', False)
            
            # If neither autopilot nor sign_detection is running, stop video_streamer
            if not sign_detection_running:
                print("Stopping video streamer (no controllers running)", file=sys.stderr)
                video_streamer.stop()
                video_streamer = None
                # Also set autopilot_controller to None to force re-initialization
                # This ensures video_streamer will be recreated when starting again
                autopilot_controller = None
                # IMPORTANT: Also clear sign detectors because they hold a reference to the now-stopped video_streamer
                sign_detection_controller = None
                sign_detector = None
        
        return jsonify({'status': 'ok', 'message': 'Auto-pilot stopped'})
    else:
        return jsonify({'error': 'Auto-pilot not running'}), 400

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
    # Handle both 'deadband' and 'tolerance' (dashboard sends 'tolerance')
    deadband = data.get('deadband')
    if deadband is None:
        deadband = data.get('tolerance')
    
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


@app.route('/sign_detection/start', methods=['POST'])
def sign_detection_start():
    """Start the sign detection controller."""
    global sign_detection_controller, sign_detector
    
    # Try to initialize if not already initialized
    if sign_detection_controller is None:
        initialized = initialize_sign_detection_if_needed()
        if not initialized:
            # Get detailed status for error message
            status = {
                'error': 'Sign detection controller not initialized',
                'details': {
                    'modules_available': {
                        'SignDetector': SignDetector is not None,
                        'SignController': SignController is not None,
                        'VideoStreamer': VideoStreamer is not None
                    },
                    'serial_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
                    'video_streamer_initialized': video_streamer is not None
                },
                'suggestions': []
            }
            
            # Add import errors if available
            if import_errors:
                status['details']['import_errors'] = import_errors
            
            if not status['details']['serial_connected']:
                status['suggestions'].append('Connect UART port first')
            if not status['details']['video_streamer_initialized']:
                if not status['details']['modules_available']['VideoStreamer']:
                    status['suggestions'].append('Install autopilot modules: pip install opencv-python numpy')
                else:
                    status['suggestions'].append('Connect camera USB device')
            if not status['details']['modules_available']['SignDetector']:
                status['suggestions'].append('Sign detection modules not found - check that sign_vision module is available')
            
            return jsonify(status), 503
    
    # Start both the detector (eye) and controller (brain)
    detector_success = sign_detector.start()
    controller_success = sign_detection_controller.start()
    
    if detector_success or controller_success:
        # Automatically switch to AUTO mode when sign detection starts
        # Important: Update state BEFORE sending command so heartbeat starts immediately
        with system_state_lock:
            system_state['mode'] = 'AUTO'
            
        # Send mode command to ESP32
        if command_sender:
            command_sender.write_uart_command("M:SYS_MODE:1")
            # Send an immediate heartbeat to reset the watchdog timer on the ESP32
            try:
                command_sender.send_heartbeat()
            except:
                pass
            
        return jsonify({'status': 'ok', 'message': 'Sign detection started'})
    else:
        return jsonify({'error': 'Sign detection already running'}), 400

@app.route('/sign_detection/stop', methods=['POST'])
def sign_detection_stop():
    """Stop the sign detection controller."""
    global sign_detection_controller, video_streamer, autopilot_controller, sign_detector
    if sign_detection_controller is None:
        return jsonify({'error': 'Sign detection controller not initialized'}), 503
    
    # Stop both
    controller_success = sign_detection_controller.stop()
    detector_success = sign_detector.stop()
    
    if controller_success or detector_success:
        # Check if video_streamer should be stopped (only if autopilot is also not running)
        if video_streamer is not None:
            autopilot_running = False
            if autopilot_controller is not None:
                autopilot_status = autopilot_controller.get_status()
                autopilot_running = autopilot_status.get('is_running', False)
            
            # If neither autopilot nor sign_detection is running, stop video_streamer
            if not autopilot_running:
                print("Stopping video streamer (no controllers running)", file=sys.stderr)
                video_streamer.stop()
                video_streamer = None
                # Also set global variables to None to force re-initialization
                # This ensures video_streamer will be recreated when starting again
                sign_detection_controller = None
                sign_detector = None
                # IMPORTANT: Also clear autopilot_controller because it holds a reference to the now-stopped video_streamer
                autopilot_controller = None
        
        return jsonify({'status': 'ok', 'message': 'Sign detection stopped'})
    else:
        return jsonify({'error': 'Sign detection not running'}), 400

@app.route('/sign_detection/confidence', methods=['POST'])
def sign_detection_update_confidence():
    """Update confidence threshold."""
    global sign_detection_controller, sign_detector
    if sign_detection_controller is None:
        return jsonify({'error': 'Sign detection controller not initialized'}), 503
    
    data = request.get_json() or {}
    threshold = data.get('threshold')
    
    if threshold is None or not isinstance(threshold, (int, float)) or not (0.0 <= threshold <= 1.0):
        return jsonify({'error': 'Invalid threshold value (must be between 0.0 and 1.0)'}), 400
    
    # Update the DETECTOR directly
    if sign_detector:
        sign_detector.update_confidence_threshold(float(threshold))
        status = sign_detector.get_status()
        return jsonify({
            'status': 'ok',
            'message': 'Confidence threshold updated',
            'confidence_threshold': status['confidence_threshold']
        })
    else:
        return jsonify({'error': 'Sign detector not initialized'}), 500


@app.route('/debug/sign_detections')
def debug_sign_detections():
    """MJPEG stream for sign detections."""
    return Response(
        stream_with_context(generate_debug_mjpeg(
            mode='sign_detector',
            waiting_text='Waiting for sign detection...',
            not_running_text='Sign detection not running'
        )),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/health')
def health():
    """Health check endpoint."""
    global serial_conn, serial_reader_running, video_streamer, autopilot_controller, sign_detection_controller
    port_name = None
    if serial_conn and serial_conn.is_open:
        port_name = serial_conn.port
    
    autopilot_status_info = None
    if autopilot_controller is not None:
        autopilot_status_info = autopilot_controller.get_status()
    
    sign_detection_status_info = None
    if sign_detection_controller is not None:
        sign_detection_status_info = sign_detection_controller.get_status()
    
    return jsonify({
        'status': 'ok',
        'message': 'Dashboard server is running',
        'uart_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
        'uart_port': port_name,
        'serial_available': SERIAL_AVAILABLE,
        'serial_reader_running': serial_reader_running,
        'video_streamer_initialized': video_streamer is not None,
        'autopilot_status': autopilot_status_info,
        'sign_detection_status': sign_detection_status_info
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
    parser.add_argument('--lane-detection', action='store_true',
                        help='Initialize lane detection (autopilot) controller on startup (requires --auto-connect or --port-name)')
    parser.add_argument('--sign-detection', action='store_true',
                        help='Initialize sign detection controller on startup (requires --auto-connect or --port-name)')
    parser.add_argument('--threshold', type=int, default=180,
                        help='Image processing threshold (default: 180)')
    parser.add_argument('--pid-kp', type=float, default=0.8380,
                        help='PID proportional gain (default: 0.8380)')
    parser.add_argument('--pid-ki', type=float, default=0.0100,
                        help='PID integral gain (default: 0.0100)')
    parser.add_argument('--pid-kd', type=float, default=0.4300,
                        help='PID derivative gain (default: 0.4300)')
    parser.add_argument('--max-angle', type=float, default=30.0,
                        help='Maximum steering angle in degrees (default: 30.0)')
    parser.add_argument('--deadband', type=float, default=6.0,
                        help='Deadband angle in degrees (default: 6.0)')

    args = parser.parse_args()

    local_ip = get_local_ip()

    print("=" * 70)
    print(" " * 15 + "ESP32 Car Control Dashboard Server")
    print("=" * 70)
    print()
    
    # Handle serial port connection
    uart_connected = False
    print("📡 Serial Port Connection")
    print("-" * 70)
    if args.port_name:
        # Connect to specific port
        if SERIAL_AVAILABLE:
            try:
                print(f"   Connecting to {args.port_name}...", end=" ")
                serial_conn = open_serial(args.port_name, UART_BAUD_RATE)
                time.sleep(0.1)
                start_serial_reader()
                start_heartbeat_sender()
                print(f"✓ Connected at {UART_BAUD_RATE} baud")
                uart_connected = True
            except Exception as e:
                print(f"✗ Failed: {e}")
                print("   You can connect manually from the dashboard.")
        else:
            print("   ✗ pyserial not installed. Cannot connect to serial port.")
    elif args.auto_connect:
        # Interactive port selection
        if SERIAL_AVAILABLE:
            selected_port = select_port_interactive()
            if selected_port:
                try:
                    print(f"   Connecting to {selected_port}...", end=" ")
                    serial_conn = open_serial(selected_port, UART_BAUD_RATE)
                    time.sleep(0.1)
                    start_serial_reader()
                    start_heartbeat_sender()
                    print(f"✓ Connected at {UART_BAUD_RATE} baud")
                    uart_connected = True
                except Exception as e:
                    print(f"✗ Failed: {e}")
                    print("   You can connect manually from the dashboard.")
            else:
                print("   ⚠ No port selected. You can connect manually from the dashboard.")
        else:
            print("   ✗ pyserial not installed. Cannot connect to serial port.")
    else:
        print("   ⚠ Not connected. Use the dashboard to connect manually.")
        print("   Tip: Use --auto-connect to select a port on startup, or --port-name to specify one.")
    print()
    
    # Initialize controllers if requested and UART is connected
    if uart_connected:
        print("🚗 Controller Initialization")
        print("-" * 70)
        
        if args.lane_detection:
            print("   [1/2] Lane Detection (Autopilot)")
            initialized = initialize_autopilot_if_needed()
            if initialized:
                success = autopilot_controller.start()
                if success:
                    print("      ✓ Autopilot controller initialized and started")
                else:
                    print("      ✗ Failed to start autopilot controller")
            else:
                print("      ✗ Autopilot initialization failed - check camera and modules")
        
        if args.sign_detection:
            print("   [2/2] Sign Detection")
            initialized = initialize_sign_detection_if_needed()
            if initialized:
                detector_success = sign_detector.start()
                controller_success = sign_detection_controller.start()
                if detector_success and controller_success:
                    print("      ✓ Sign detection controller initialized and started")
                else:
                    print("      ✗ Failed to start sign detection (detector: {}, controller: {})".format(
                        "✓" if detector_success else "✗",
                        "✓" if controller_success else "✗"
                    ))
            else:
                print("      ✗ Sign detection initialization failed - check camera and modules")
        
        if args.lane_detection or args.sign_detection:
            print()
    else:
        # No UART connected, controllers will be initialized when UART connects from dashboard
        print("🚗 Controller Initialization")
        print("-" * 70)
        if VideoStreamer is None:
            print("   ⚠ Video streamer not available - autopilot modules not found")
        else:
            print("   ℹ Controllers will be initialized when started from the dashboard.")
        print()

    print("=" * 70)
    print("🌐 Dashboard Server")
    print("-" * 70)
    print(f"   Local:   http://127.0.0.1:{args.port}")
    print(f"   Network: http://{local_ip}:{args.port}")
    print()
    print("   Press Ctrl+C to stop the server")
    print("=" * 70)
    print()

    try:
        # Use threaded=True to handle multiple requests concurrently
        # This prevents blocking when video streamer is capturing frames
        app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)
    finally:
        # Cleanup on shutdown
        stop_sign_detection_status_broadcast()
        stop_autopilot_status_broadcast()
        stop_serial_reader()
        stop_heartbeat_sender()
        if sign_detection_controller:
            sign_detection_controller.stop()
        if autopilot_controller:
            autopilot_controller.stop()
        if video_streamer:
            video_streamer.stop()
