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
UART_BAUD_RATE = 921600

# Global serial connection
serial_conn = None
serial_lock = threading.Lock()
serial_message_queue = queue.Queue()
serial_reader_thread = None
serial_reader_running = False

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
    SERVO_LEFT = 50
    SERVO_RIGHT = 135
    MOTOR_SPEED_MAX = 255
    
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
        return True, "FORWARD", f"C:SET_SPEED:{MOTOR_SPEED_MAX}"
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
    elif endpoint == 'steer':
        angle_str = args.get('angle', '105')
        try:
            angle = int(angle_str)
            if angle < SERVO_LEFT or angle > SERVO_RIGHT:
                return False, f"Angle must be {SERVO_LEFT}-{SERVO_RIGHT}", None
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
        timeout=1.0,
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
    """Read all available lines from serial port, like test_uart_simulator.py"""
    lines = []
    if not ser or not ser.is_open:
        return lines
    
    try:
        while True:
            data = ser.readline()
            if not data:
                break
            try:
                line = data.decode('utf-8', errors='ignore').rstrip()
                # Debug: print raw data to see what we're receiving
                if len(data) > 0:
                    print(f"[DEBUG] Raw data: {repr(data)}, Decoded: {repr(line)}", file=sys.stderr)
                if line:
                    lines.append(line)
            except Exception as e:
                print(f"[DEBUG] Decode error: {e}, data: {repr(data)}", file=sys.stderr)
                pass
    except Exception as e:
        print(f"[DEBUG] Read error: {e}", file=sys.stderr)
        pass
    
    return lines

def serial_reader_worker():
    """Background thread that reads from serial port."""
    global serial_conn, serial_reader_running, serial_message_queue
    
    while serial_reader_running:
        if serial_conn and serial_conn.is_open:
            try:
                lines = read_available_lines(serial_conn)
                for line in lines:
                    if line:
                        serial_message_queue.put({
                            'type': 'serial',
                            'message': line,
                            'timestamp': time.time()
                        })
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
        serial_reader_thread = threading.Thread(target=serial_reader_worker, daemon=True)
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

@app.route('/uart/connect', methods=['POST'])
def uart_connect():
    """Connect to ESP32 via UART."""
    global serial_conn
    
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
        
        serial_conn = open_serial(port, UART_BAUD_RATE)
        time.sleep(0.1)
        start_serial_reader()
        
        return jsonify({
            'status': 'ok',
            'message': f'Connected to {port}',
            'port': port,
            'baudrate': UART_BAUD_RATE
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/uart/disconnect', methods=['POST'])
def uart_disconnect():
    """Disconnect from ESP32."""
    global serial_conn
    
    stop_serial_reader()
    
    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        serial_conn = None
    
    return jsonify({'status': 'ok', 'message': 'Disconnected'})

@app.route('/uart/stream')
def uart_stream():
    """Server-Sent Events stream for serial messages."""
    def generate():
        while True:
            try:
                try:
                    msg = serial_message_queue.get(timeout=1.0)
                    message = msg.get('message', '')
                    if message and len(message.strip()) > 0:
                        data = f"data: {message}\n\n"
                        yield data
                except queue.Empty:
                    yield ": heartbeat\n\n"
            except GeneratorExit:
                break
            except Exception as e:
                print(f"[SSE] Error: {e}", file=sys.stderr)
                break
    
    response = Response(stream_with_context(generate()), mimetype='text/event-stream')
    response.headers['Cache-Control'] = 'no-cache'
    return response

@app.route('/health')
def health():
    """Health check endpoint."""
    global serial_conn, serial_reader_running
    return jsonify({
        'status': 'ok',
        'message': 'Dashboard server is running',
        'uart_connected': serial_conn is not None and serial_conn.is_open if serial_conn else False,
        'serial_available': SERIAL_AVAILABLE,
        'serial_reader_running': serial_reader_running
    })

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
    
    parser = argparse.ArgumentParser(description='ESP32 Car Control Dashboard Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to (default: 0.0.0.0 for all interfaces)')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to (default: 5000)')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()
    
    local_ip = get_local_ip()
    
    print("=" * 60)
    print("ESP32 Car Control Dashboard Server")
    print("=" * 60)
    print(f"Dashboard available at:")
    print(f"  Local:   http://127.0.0.1:{args.port}")
    print(f"  Network: http://{local_ip}:{args.port}")
    print("=" * 60)
    print("Press Ctrl+C to stop the server")
    print("=" * 60)
    
    app.run(host=args.host, port=args.port, debug=args.debug)

