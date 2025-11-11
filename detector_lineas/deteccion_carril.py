import time
import sys
import argparse

import cv2
import numpy as np
import math

from config import choose_camera_by_OS

# UART imports (optional)
try:
    import serial
    from serial.tools import list_ports
    UART_AVAILABLE = True
except ImportError:
    UART_AVAILABLE = False
    print("Warning: pyserial not available. UART control disabled.")

# ===== STUBS PARA FUNCIONES FALTANTES (para que funcione sin el sistema completo) =====
def get_new_votes_logic():
    return NEW_VOTES_LOGIC_ENABLED

def set_new_votes_logic(value):
    global NEW_VOTES_LOGIC_ENABLED
    NEW_VOTES_LOGIC_ENABLED = value

# Mock para queue_list (no se usa en la demo)
class MockQueue:
    def put(self, item):
        pass  # No hacer nada en la demo

class MockQueueList(dict):
    def __init__(self):
        super().__init__()
        self['Warning'] = MockQueue()

# LANES
PID_TOLERANCE = 50
PID_KP = 0.075
PROP_CONSTANT = 1.2

THRESHOLD = 165
KERNEL = 3
ROI = 35

PID_KI = 0.05
PID_KD = 0.05

# THRESHOLD = 100
# KERNEL = 9
# ROI = 30
NECESSARY_VOTES = 50

NEW_VOTES_LOGIC_ENABLED = False

IS_ABLE_TO_PARK = False


class PIDController:
    def __init__(self, Kp, Ki, Kd, tolerancia):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.iteration_count = 0
        self.integral_reset_interval = 10
        self.tolerancia = tolerancia

    def compute(self, error, dt, integral_reset_interval=None):
        if integral_reset_interval is not None:
            self.integral_reset_interval = integral_reset_interval

        proportional = error
        self.integral = self.integral + error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        self.iteration_count = self.iteration_count + 1

        # Reiniciar el término integral si se alcanza el intervalo deseado
        if self.iteration_count % self.integral_reset_interval == 0:
            self.integral = 0

        if abs(error) < self.tolerancia:
            control_signal = -3
        else:
            control_signal = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
            control_signal = max(min(control_signal, 22), -22)
        return control_signal


class MarcosLaneDetector:
    def __init__(self, queue_list):
        self.consecutive_frames_without_left_line = 0
        self.consecutive_frames_without_right_line = 0
        self.consecutive_frames_with_horizontal_line = 0
        self.dt = 0.2
        self.kp = PID_KP
        self.ki = PID_KI
        self.kd = PID_KD
        self.tolerancia = PID_TOLERANCE
        self.allowed_frames = 4
        self.prev_steering_angle = 0
        self.prev_horizontal_line = 0
        self.pid_controller = PIDController(self.kp, self.ki, self.kd, self.tolerancia)
        self.threshold_value = THRESHOLD
        self.kernel_value = KERNEL
        self.ROI_value = ROI / 100
        self.necessary_votes = NECESSARY_VOTES
        self.queue_list = queue_list
        self.consecutive_single_right_lines = 0
        self.consecutive_single_left_lines = 0
        self.just_seen_two_lines = False
        self.lowered_speed = False
        self.should_decrease_votes = False
        self.first_time_in_votes_logic = True
        self.new_votes_logic_start_time = 0

    def follow_left_line(self, line):
        x1, y1, x2, y2 = line[0]
        dx = x2 - x1
        dy = y2 - y1
        if dx != 0:
            pendiente = math.degrees(abs(dy / dx))

            steering_angle = self.slope_mapper(pendiente)
        return steering_angle

    def follow_right_line(self, line):
        x1, y1, x2, y2 = line[0]
        dx = x2 - x1
        dy = y2 - y1
        if dx != 0:
            pendiente = math.degrees(abs(dy / dx))
            steering_angle = self.slope_mapper(pendiente) * -1
        return steering_angle

    def control_signal(self, error):
        steering_angle = self.pid_controller.compute(error)
        return round(steering_angle)

    # ------------------------------------------------------------------------------------------

    def signal_detector(self):
        signal_detected = 1
        return signal_detected

    def follow_mid_line(self, image, average_horizontal_line, height, width):
        x1, y1, x2, y2 = average_horizontal_line[0]
        mid_point_horizontal_line_x = (x1 + x2) // 2
        mid_point_horizontal_line_y = (y1 + y2) // 2
        bottom_center_x = width // 2
        error = mid_point_horizontal_line_x - bottom_center_x
        steering_angle = self.control_signal(error)
        cv2.line(image, (mid_point_horizontal_line_x, mid_point_horizontal_line_y), (bottom_center_x, height),
                 (165, 0, 255), 2)
        return steering_angle

    def lower_speed(self):
        self.lowered_speed = True
        # En la demo, solo imprimir (no hay sistema de mensajes)
        print("Velocidad reducida")
        self.queue_list['Warning'].put({
            "msgValue": "LOW_SPEED"
        })

    def increase_speed(self):
        self.lowered_speed = False
        # En la demo, solo imprimir (no hay sistema de mensajes)
        print("Velocidad normal")
        self.queue_list['Warning'].put({
            "msgValue": "BASE_SPEED"
        })

    def get_steering_angle(self, image, repetition=1):
        average_left_line, average_right_line, height, width, canny_image = self.image_processing(image)

        if average_left_line is not None and average_right_line is not None:
            if self.lowered_speed and self.just_seen_two_lines:
                self.increase_speed()

            if self.just_seen_two_lines:
                error = self.getting_error(image, average_left_line, average_right_line, height, width)
                self.just_seen_two_lines = False
                self.consecutive_single_right_lines = 0
                self.consecutive_single_left_lines = 0
                steering_angle = self.control_signal(error)
            else:
                self.just_seen_two_lines = True
                steering_angle = self.prev_steering_angle
        elif average_left_line is not None:
            if self.consecutive_single_left_lines == 2:
                if not self.lowered_speed:
                    self.lower_speed()

            if self.consecutive_single_left_lines == 2:
                steering_angle = 22
            else:
                steering_angle = self.follow_left_line(average_left_line)
                self.consecutive_single_left_lines = self.consecutive_single_left_lines + 1
        elif average_right_line is not None:
            self.should_decrease_votes = True
            if self.consecutive_single_right_lines == 2:
                if not self.lowered_speed:
                    self.lower_speed()

            if self.consecutive_single_right_lines == 2:
                steering_angle = -22
            else:
                steering_angle = self.follow_right_line(average_right_line)
                self.consecutive_single_right_lines = self.consecutive_single_right_lines + 1
        else:
            if repetition == 2:
                self.kernel_value = KERNEL
                self.threshold_value = THRESHOLD
                self.necessary_votes = NECESSARY_VOTES
                return self.prev_steering_angle

            self.kernel_value = 3
            self.threshold_value = 90
            steering_angle = self.prev_steering_angle
            return self.get_steering_angle(image, repetition=2)
        self.kernel_value = KERNEL
        self.threshold_value = THRESHOLD
        if get_new_votes_logic():
            if self.first_time_in_votes_logic:
                print("SETTING START TIME **********")
                self.new_votes_logic_start_time = time.time()
                self.first_time_in_votes_logic = False

            if time.time() - self.new_votes_logic_start_time > 90:
                print("TURING OFF NEW LOGIC **********")
                set_new_votes_logic(False)

            self.necessary_votes = 33
            print("DECREASING VOTES TO", self.necessary_votes)
            self.should_decrease_votes = False

        else:
            self.necessary_votes = NECESSARY_VOTES

        self.prev_steering_angle = steering_angle
        return steering_angle

    def is_detecting_both_lines(self, average_left_line, average_right_line):
        return average_left_line is not None and average_right_line is not None

    def control_signal(self, error):
        steering_angle = self.pid_controller.compute(error, self.dt)
        return round(steering_angle)

    def lines_classifier(self, lines):
        left_lines = []
        right_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0:
                    slope = np.pi / 2
                else:
                    slope = np.arctan((y2 - y1) / (x2 - x1))

                angle_degrees = np.degrees(abs(slope))
                if angle_degrees > 30 or (angle_degrees < 155 and angle_degrees > 180):
                    if slope < 0:
                        left_lines.append(line)
                    else:
                        right_lines.append(line)
                else:
                    continue
        return left_lines, right_lines

    def average_lines(self, lines):
        if len(lines) > 0:
            lines_array = np.array(lines)
            average_line = np.mean(lines_array, axis=0, dtype=np.int32)
            return average_line
        else:
            return None

    def get_intersection_point(self, line, y):
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)
        if slope == 0:
            return x1, y
        else:
            x = int(x1 + (y - y1) / slope)
            return x, y

    def line_drawing(self, image, line, height):
        x1, y1, x2, y2 = line[0]
        extended_line = np.array([
            self.get_intersection_point(line, 0),
            self.get_intersection_point(line, height)
        ], dtype=np.int32)
        cv2.line(image, (extended_line[0][0], extended_line[0][1]), (extended_line[1][0], extended_line[1][1]),
                 (0, 0, 255),
                 2)

    def lane_detection(self, left_lines, right_lines):
        # Si se detectaron líneas tanto a la izquierda como a la derecha
        if len(left_lines) > 0 and len(right_lines) > 0:
            self.consecutive_frames_without_left_line = 0
            self.consecutive_frames_without_right_line = 0
        # Si solo se detectaron líneas a la izquierda
        elif len(left_lines) > 0:
            self.consecutive_frames_without_left_line = 0
            self.consecutive_frames_without_right_line = self.consecutive_frames_without_right_line + 1
            print("Consecutive frames without right line:", self.consecutive_frames_without_right_line)
        # Si solo se detectaron líneas a la derecha
        elif len(right_lines) > 0:
            self.consecutive_frames_without_left_line = self.consecutive_frames_without_left_line + 1
            self.consecutive_frames_without_right_line = 0
            print("Consecutive frames without left line:", self.consecutive_frames_without_left_line)
        # Si no se detectaron líneas en ninguno de los lados
        else:
            self.consecutive_frames_without_left_line = self.consecutive_frames_without_left_line + 1
            self.consecutive_frames_without_right_line = self.consecutive_frames_without_right_line + 1

    def getting_error(self, image, average_left_line, average_right_line, height, width):
        x1_left, y1_left, x2_left, y2_left = average_left_line[0]
        x1_right, y1_right, x2_right, y2_right = average_right_line[0]

        # Calcular los puntos donde las líneas promedio izquierda y derecha intersectan el borde inferior de la imagen
        a = (y2_left - y1_left)
        if y2_left - y1_left == 0:
            a = 0.01
        bottom_left_x = int(x1_left + (height - y1_left) * (x2_left - x1_left) / a)

        b = (y2_right - y1_right)
        if y2_right - y1_right == 0:
            b = 0.01
        bottom_right_x = int(x1_right + (height - y1_right) * (x2_right - x1_right) / b)

        # Calcular el punto medio entre estos puntos
        midpoint_x = (bottom_left_x + bottom_right_x) // 2
        midpoint_y = height

        # Calcular la intersección de las líneas promedio izquierda y derecha
        c = (x2_left - x1_left)
        if x2_left - x1_left == 0:
            c = 0.01
        slope_left = (y2_left - y1_left) / c

        d = (x2_right - x1_right)
        if x2_right - x1_right == 0:
            d = 0.01
        slope_right = (y2_right - y1_right) / d

        e = (slope_left - slope_right)
        if slope_left - slope_right == 0:
            e = 0.01
        intersection_x = int(
            (y1_right - y1_left + slope_left * x1_left - slope_right * x1_right) / e)
        intersection_y = int(slope_left * (intersection_x - x1_left) + y1_left)

        cv2.circle(image, (intersection_x, intersection_y), 5, (255, 0, 255), -1)

        cv2.circle(image, (midpoint_x, midpoint_y), 5, (0, 255, 255), -1)

        cv2.line(image, (intersection_x, intersection_y), (midpoint_x, midpoint_y), (0, 165, 255), 2)

        bottom_center_x = width // 2
        bottom_center_y = height
        cv2.circle(image, (bottom_center_x, bottom_center_y), 5, (0, 255, 0), -1)
        error = midpoint_x - bottom_center_x
        return error

    def merge_lines(self, lines):
        merged_lines = []
        for line in lines:
            if len(merged_lines) == 0:
                merged_lines.append(line)
            else:
                # Comprobar si la línea actual está lo suficientemente cerca de alguna de las líneas fusionadas
                merge_flag = False
                for i, merged_line in enumerate(merged_lines):
                    if abs(line[0][0] - merged_line[0][
                        2]) < 175:  # Cambiar el valor según la distancia de fusión deseada
                        merged_lines[i] = np.array([[merged_line[0][0], merged_line[0][1], line[0][2], line[0][3]]])
                        merge_flag = True
                        break
                if not merge_flag:
                    merged_lines.append(line)
        return merged_lines

    def slope_mapper(self, angle):
        to_return = 0
        if angle > 50:
            to_return = 3
        elif angle > 40:
            to_return = 11
        elif angle > 0:
            to_return = 22
        elif angle > -40:
            to_return = -22
        elif angle > -50:
            to_return = -11
        else:
            to_return = -3
        # print('### MAPEADO', to_return)
        return to_return

    def conditioning(self, frame, gauss_size, deviation):
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grey[:117, :] = 140
        avg_brightness = np.mean(grey)
        if avg_brightness < 129:
            frame = cv2.convertScaleAbs(frame, 1000, 0.52)
        elif avg_brightness > 128:
            frame = cv2.convertScaleAbs(frame, 1000, 3)
        conditioned = cv2.GaussianBlur(frame, (gauss_size, gauss_size), deviation)
        return conditioned

    def image_processing(self, image):

        if self.kernel_value % 2 == 0:
            self.kernel_value = self.kernel_value + 1

        height, width = image.shape[:2]

        x1 = 0
        y1 = int(self.ROI_value * height)
        x2 = width
        y2 = (height)

        roi_vertices = np.array([[(x1, y1), (x2, y1), (x2, y2), (x1, y2)]], dtype=np.int32)
        # image = self.conditioning(image, 5, 0.9)
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, roi_vertices, (255, 255, 255))
        masked_image = cv2.bitwise_and(image, mask)

        grey_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(grey_image, self.threshold_value, 255, cv2.THRESH_BINARY)
        noiseless_image = cv2.medianBlur(binary_image, self.kernel_value)
        canny_image = cv2.Canny(noiseless_image, 100, 150)

        lines = cv2.HoughLinesP(canny_image, 1, np.pi / 180, self.necessary_votes, minLineLength=50, maxLineGap=150)

        left_lines, right_lines = self.lines_classifier(lines)

        merged_left_lines = self.merge_lines(left_lines)
        merged_right_lines = self.merge_lines(right_lines)

        average_left_line = self.average_lines(merged_left_lines)
        if average_left_line is not None:
            self.line_drawing(image, average_left_line, height=height)

        average_right_line = self.average_lines(merged_right_lines)
        if average_right_line is not None:
            self.line_drawing(image, average_right_line, height=height)

        return average_left_line, average_right_line, height, width, canny_image

    def plan_c(self, canny, width, height):
        slope = 70
        for y in np.linspace(0, int(height * 0.35) - 2, int(height * 0.35) - 1):
            for x in np.linspace(0, int(width) - 2, int(width) - 1):
                x = int(x)
                y = int(y)
                value = canny[y][x]
                if value == 255:
                    a = y - height
                    if a == 0:
                        a = 0.01

                    slope = np.abs((x - (width / 2)) / a)
                    x = width - 1
                    y = height * 0.35 - 1

        slope = math.degrees(slope)
        steering_angle = self.slope_mapper(slope)
        return steering_angle


# ===== FUNCIONES UART =====
def degrees_to_servo(degrees: float, max_degrees: float = 22.0) -> int:
    """
    Convierte grados de lane detector a valor de servo.
    
    Args:
        degrees: Ángulo en grados (-max_degrees a +max_degrees)
        max_degrees: Máximo ángulo permitido (default: 22°)
    
    Returns:
        Valor de servo (50-135)
    """
    SERVO_CENTER = 105
    SERVO_RANGE = 85  # 135 - 50
    
    # Normalizar a -1.0 a +1.0
    normalized = degrees / max_degrees
    # Limitar al rango [-1, 1]
    normalized = max(-1.0, min(1.0, normalized))
    # Convertir a valor de servo
    # NOTA: La convención está invertida - cuando degrees > 0 (derecha en detector),
    # el servo necesita un valor menor a 105 (izquierda en servo) para que el carro vaya a la derecha
    # Por lo tanto invertimos el signo
    servo_value = SERVO_CENTER - (normalized * (SERVO_RANGE / 2))
    return int(round(servo_value))


def open_serial(port: str, baud: int = 115200):
    """Open serial port with appropriate settings."""
    if not UART_AVAILABLE:
        raise ImportError("pyserial not available")
    
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
    # Clear any pending data
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def write_line(ser: serial.Serial, msg: str) -> None:
    """Write a line to serial port with error handling."""
    if not ser.is_open:
        raise serial.SerialException("Serial port is not open")
    line = (msg.strip() + "\n").encode("utf-8")
    try:
        ser.write(line)
        ser.flush()
    except serial.SerialTimeoutException:
        print("Warning: Write timeout - command may not have been sent", file=sys.stderr)
    except serial.SerialException as e:
        print(f"Error writing to serial: {e}", file=sys.stderr)
        raise


def select_port_interactive() -> str:
    """Select serial port interactively."""
    if not UART_AVAILABLE:
        raise ImportError("pyserial not available")
    
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Connect the ESP32 and try again.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx}] {p.device} - {p.description}")
    while True:
        sel = input("Select port index: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Try again.")


# ===== BUCLE PRINCIPAL PARA CÁMARA =====
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lane detection with UART control")
    parser.add_argument("--uart-port", type=str, help="Serial port for UART control (e.g., /dev/ttyUSB0, COM5). If omitted, UART is disabled.")
    parser.add_argument("--uart-baud", type=int, default=115200, help="Baud rate for UART (default: 115200)")
    parser.add_argument("--uart-interactive", action="store_true", help="Select UART port interactively")
    parser.add_argument("--speed", type=int, default=180, help="Speed value (180-255) to send via UART. Minimum working speed is 180 (default: 180)")
    parser.add_argument("--camera", type=int, default=None, help="Camera index (default: auto-select via config)")
    parser.add_argument("--arm-system", action="store_true", help="Arm the system before starting (requires --uart-port)")
    parser.add_argument("--mode", choices=["manual", "auto"], default="manual", help="System mode: manual or auto (default: manual)")
    args = parser.parse_args()
    
    # Validar velocidad mínima
    MIN_SPEED = 180
    if args.speed < MIN_SPEED:
        print(f"Warning: Speed {args.speed} is below minimum working speed ({MIN_SPEED}). Setting to {MIN_SPEED}.")
        args.speed = MIN_SPEED
    if args.speed > 255:
        print(f"Warning: Speed {args.speed} exceeds maximum (255). Setting to 255.")
        args.speed = 255
    
    # UART setup
    ser = None
    use_uart = args.uart_port is not None or args.uart_interactive
    
    if use_uart:
        if not UART_AVAILABLE:
            print("Error: pyserial not available. Install with: pip install pyserial")
            sys.exit(1)
        
        try:
            if args.uart_interactive:
                port = select_port_interactive()
            else:
                port = args.uart_port
            
            print(f"Opening UART port: {port} at {args.uart_baud} baud...")
            ser = open_serial(port, args.uart_baud)
            print("UART connection established!")
            
            # Arm system and set mode if requested
            if args.arm_system:
                mode_value = 0 if args.mode == "manual" else 1
                print(f"Setting system mode to {args.mode}...")
                write_line(ser, f"M:SYS_MODE:{mode_value}")
                time.sleep(0.1)
                
                print("Arming system...")
                write_line(ser, "M:SYS_ARM:0")
                time.sleep(0.2)
                print("System armed!")
        except Exception as e:
            print(f"Error opening UART port: {e}")
            print("Continuing without UART control...")
            ser = None
            use_uart = False
    
    # Abrir cámara
    if args.camera is not None:
        camera_path = args.camera
    else:
        camera_path = choose_camera_by_OS()
    
    cap = cv2.VideoCapture(4)
    
    if not cap.isOpened():
        print(f"Error: No se pudo abrir la cámara (ruta: {camera_path})")
        if ser and ser.is_open:
            ser.close()
        sys.exit(1)
    
    # Resolución deseada para el procesamiento
    TARGET_WIDTH = 640
    TARGET_HEIGHT = 480
    
    # Configurar resolución de la cámara
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
    
    # Obtener la resolución real de la cámara
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Delay para mostrar frames (aproximadamente 30 FPS)
    frame_delay = 33
    
    print(f"Cámara abierta (ruta: {camera_path})")
    print(f"Resolución de la cámara: {actual_width}x{actual_height}")
    print(f"Resolución de procesamiento: {TARGET_WIDTH}x{TARGET_HEIGHT}")
    print(f"FPS: {fps:.2f}")
    if use_uart:
        print(f"UART control: ENABLED (speed: {args.speed})")
    else:
        print("UART control: DISABLED")
    print("Presiona 'q' para salir")
    
    # Inicializar detector
    queue_list = MockQueueList()
    detector = MarcosLaneDetector(queue_list)
    
    # Crear ventanas
    cv2.namedWindow('Detección de Carriles - Marcos', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Canny - Detección de Bordes', cv2.WINDOW_NORMAL)
    
    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            print("Error al leer frame de la cámara")
            break
        
        try:
            # Redimensionar el frame a la resolución deseada si es necesario
            if frame.shape[1] != TARGET_WIDTH or frame.shape[0] != TARGET_HEIGHT:
                frame = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT))
            
            # Calcular steering angle (esto procesa la imagen y dibuja las líneas)
            # La imagen ya está modificada con las líneas dibujadas
            steering_angle = detector.get_steering_angle(frame)
            
            # Enviar comandos UART si está habilitado
            if use_uart and ser and ser.is_open:
                try:
                    # Convertir ángulo de grados a valor de servo
                    servo_value = degrees_to_servo(steering_angle, max_degrees=22.0)
                    
                    # Enviar comando de dirección
                    write_line(ser, f"C:SET_STEER:{servo_value}")
                    
                    # Enviar comando de velocidad (mantener constante)
                    # Solo enviar velocidad si cambió o cada N frames para reducir carga
                    write_line(ser, f"C:SET_SPEED:{args.speed}")
                except Exception as e:
                    print(f"Error sending UART command: {e}")
            
            # Obtener el canny_image procesando una copia (para no modificar frame dos veces)
            frame_copy = frame.copy()
            _, _, _, _, canny_image = detector.image_processing(frame_copy)
            
            # Mostrar información en la imagen
            resolution_text = f"Resolucion: {TARGET_WIDTH}x{TARGET_HEIGHT}"
            cv2.putText(frame, resolution_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            info_text = f"Steering Angle: {steering_angle}°"
            cv2.putText(frame, info_text, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Mostrar información UART si está habilitado
            if use_uart and ser and ser.is_open:
                servo_value = degrees_to_servo(steering_angle, max_degrees=22.0)
                uart_text = f"UART: SERVO={servo_value}, SPEED={args.speed}"
                cv2.putText(frame, uart_text, (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Mostrar dirección
            if steering_angle > 0:
                direction = f"GIRAR DERECHA"
                color = (0, 165, 255)
            elif steering_angle < 0:
                direction = f"GIRAR IZQUIERDA"
                color = (255, 0, 255)
            else:
                direction = "RECTO"
                color = (0, 255, 0)
            
            cv2.putText(frame, direction, (10, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            
            # Convertir canny a BGR para mostrar
            canny_display = cv2.cvtColor(canny_image, cv2.COLOR_GRAY2BGR)
            
            # Mostrar ventanas
            cv2.imshow('Detección de Carriles - Marcos', frame)
            cv2.imshow('Canny - Detección de Bordes', canny_display)
            
        except Exception as e:
            print(f"Error procesando frame: {e}")
            continue
        
        # Control de salida
        key = cv2.waitKey(frame_delay) & 0xFF
        
        if key == ord('q'):
            break
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    if ser and ser.is_open:
        # Send emergency stop before closing
        try:
            write_line(ser, "E:BRAKE_NOW:0")
            time.sleep(0.1)
        except:
            pass
        ser.close()
        print("UART connection closed")
    
    print("Cámara cerrada")
    