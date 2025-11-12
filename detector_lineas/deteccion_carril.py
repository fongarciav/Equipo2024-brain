import time
import sys
import threading
from typing import Optional, Callable

import cv2
import numpy as np
import math

from config import choose_camera_by_OS

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
PID_TOLERANCE = 60
PID_KP = 0.02
PROP_CONSTANT = 1.2

THRESHOLD = 170
KERNEL = 3
ROI = 25

PID_KI = 0.001
PID_KD = 0.001

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
        self.should_decrease_votes = False
        self.first_time_in_votes_logic = True
        self.new_votes_logic_start_time = 0
        self._pid_lock = threading.Lock()
    
    def update_pid_parameters(self, kp=None, ki=None, kd=None, tolerance=None):
        """
        Update PID parameters in real-time.
        
        Args:
            kp: New Kp value (optional)
            ki: New Ki value (optional)
            kd: New Kd value (optional)
            tolerance: New tolerance value (optional)
        """
        with self._pid_lock:
            if kp is not None:
                self.kp = kp
            if ki is not None:
                self.ki = ki
            if kd is not None:
                self.kd = kd
            if tolerance is not None:
                self.tolerancia = tolerance
            
            # Recreate PID controller with new parameters
            self.pid_controller = PIDController(self.kp, self.ki, self.kd, self.tolerancia)
    
    def get_pid_parameters(self):
        """
        Get current PID parameters.
        
        Returns:
            dict with kp, ki, kd, tolerance
        """
        with self._pid_lock:
            return {
                'kp': self.kp,
                'ki': self.ki,
                'kd': self.kd,
                'tolerance': self.tolerancia
            }

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

    def get_steering_angle(self, image, repetition=1):
        average_left_line, average_right_line, height, width, canny_image = self.image_processing(image)

        if average_left_line is not None and average_right_line is not None:
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
            print(f"[STEERING] ONLY LEFT line detected -> steering={self.follow_left_line(average_left_line) if self.consecutive_single_left_lines < 2 else 22}")
            if self.consecutive_single_left_lines == 2:
                steering_angle = 22
            else:
                steering_angle = self.follow_left_line(average_left_line)
                self.consecutive_single_left_lines = self.consecutive_single_left_lines + 1
        elif average_right_line is not None:
            print(f"[STEERING] ONLY RIGHT line detected -> steering={self.follow_right_line(average_right_line) if self.consecutive_single_right_lines < 2 else -22}")
            self.should_decrease_votes = True
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

    def lines_classifier(self, lines, image_width=None, image_height=None):
        """
        Classify lines as left or right based on their position at the bottom of the image.
        
        Args:
            lines: Detected lines from HoughLinesP
            image_width: Width of the image (required for position-based classification)
            image_height: Height of the image (required for finding lowest point)
        
        Returns:
            left_lines, right_lines: Lists of classified lines
        """
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
                
                # Filter lines by angle (same threshold as before)
                if angle_degrees > 30 or (angle_degrees < 155 and angle_degrees > 180):
                    # NEW LOGIC: Classify based on position at the bottom of the image
                    if image_width is not None and image_height is not None:
                        # Calculate where the line intersects the bottom of the image (highest y value)
                        # Using line equation: y = mx + b, solve for x when y = image_height
                        if x2 - x1 != 0:
                            line_slope = (y2 - y1) / (x2 - x1)
                            # y - y1 = m(x - x1) => x = (y - y1)/m + x1
                            if line_slope != 0:
                                bottom_x = int((image_height - y1) / line_slope + x1)
                            else:
                                # Horizontal line, use x1
                                bottom_x = x1
                        else:
                            # Vertical line, use x1
                            bottom_x = x1
                        
                        # Classify based on which half of the screen the bottom point is in
                        screen_center = image_width / 2
                        if bottom_x < screen_center:
                            left_lines.append(line)
                            print(f"[LINE] LEFT - bottom_x: {bottom_x} (center: {screen_center})")
                        else:
                            right_lines.append(line)
                            print(f"[LINE] RIGHT - bottom_x: {bottom_x} (center: {screen_center})")
                    else:
                        # Fallback to old slope-based method if dimensions not provided
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

        left_lines, right_lines = self.lines_classifier(lines, image_width=width, image_height=height)

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


# ===== FUNCIÓN PARA EJECUTAR DETECCIÓN CON CALLBACK =====
def run_lane_detection(
    camera_path,
    steering_callback: Optional[Callable[[float], None]] = None,
    show_display: bool = True,
    target_width: int = 640,
    target_height: int = 480,
    web_streamer=None,
    signal_detector=None
):
    """
    Ejecuta la detección de carriles con callback para eventos de dirección.
    
    Args:
        camera_path: Ruta o índice de la cámara
        steering_callback: Función callback que recibe el ángulo de dirección en grados
        show_display: Si True, muestra las ventanas de visualización
        target_width: Ancho objetivo para procesamiento
        target_height: Alto objetivo para procesamiento
        web_streamer: Optional WebStreamer instance for web streaming
        signal_detector: Optional SignalDetector instance for traffic signal detection
    
    Returns:
        None (ejecuta hasta que se presiona 'q')
    """
    cap = cv2.VideoCapture(camera_path)
    
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara (ruta: {camera_path})")
    
    # Configurar resolución de la cámara
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
    
    # Obtener la resolución real de la cámara
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Delay para mostrar frames (aproximadamente 30 FPS)
    frame_delay = 33
    
    # Si web_streamer está habilitado, deshabilitar display automáticamente
    if web_streamer:
        show_display = False
    
    print(f"Cámara abierta (ruta: {camera_path})")
    print(f"Resolución de la cámara: {actual_width}x{actual_height}")
    print(f"Resolución de procesamiento: {target_width}x{target_height}")
    print(f"FPS: {fps:.2f}")
    if steering_callback:
        print("Steering callback: ENABLED")
    else:
        print("Steering callback: DISABLED")
    if web_streamer:
        print("Web streaming: ENABLED (display windows disabled)")
    elif show_display:
        print("Display windows: ENABLED (Presiona 'q' para salir)")
    else:
        print("Display windows: DISABLED")
    
    # Inicializar detector
    queue_list = MockQueueList()
    detector = MarcosLaneDetector(queue_list)
    
    # Pasar referencia del detector al web_streamer si está disponible
    if web_streamer:
        web_streamer.detector = detector
    
    # Crear ventanas si se requiere visualización (y no hay web streaming)
    if show_display and not web_streamer:
        cv2.namedWindow('Detección de Carriles - Marcos', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Canny - Detección de Bordes', cv2.WINDOW_NORMAL)
    
    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            print("Error al leer frame de la cámara")
            break
        
        try:
            # Redimensionar el frame a la resolución deseada si es necesario
            if frame.shape[1] != target_width or frame.shape[0] != target_height:
                frame = cv2.resize(frame, (target_width, target_height))
            
            # Calcular steering angle (esto procesa la imagen y dibuja las líneas)
            # La imagen ya está modificada con las líneas dibujadas
            steering_angle = detector.get_steering_angle(frame)
            
            # Llamar callback si está disponible
            if steering_callback:
                try:
                    steering_callback(steering_angle)
                except Exception as e:
                    print(f"Error en steering callback: {e}")
            
            # Obtener el canny_image procesando una copia (para no modificar frame dos veces)
            frame_copy = frame.copy()
            _, _, _, _, canny_image = detector.image_processing(frame_copy)
            
            # Preparar frame con información para visualización
            display_frame = frame.copy()
            
            # Detectar señales si el detector está disponible
            signal_detections = []
            if signal_detector and signal_detector.is_available():
                signal_detections = signal_detector.detect(display_frame)
                if signal_detections:
                    display_frame = signal_detector.draw_detections(display_frame, signal_detections)
                    # Mostrar información de la señal más confiable
                    most_confident = signal_detector.get_most_confident(signal_detections)
                    if most_confident:
                        signal_text = f"Signal: {most_confident['class']} ({most_confident['confidence']:.2f})"
                        cv2.putText(display_frame, signal_text, (10, 130), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            resolution_text = f"Resolucion: {target_width}x{target_height}"
            cv2.putText(display_frame, resolution_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            info_text = f"Steering Angle: {steering_angle}°"
            cv2.putText(display_frame, info_text, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
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
            
            cv2.putText(display_frame, direction, (10, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            
            # Actualizar web streamer si está disponible
            if web_streamer:
                canny_display = cv2.cvtColor(canny_image, cv2.COLOR_GRAY2BGR)
                web_streamer.update_frame(display_frame, canny_display)
            
            # Mostrar ventanas solo si display está habilitado Y no hay web streaming
            if show_display and not web_streamer:
                # Convertir canny a BGR para mostrar
                canny_display = cv2.cvtColor(canny_image, cv2.COLOR_GRAY2BGR)
                
                # Mostrar ventanas
                cv2.imshow('Detección de Carriles - Marcos', display_frame)
                cv2.imshow('Canny - Detección de Bordes', canny_display)
            
        except Exception as e:
            print(f"Error procesando frame: {e}")
            continue
        
        # Control de salida
        if show_display and not web_streamer:
            key = cv2.waitKey(frame_delay) & 0xFF
            if key == ord('q'):
                break
        else:
            # Si no hay display o hay web streaming, usar pequeño delay para no saturar CPU
            # El web streaming maneja su propio ciclo de eventos
            time.sleep(frame_delay / 1000.0)
            
            # Verificar si el web streamer ha sido detenido
            if web_streamer and not web_streamer.is_running():
                break
    
    # Cleanup
    cap.release()
    if show_display and not web_streamer:
        cv2.destroyAllWindows()
    if web_streamer:
        web_streamer.stop()
    
    print("Cámara cerrada")


# ===== BUCLE PRINCIPAL PARA DEMO (sin UART) =====
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Lane detection (demo mode - no UART)")
    parser.add_argument("--camera", type=int, default=None, help="Camera index (default: auto-select via config)")
    parser.add_argument("--no-display", action="store_true", help="Disable display windows")
    parser.add_argument("--web-stream", action="store_true", help="Enable web streaming (accessible via browser)")
    parser.add_argument("--web-port", type=int, default=5000, help="Port for web streaming (default: 5000)")
    parser.add_argument("--signal-detection", action="store_true", help="Enable traffic signal detection (YOLO)")
    parser.add_argument("--signal-model", type=str, default="../detector_señales/weights/best.engine", help="Path to model file (.engine/.trt for TensorRT, .pt for YOLO)")
    parser.add_argument("--signal-conf", type=float, default=0.5, help="Confidence threshold for signal detection (default: 0.5)")
    args = parser.parse_args()
    
    # Seleccionar cámara
    if args.camera is not None:
        camera_path = args.camera
    else:
        camera_path = choose_camera_by_OS()
    
    # Inicializar web streamer si está habilitado
    web_streamer = None
    if args.web_stream:
        try:
            from web_streamer import WebStreamer
            web_streamer = WebStreamer(port=args.web_port)
            web_streamer.start()
        except ImportError:
            print("Error: Flask not installed. Install with: pip install flask")
            print("Web streaming disabled.")
            args.web_stream = False
    
    # Inicializar detector de señales si está habilitado
    signal_detector = None
    if args.signal_detection:
        try:
            from signal_detector import SignalDetector
            import os
            # Resolve path relative to this script's directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            if os.path.isabs(args.signal_model):
                model_path = args.signal_model
            else:
                model_path = os.path.join(script_dir, args.signal_model)
            model_path = os.path.abspath(model_path)
            
            if not os.path.exists(model_path):
                print(f"Warning: Signal model not found at {model_path}")
                # Try fallback to .pt file if .engine not found
                if model_path.endswith('.engine'):
                    fallback_path = model_path.replace('.engine', '.pt')
                    if os.path.exists(fallback_path):
                        print(f"Using fallback model: {fallback_path}")
                        model_path = fallback_path
                    else:
                        print("Signal detection disabled.")
                        signal_detector = None
                        model_path = None
                else:
                    print("Signal detection disabled.")
                    signal_detector = None
                    model_path = None
            
            if model_path and os.path.exists(model_path):
                try:
                    signal_detector = SignalDetector(model_path, conf_threshold=args.signal_conf)
                    if signal_detector.is_available():
                        print(f"Signal detection: ENABLED (model: {model_path})")
                    else:
                        print("Signal detection: DISABLED (model failed to load)")
                        signal_detector = None
                except Exception as e:
                    print(f"Error loading signal detector: {e}")
                    signal_detector = None
        except ImportError as e:
            print(f"Error: Signal detector dependencies not installed: {e}")
            print("Install with: pip install ultralytics")
            print("Signal detection disabled.")
    
    # Ejecutar detección sin callback (solo visualización)
    run_lane_detection(
        camera_path=camera_path,
        steering_callback=None,
        show_display=not args.no_display,
        web_streamer=web_streamer,
        signal_detector=signal_detector
    )
    