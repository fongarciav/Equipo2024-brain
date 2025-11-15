import cv2
import numpy as np
import time
import math

# ======================================================================
# --- 1. TU CLASE PID CONTROLLER ORIGINAL (Sin Cambios) ---
# ======================================================================

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

        if self.iteration_count % self.integral_reset_interval == 0:
            self.integral = 0

        # La lógica de "ir recto"
        if abs(error) < self.tolerancia:
            # -3 es un valor muy cercano a 0, significa "ir recto"
            control_signal = -3 
        else:
            control_signal = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
            control_signal = max(min(control_signal, 22), -22) # Limitar el ángulo
        return control_signal

# ======================================================================
# --- 2. LA NUEVA CLASE DE DETECCIÓN (LA "MEZCLA") ---
# ======================================================================

class MarcosLaneDetector_Advanced:
    
    def __init__(self, pid_kp, pid_ki, pid_kd, pid_tolerance, threshold):
        # --- Parámetros de la lógica de tu NUEVO script ---
        self.LANE_WIDTH_PX = 500 # ¡CALIBRAR ESTE VALOR! Ancho del carril en píxeles en vista cenital
        self.prev_left_fit = None
        self.prev_right_fit = None
        self.MIN_POINTS_FOR_FIT = 3
        
        # --- Puntos de perspectiva (de tu nuevo script) ---
        # Puntos Origen (SRC) en la imagen original - roi
        self.tl = (0, 150)
        self.bl = (0, 440)
        self.tr = (640, 150)
        self.br = (640, 440)
        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        
        # Puntos Destino (DST) para la vista cenital
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        
        # Matrices de transformación
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        # --- Valores de color HSV (reemplazo de Trackbars) ---
        # ¡¡¡DEBES AJUSTAR ESTOS VALORES!!!
        # Usa tu script de trackbars por separado para encontrar los 
        # valores buenos y ponlos aquí.
        # El threshold controla el valor mínimo de brillo (V) en HSV
        self.hsv_lower = np.array([0, 0, threshold]) # L-H, L-S, L-V (threshold usado aquí)
        self.hsv_upper = np.array([255, 50, 255]) # U-H, U-S, U-V
        
        # --- Parámetros del PID (de tu script ORIGINAL) ---
        self.pid_controller = PIDController(pid_kp, pid_ki, pid_kd, pid_tolerance)
        self.last_time = time.time()

    def get_steering_angle(self, frame):
        
        # --- 1. Redimensionar y aplicar Vista Cenital (de tu nuevo script) ---
        frame = cv2.resize(frame, (640, 480))
        original_frame = frame.copy() # Guardar el original para el final
        
        transformed_frame = cv2.warpPerspective(frame, self.matrix, (640, 480))
        
        # --- 2. Detección de color (reemplazo de Trackbars) ---
        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_transformed_frame, self.hsv_lower, self.hsv_upper)
        
        # --- 3. Histograma y Sliding Windows (de tu nuevo script) ---
        # Usar toda la ventana del birdview para el histograma
        histogram = np.sum(mask[:, :], axis=0)
        midpoint = int(histogram.shape[0]/2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        
        y = 472
        lx, ly, rx, ry = [], [], [], []
        msk = mask.copy() # Copia para dibujar las ventanas

        while y > 0:
            # Ventana Izquierda
            img = mask[y-40:y, left_base-50:left_base+50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            found_left = False
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    lx.append(left_base-50 + cx)
                    ly.append(y-40 + cy)
                    left_base = left_base-50 + cx
                    found_left = True
                    break  # Solo tomar el primer contorno encontrado
            
            # Si no encontró nada, expandir la búsqueda en la ventana actual
            if not found_left and y > 40:
                # Buscar en una ventana más ancha
                search_width = 150
                img_expanded = mask[y-40:y, max(0, left_base-search_width//2):min(mask.shape[1], left_base+search_width//2)]
                contours_expanded, _ = cv2.findContours(img_expanded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours_expanded:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        lx.append(max(0, left_base-search_width//2) + cx)
                        ly.append(y-40 + cy)
                        left_base = max(0, left_base-search_width//2) + cx
                        found_left = True
                        break
            
            # Ventana Derecha
            img = mask[y-40:y, right_base-50:right_base+50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            found_right = False
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    rx.append(right_base-50 + cx)
                    ry.append(y-40 + cy)
                    right_base = right_base-50 + cx
                    found_right = True
                    break  # Solo tomar el primer contorno encontrado
            
            # Si no encontró nada, expandir la búsqueda en la ventana actual
            if not found_right and y > 40:
                # Buscar en una ventana más ancha
                search_width = 150
                img_expanded = mask[y-40:y, max(0, right_base-search_width//2):min(mask.shape[1], right_base+search_width//2)]
                contours_expanded, _ = cv2.findContours(img_expanded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours_expanded:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        rx.append(max(0, right_base-search_width//2) + cx)
                        ry.append(y-40 + cy)
                        right_base = max(0, right_base-search_width//2) + cx
                        found_right = True
                        break
                    
            cv2.rectangle(msk, (left_base-50,y), (left_base+50,y-40), (255,255,255), 2)
            cv2.rectangle(msk, (right_base-50,y), (right_base+50,y-40), (255,255,255), 2)
            y -= 40
            
        # --- 4. Polyfit y Lógica de Estimación (de tu nuevo script) ---
        left_fit_current = None
        right_fit_current = None
        
        if len(lx) >= self.MIN_POINTS_FOR_FIT:
            try: left_fit_current = np.polyfit(ly, lx, 2)
            except np.linalg.LinAlgError: pass

        if len(ry) >= self.MIN_POINTS_FOR_FIT:
            try: right_fit_current = np.polyfit(ry, rx, 2)
            except np.linalg.LinAlgError: pass

        # Decidir qué líneas usar (CASO 1-5)
        if left_fit_current is not None and right_fit_current is not None:
            left_fit = left_fit_current
            right_fit = right_fit_current
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
        elif left_fit_current is not None:
            left_fit = left_fit_current
            right_fit = left_fit + [0, 0, self.LANE_WIDTH_PX]
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
        elif right_fit_current is not None:
            right_fit = right_fit_current
            left_fit = right_fit - [0, 0, self.LANE_WIDTH_PX]
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
        elif self.prev_left_fit is not None:
            left_fit = self.prev_left_fit
            right_fit = self.prev_right_fit
        else:
            # No se ve nada y no hay memoria
            # Devolveremos la imagen original y un ángulo de 0
            debug_images = {
                "original": original_frame,
                "cenital": transformed_frame,
                "mask": mask,
                "sliding_windows": msk,
                "final_result": original_frame,
                "bird_view_lines": transformed_frame
            }
            return 0, debug_images
        
        # --- 5. CÁLCULO DEL ERROR (La "Mezcla" Clave) ---
        # Usamos el método de tu nuevo script para encontrar el centro del carril
        
        y_eval = 480 # Evaluar en la parte baja de la imagen
        left_x_bottom = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
        right_x_bottom = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
        
        lane_center = (left_x_bottom + right_x_bottom) / 2
        car_position = 320 # Centro del auto (ancho/2)
        
        # ¡AQUÍ ESTÁ LA MEZCLA!
        # Usamos el 'error' en píxeles de tu nuevo script
        # como 'input' para el 'PIDController' de tu script original.
        error = lane_center - car_position
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Usamos el PID de tu código original para calcular el ángulo
        steering_angle = self.pid_controller.compute(error, dt)

        # --- 6. Visualización (de tu nuevo script) ---
        # Vista aérea sin procesar (solo la transformación)
        bird_view_raw = transformed_frame.copy()
        
        # Vista aérea con líneas dibujadas (sin overlay verde)
        bird_view_with_lines = transformed_frame.copy()
        plot_y = np.linspace(0, 479, 480)
        plot_x_left = left_fit[0]*plot_y**2 + left_fit[1]*plot_y + left_fit[2]
        plot_x_right = right_fit[0]*plot_y**2 + right_fit[1]*plot_y + right_fit[2]
        
        # Dibujar las líneas del carril en la vista aérea
        for i in range(len(plot_y)-1):
            cv2.line(bird_view_with_lines, 
                    (int(plot_x_left[i]), int(plot_y[i])), 
                    (int(plot_x_left[i+1]), int(plot_y[i+1])), 
                    (0, 0, 255), 3)  # Línea roja para el carril izquierdo
            cv2.line(bird_view_with_lines, 
                    (int(plot_x_right[i]), int(plot_y[i])), 
                    (int(plot_x_right[i+1]), int(plot_y[i+1])), 
                    (255, 0, 0), 3)  # Línea azul para el carril derecho
        
        # Dibujar el centro del carril
        center_line_x = (plot_x_left + plot_x_right) / 2
        for i in range(len(plot_y)-1):
            cv2.line(bird_view_with_lines, 
                    (int(center_line_x[i]), int(plot_y[i])), 
                    (int(center_line_x[i+1]), int(plot_y[i+1])), 
                    (0, 255, 255), 2)  # Línea amarilla para el centro
        
        # Dibujar la posición del auto (centro de la imagen)
        cv2.circle(bird_view_with_lines, (320, 480), 10, (0, 255, 0), -1)  # Círculo verde para el auto
        
        # Vista aérea con overlay verde (la original)
        overlay = transformed_frame.copy()
        points_left = np.array([np.transpose(np.vstack([plot_x_left, plot_y]))])
        points_right = np.array([np.flipud(np.transpose(np.vstack([plot_x_right, plot_y])))])
        quad_points = np.hstack((points_left, points_right)).astype(np.int32)

        cv2.fillPoly(overlay, [quad_points], (0, 255, 0))
        cv2.addWeighted(overlay, 0.2, transformed_frame, 0.8, 0, transformed_frame) # Dibujar área en cenital

        # Invertir la perspectiva
        original_perpective_lane_image = cv2.warpPerspective(transformed_frame, self.inv_matrix, (640, 480))
        result = cv2.addWeighted(original_frame, 1, original_perpective_lane_image, 0.5, 0)
        
        # Mostrar el ángulo (usando el del PID)
        cv2.putText(result, f'Angle (PID): {steering_angle:.2f} deg', (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f'Error: {error:.2f} px', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Agregar texto a la vista aérea con líneas
        cv2.putText(bird_view_with_lines, f'Error: {error:.2f} px', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Lane Center: {lane_center:.1f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Car Position: {car_position}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Empaquetar imágenes de depuración para mostrarlas fuera
        debug_images = {
            "original": original_frame,
            "bird_view_raw": bird_view_raw,  # Vista aérea sin procesar
            "bird_view_lines": bird_view_with_lines,  # Vista aérea con líneas dibujadas
            "cenital": transformed_frame, # Muestra el overlay verde
            "mask": mask,
            "sliding_windows": msk,
            "final_result": result
        }

        return steering_angle, debug_images
