import cv2
import numpy as np
import math

# ======================================================================
# --- LANE DETECTOR ---
# ======================================================================

class MarcosLaneDetector_Advanced:
    """
    Lane detector that only handles lane detection logic.
    Returns angle_desviacion_deg (deviation angle) for PID control.
    """
    
    def __init__(self, threshold):
        # --- Parámetros de la lógica de tu NUEVO script ---
        self.LANE_WIDTH_PX = 500 # ¡CALIBRAR ESTE VALOR! Ancho del carril en píxeles en vista cenital
        self.prev_left_fit = None
        self.prev_right_fit = None
        self.MIN_POINTS_FOR_FIT = 3
        self.MIN_LANE_DISTANCE_PX = 100  # Distancia mínima entre líneas para evitar que se fusionen
        
        # --- Puntos de perspectiva (de tu nuevo script) ---
        # Puntos Origen (SRC) en la imagen original - roi
        self.tl = (160, 180)
        self.bl = (-150, 480)
        self.tr = (480, 180)
        self.br = (790, 480)
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

    def get_lane_metrics(self, frame):
        """
        Detect lanes and return deviation angle.
        
        Args:
            frame: Input frame from camera
            
        Returns:
            tuple: (angle_desviacion_deg, debug_images)
                - angle_desviacion_deg: Angle of deviation in degrees (positive = lane to right, negative = lane to left)
                - debug_images: Dictionary of debug images
            If no lanes found, returns (0, debug_images)
        """
        
        # --- 1. Redimensionar y aplicar Vista Cenital (de tu nuevo script) ---
        frame = cv2.resize(frame, (640, 480))
        original_frame = frame.copy() # Guardar el original para el final
        
        transformed_frame = cv2.warpPerspective(frame, self.matrix, (640, 480))
        
        # --- 2. Detección de color (reemplazo de Trackbars) ---
        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_transformed_frame, self.hsv_lower, self.hsv_upper)
        
        # --- 3. Histograma y Sliding Windows (de tu nuevo script) ---
        # Usar toda la ventana del birdview para el histograma
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0]/2)
        
        # 1. Calcular los picos brutos (brute force peak finding)
        raw_left_base = np.argmax(histogram[:midpoint])
        raw_right_base = np.argmax(histogram[midpoint:]) + midpoint

        # =========================================================
        # --- VALIDACIÓN DE CONTENIDO BLANCO EN ZONA INFERIOR ---
        # =========================================================
        # Verificar si hay suficiente contenido blanco en la zona inferior de cada lado
        BOTTOM_ZONE_HEIGHT = 100  # Altura de la zona inferior a verificar (píxeles desde abajo)
        MIN_WHITE_PIXELS = 20  # Mínimo de píxeles blancos requeridos en la zona inferior
        SEARCH_WINDOW_WIDTH = 100  # Ancho de la ventana de búsqueda alrededor del pico
        
        # Zona inferior de la máscara (últimos BOTTOM_ZONE_HEIGHT píxeles)
        bottom_zone = mask[mask.shape[0] - BOTTOM_ZONE_HEIGHT:, :]
        
        # Verificar carril izquierdo: buscar contenido blanco en zona inferior izquierda
        left_zone_x_start = max(0, raw_left_base - SEARCH_WINDOW_WIDTH // 2)
        left_zone_x_end = min(bottom_zone.shape[1], raw_left_base + SEARCH_WINDOW_WIDTH // 2)
        left_bottom_zone = bottom_zone[:, left_zone_x_start:left_zone_x_end]
        left_white_pixels = np.sum(left_bottom_zone > 0)
        
        # Verificar carril derecho: buscar contenido blanco en zona inferior derecha
        right_zone_x_start = max(0, raw_right_base - SEARCH_WINDOW_WIDTH // 2)
        right_zone_x_end = min(bottom_zone.shape[1], raw_right_base + SEARCH_WINDOW_WIDTH // 2)
        right_bottom_zone = bottom_zone[:, right_zone_x_start:right_zone_x_end]
        right_white_pixels = np.sum(right_bottom_zone > 0)
        
        # Descartar carriles sin suficiente contenido blanco en la zona inferior
        if left_white_pixels < MIN_WHITE_PIXELS:
            raw_left_base = -1  # Descartar carril izquierdo
        
        if right_white_pixels < MIN_WHITE_PIXELS:
            raw_right_base = -1  # Descartar carril derecho
        
        # =========================================================
        # --- CORRECCIÓN DE FUSIÓN (VALIDACIÓN DE DISTANCIA) ---
        # =========================================================
        CONFIDENCE_THRESHOLD = 50  # Altura mínima del pico para considerarlo real
        
        # 2. Verificar si están demasiado cerca o solapadas (solo si ambos están válidos)
        if raw_left_base != -1 and raw_right_base != -1 and raw_right_base - raw_left_base < self.MIN_LANE_DISTANCE_PX:
            # 3. Si están muy cerca, determinar qué pico es más fuerte (confianza)
            left_peak_height = histogram[raw_left_base]
            right_peak_height = histogram[raw_right_base]
            
            # 4. PRIORIZACIÓN Y DESCARTE (priorizar carril derecho)
            if right_peak_height > left_peak_height:
                # El carril derecho es más fuerte: lo mantenemos.
                left_base = -1 
                right_base = raw_right_base
            elif left_peak_height > right_peak_height:
                # El carril izquierdo es más fuerte: lo mantenemos.
                right_base = -1
                left_base = raw_left_base
            else:
                # Si son iguales (o muy débiles), priorizar el derecho (como pediste)
                # O si no tienen suficiente altura, descartar ambos.
                if right_peak_height < CONFIDENCE_THRESHOLD:
                    left_base = -1
                    right_base = -1
                else:
                    # Ambos son buenos, pero están fusionados, priorizamos derecha
                    left_base = -1
                    right_base = raw_right_base 
        else:
            # 5. Si están lo suficientemente separados, usar ambos picos (o el que esté disponible)
            left_base = raw_left_base
            right_base = raw_right_base
        
        # =========================================================
        
        y = 472
        lx, ly, rx, ry = [], [], [], []
        msk = mask.copy() # Copia para dibujar las ventanas

        while y > 0:
            # --- VENTANA IZQUIERDA (Solo busca si left_base != -1) ---
            if left_base != -1:
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
            
            # --- VENTANA DERECHA (Solo busca si right_base != -1) ---
            if right_base != -1:
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
            
            # DIBUJO DE RECTÁNGULOS (Solo si no están deshabilitados)
            if left_base != -1:
                cv2.rectangle(msk, (left_base-50,y), (left_base+50,y-40), (255,255,255), 2)
            if right_base != -1:
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

        # Helper function to calculate distance between two lines at a reference y position
        def get_line_distance(fit1, fit2, y_ref=480):
            """Calculate horizontal distance between two polynomial fits at y_ref"""
            x1 = fit1[0]*y_ref**2 + fit1[1]*y_ref + fit1[2]
            x2 = fit2[0]*y_ref**2 + fit2[1]*y_ref + fit2[2]
            return abs(x2 - x1)

        # Decidir qué líneas usar (priorizando el carril derecho primero)
        # CASO 1: Ambas líneas detectadas - verificar distancia mínima
        if left_fit_current is not None and right_fit_current is not None:
            distance = get_line_distance(left_fit_current, right_fit_current)
            if distance >= self.MIN_LANE_DISTANCE_PX:
                # Distancia válida, usar ambas líneas
                left_fit = left_fit_current
                right_fit = right_fit_current
                self.prev_left_fit = left_fit
                self.prev_right_fit = right_fit
            else:
                # Líneas demasiado cercanas - priorizar carril derecho
                right_fit = right_fit_current
                left_fit = right_fit - [0, 0, self.LANE_WIDTH_PX]
                self.prev_left_fit = left_fit
                self.prev_right_fit = right_fit
        # CASO 2: Solo carril derecho detectado (prioridad)
        elif right_fit_current is not None:
            right_fit = right_fit_current
            left_fit = right_fit - [0, 0, self.LANE_WIDTH_PX]
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
        # CASO 3: Solo carril izquierdo detectado
        elif left_fit_current is not None:
            left_fit = left_fit_current
            right_fit = left_fit + [0, 0, self.LANE_WIDTH_PX]
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
        # CASO 4: Usar líneas previas si están disponibles
        elif self.prev_left_fit is not None and self.prev_right_fit is not None:
            left_fit = self.prev_left_fit
            right_fit = self.prev_right_fit
        else:
            # No se ve nada y no hay memoria
            # Crear vista aérea sin líneas para mantener consistencia
            bird_view_with_lines = transformed_frame.copy()
            cv2.putText(bird_view_with_lines, "No lane detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.circle(bird_view_with_lines, (320, 480), 10, (0, 255, 0), -1)  # Círculo verde para el auto
            
            debug_images = {
                "original": original_frame,
                "bird_view_raw": transformed_frame.copy(),
                "bird_view_lines": bird_view_with_lines,
                "cenital": transformed_frame,
                "mask": mask,
                "sliding_windows": msk,
                "final_result": original_frame
            }
            return None, debug_images  # Return None to indicate no lanes detected
        
        # --- 5. CÁLCULO DEL ÁNGULO DE DESVIACIÓN ---
        # Calculamos el centro del carril como la línea amarilla
        center_fit = [(left_fit[0] + right_fit[0]) / 2, 
                      (left_fit[1] + right_fit[1]) / 2, 
                      (left_fit[2] + right_fit[2]) / 2]
        
        # Evaluar en la posición del carro (y = 480, parte inferior de la imagen)
        y_car = 480
        car_position_x = 320  # Centro del auto (ancho/2)
        
        # Calcular el error posicional: diferencia entre el centro del carril y la posición del auto
        lane_center = center_fit[0]*y_car**2 + center_fit[1]*y_car + center_fit[2]
        error_pixels = lane_center - car_position_x
        
        # Calcular el ángulo de curvatura usando arctan
        lookahead_distance = 100  # Distancia hacia adelante en píxeles para calcular la dirección
        y_current = y_car
        y_ahead = max(0, y_car - lookahead_distance)  # Hacia arriba en la imagen (dirección del vehículo)
        
        # Calcular las posiciones x en ambos puntos
        x_current = center_fit[0] * y_current**2 + center_fit[1] * y_current + center_fit[2]
        x_ahead = center_fit[0] * y_ahead**2 + center_fit[1] * y_ahead + center_fit[2]
        
        # Calcular el ángulo de la línea entre estos dos puntos
        dx = x_ahead - x_current
        dy = y_ahead - y_current  # Será negativo (y_ahead < y_current)
        
        # El ángulo se calcula como atan2(dx, -dy) porque:
        # - El vehículo avanza hacia arriba (y disminuye)
        # - Necesitamos negar dy para que represente la dirección de avance
        # - atan2 maneja correctamente los signos en todos los cuadrantes
        angle_rad = math.atan2(dx, -dy)
        curvature_angle_deg = math.degrees(angle_rad)
        
        # Limitar el ángulo de curvatura al rango válido (para visualización)
        curvature_angle_deg = max(min(curvature_angle_deg, 30), -30)
        
        # --- CALCULAR ERROR ANGULAR USANDO ARCTAN ---
        # Convertir el error posicional (píxeles) a error angular usando arctan
        # Error positivo = carril a la derecha del auto → ángulo positivo
        # Error negativo = carril a la izquierda del auto → ángulo negativo
        error_angle_rad = math.atan2(-error_pixels, lookahead_distance)
        error_angle_deg = math.degrees(error_angle_rad)
        
        # Combinar el error angular con el ángulo de curvatura
        # El error angular corrige la posición, el ángulo de curvatura anticipa la dirección
        curvature_factor = 0.5  # Factor para combinar curvatura (ajustable)
        angle_desviacion_deg = error_angle_deg + curvature_factor * curvature_angle_deg
        
        # Limitar el ángulo de desviación al rango válido
        angle_desviacion_deg = max(min(angle_desviacion_deg, 30), -30)

        # --- 6. Visualización ---
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
        cv2.addWeighted(overlay, 0.4, transformed_frame, 0.8, 0, transformed_frame) # Dibujar área en cenital

        # Invertir la perspectiva
        original_perpective_lane_image = cv2.warpPerspective(transformed_frame, self.inv_matrix, (640, 480))
        result = cv2.addWeighted(original_frame, 1, original_perpective_lane_image, 0.5, 0)
        
        # Mostrar información de depuración
        cv2.putText(result, f'Deviation Angle: {angle_desviacion_deg:.2f} deg', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f'Curvature: {curvature_angle_deg:.2f} deg', (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.putText(result, f'Error Pixels: {error_pixels:.1f}', (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
        
        # Agregar texto a la vista aérea con líneas
        cv2.putText(bird_view_with_lines, f'Deviation Angle: {angle_desviacion_deg:.2f} deg', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Curvature: {curvature_angle_deg:.2f} deg', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(bird_view_with_lines, f'Error Pixels: {error_pixels:.1f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.putText(bird_view_with_lines, f'Lane Center: {lane_center:.1f}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Car Position: {car_position_x}', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

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

        return angle_desviacion_deg, debug_images
