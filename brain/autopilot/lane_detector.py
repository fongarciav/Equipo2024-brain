import cv2
import numpy as np
import time
import math

# ======================================================================
# --- LANE DETECTOR ---
# ======================================================================

class MarcosLaneDetector_Advanced:
    
    def __init__(self, threshold):
        # --- Parámetros de la lógica de tu NUEVO script ---
        self.LANE_WIDTH_PX = 400 # ¡CALIBRAR ESTE VALOR! Ancho del carril en píxeles en vista cenital
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
        
        # Time tracking for frame rate
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
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
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
            return 0, debug_images
        
        # --- 5. CÁLCULO DEL ÁNGULO BASADO EN LA CURVATURA DE LA LÍNEA AMARILLA ---
        # Calculamos el centro del carril como la línea amarilla
        center_fit = [(left_fit[0] + right_fit[0]) / 2, 
                      (left_fit[1] + right_fit[1]) / 2, 
                      (left_fit[2] + right_fit[2]) / 2]
        
        # Evaluar en la posición del carro (y = 480, parte inferior de la imagen)
        y_car = 480
        car_position_x = 320  # Centro del auto (ancho/2)
        
        # Calcular el ángulo de dirección basado en la curvatura de la línea amarilla
        # Usamos dos puntos cercanos en la línea para calcular la dirección
        # Punto 1: posición actual del carro (y_car)
        # Punto 2: un punto adelante en la línea (y_car - lookahead_distance)
        lookahead_distance = 100  # Distancia hacia adelante en píxeles para calcular la dirección
        
        y_current = y_car
        y_ahead = max(0, y_car - lookahead_distance)  # Hacia arriba en la imagen (dirección del vehículo)
        
        # Calcular las posiciones x en ambos puntos
        x_current = center_fit[0] * y_current**2 + center_fit[1] * y_current + center_fit[2]
        x_ahead = center_fit[0] * y_ahead**2 + center_fit[1] * y_ahead + center_fit[2]
        
        # Calcular el error: diferencia entre el centro del carril y la posición del auto
        # Error positivo = carril está a la derecha → necesitamos girar a la derecha
        # Error negativo = carril está a la izquierda → necesitamos girar a la izquierda
        lane_center = center_fit[0]*y_car**2 + center_fit[1]*y_car + center_fit[2]
        error = lane_center - car_position_x
        
        # Calcular el ángulo de curvatura para visualización
        # Usamos dos puntos cercanos en la línea para calcular la dirección
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
        
        # Limitar el ángulo de curvatura al rango válido
        curvature_angle_deg = max(min(curvature_angle_deg, 30), -30)
        
        # --- USAR CURVATURA DIRECTAMENTE ---
        # Usar el ángulo de curvatura directamente como ángulo de dirección
        # El ángulo de curvatura ya está en grados y representa la dirección del carril
        # Positivo = carril se curva hacia la derecha → girar a la derecha
        # Negativo = carril se curva hacia la izquierda → girar a la izquierda
        # Rango: -30 a +30 grados
        steering_angle = curvature_angle_deg
        
        # Si el ángulo está entre -6 y 6 grados, ir recto
        if abs(steering_angle) <= 6.0:
            steering_angle = 0.0

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
        
        # Mostrar el ángulo de dirección (usado para control)
        cv2.putText(result, f'Steering Angle: {steering_angle:.2f} deg', (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(result, f'Error: {error:.2f} px', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Agregar texto a la vista aérea con líneas
        cv2.putText(bird_view_with_lines, f'Steering Angle: {steering_angle:.2f} deg', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(bird_view_with_lines, f'Error: {error:.2f} px', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Lane Center: {lane_center:.1f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(bird_view_with_lines, f'Car Position: {car_position_x}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

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
