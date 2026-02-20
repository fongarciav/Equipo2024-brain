import cv2
import numpy as np
import math
from abc import ABC, abstractmethod
from sklearn.cluster import DBSCAN

# ======================================================================
# --- CLASE BASE PARA DETECTORES DE CARRIL ---
# ======================================================================

class LaneDetector(ABC):
    """Clase base abstracta para estrategias de detección de carriles."""
    
    @abstractmethod
    def get_lane_metrics(self, frame):
        """
        Detectar carriles y retornar ángulo de desviación.
        
        Args:
            frame: Frame de entrada de la cámara (numpy array)
            
        Returns:
            tuple: (angle_deviation_deg, debug_images)
                - angle_deviation_deg: Ángulo de desviación en grados (float o None)
                - debug_images: Diccionario de imágenes de depuración (dict o None)
        """
        pass

# ======================================================================
# --- IMPLEMENTACIONES DE DETECTORES DE CARRIL ---
# ======================================================================

class MarcosLaneDetector_Advanced(LaneDetector):
    """
    Detector de carriles que solo maneja la lógica de detección.
    Retorna angle_desviacion_deg (ángulo de desviación) para control PID.
    """
    
    def __init__(self, threshold):
        # --- Parámetros de la lógica de tu NUEVO script ---
        self.LANE_WIDTH_PX = 800 # ¡CALIBRAR ESTE VALOR! Ancho del carril en píxeles en vista cenital
        self.prev_left_fit = None
        self.prev_right_fit = None
        self.ENABLE_MEMORY_MODE = True  # Permite habilitar/deshabilitar fallback MEMORY
        self.MIN_POINTS_FOR_FIT = 3
        self.MIN_LANE_DISTANCE_PX = 40  # Distancia mínima entre líneas (reducida para modo más permisivo)
        self.ENABLE_MIN_LANE_DISTANCE_CHECK = True  # Permite desactivar chequeo de distancia mínima en STEREO
        self.ENABLE_HARD_SIDE_POINT_FILTER = False  # Modo limpio: no descartar puntos por cruzar el centro
        self.ENABLE_BASE_EXCLUSION_FILTER = False  # Modo limpio: no invalidar fits por base en lado opuesto
        
        # --- Parámetros de cálculo de ángulos ---
        self.LOOKAHEAD_DISTANCE = 250  # Distancia hacia adelante para calcular dirección (px)
        self.CURVATURE_THRESHOLD = 10.0  # Grados: menor a esto se considera recto
        self.STRAIGHT_LANE_WIDTH_REDUCTION = 60  # Reducción de píxeles para rectas
        
        # --- Parámetros de ventanas deslizantes ---
        self.SLIDING_WINDOW_START_Y = 472  # Posición Y inicial para sliding windows (desde abajo)
        self.SLIDING_WINDOW_HEIGHT = 20  # Altura de cada slice para mejorar conectividad de DBSCAN
        self.SLIDING_WINDOW_WIDTH = 50  # Ancho a cada lado del centro de la ventana
        self.SLIDING_WINDOW_EXPANDED_WIDTH = 150  # Ancho expandido para búsqueda ampliada
        self.ENABLE_EXPANDED_SEARCH = False  # Habilitar/deshabilitar búsqueda expandida
        self.HISTOGRAM_PEAK_THRESHOLD = 3000  # Umbral mínimo para considerar un pico de histograma válido
        self.HISTOGRAM_SMOOTH_KERNEL_SIZE = 9  # Kernel 1D para suavizar histograma antes de detectar picos

        # --- Parámetros de slices por hemisferio ---
        self.HEMISLICE_MIN_CONTOUR_AREA = 20
        self.HEMISLICE_MAX_JUMP_PX = 120
        self.HEMISLICE_TREND_TOLERANCE_PX = 90

        # --- Parámetros DBSCAN (selección de carriles por clustering) ---
        self.DBSCAN_EPS = 35
        self.DBSCAN_MIN_SAMPLES = 3
        self.MIN_CLUSTER_POINTS = 6
        self.USE_WORLD_COORDINATES_FOR_ORDERING = False
        self.DEBUG_LANE_CLUSTER_SELECTION = False
        self.ENABLE_TEMPORAL_CLUSTER_MATCHING = True
        self.CLUSTER_MATCH_Y_SAMPLES = np.array([480, 420, 360, 300], dtype=np.float32)
        self.CLUSTER_MATCH_MAX_COST = 180.0
        self.CLUSTER_SWAP_PENALTY = 40.0

        # --- Threshold automático por ROI de referencia (bloque compartido) ---
        self.AUTO_THR_REF_X_NORM = 0.5
        self.AUTO_THR_REF_Y_FROM_BOTTOM_PX = 8
        self.AUTO_THR_REF_ROI_SIZE = 41
        self.AUTO_THR_BG_PERCENTILE = 90.0
        self.AUTO_THR_OFFSET = 45
        
        # --- Parámetros de Control ---
        # La Ganancia o peso que le das a la anticipación.
        # Función: Es una perilla de ajuste (tuning).
        # Valor alto (ej: 1.0): El auto "corta" las curvas agresivamente.
        # Valor bajo (ej: 0.2): El auto entra tarde a las curvas y depende más de corregir cuando ya se salió un poco.
        # TODO: Ajustar este valor desde el web server.
        self.curvature_factor = 0.5  # Factor para combinar curvatura
        self.error_factor = 0.3  # Factor para combinar error posicional
        
        # --- Puntos de perspectiva (de tu nuevo script) ---
        # Puntos Origen (SRC) - ROI
        self.tl = (160, 180)
        self.bl = (0, 450)
        self.tr = (480, 180)
        self.br = (640, 450)
        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        
        # Puntos Destino (DST) - VISTA CENITAL
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        
        # Matrices de transformación
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        # --- Valores de color HSV ---
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
        
        # --- Redimensionar y aplicar Vista Cenital ---
        frame = cv2.resize(frame, (640, 480))
        original_frame = frame.copy() # Guardar el original para el final
        
        # use cv2.cuda.warpPerspective
        transformed_frame = cv2.warpPerspective(frame, self.matrix, (640, 480))
        
        # --- Detección de color + threshold automático por ROI de referencia ---
        gray_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY)

        def _clamp(v, lo, hi):
            return max(lo, min(hi, v))

        def _compute_auto_threshold_from_ref_roi(gray):
            """
            Calcula threshold automático usando un bloque de referencia que debe caer en suelo negro.
            Basado en el bloque de referencia compartido por el usuario.
            """
            h, w = gray.shape[:2]
            cx = int(_clamp(self.AUTO_THR_REF_X_NORM, 0.0, 1.0) * (w - 1))
            cy = int(_clamp(h - 1 - self.AUTO_THR_REF_Y_FROM_BOTTOM_PX, 0, h - 1))

            r = max(1, self.AUTO_THR_REF_ROI_SIZE // 2)
            x0 = _clamp(cx - r, 0, w - 1)
            x1 = _clamp(cx + r, 0, w - 1)
            y0 = _clamp(cy - r, 0, h - 1)
            y1 = _clamp(cy + r, 0, h - 1)

            roi = gray[y0:y1 + 1, x0:x1 + 1]
            if roi.size == 0:
                t = 128
            else:
                bg = float(np.percentile(roi, _clamp(self.AUTO_THR_BG_PERCENTILE, 0.0, 100.0)))
                t = int(round(bg + float(self.AUTO_THR_OFFSET)))

            t = int(_clamp(t, 0, 255))
            return t, (x0, y0, x1, y1)

        auto_thr, auto_thr_roi = _compute_auto_threshold_from_ref_roi(gray_transformed_frame)
        _, mask = cv2.threshold(gray_transformed_frame, auto_thr, 255, cv2.THRESH_BINARY)

        # --- Histograma (debug) ---
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)

        # --- Slices por hemisferio con validación de coherencia ---
        y = self.SLIDING_WINDOW_START_Y
        lx, ly, rx, ry = [], [], [], []
        msk = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)
        h, w = mask.shape[:2]

        # Draw histogram visualization
        histogram_viz = np.zeros((100, mask.shape[1], 3), dtype=np.uint8)
        histogram_normalized = (histogram / histogram.max() * 100).astype(int) if histogram.max() > 0 else histogram
        for i, hv in enumerate(histogram_normalized):
            if hv > 0:
                cv2.line(histogram_viz, (i, 100), (i, 100 - hv), (255, 255, 255), 1)
        cv2.line(histogram_viz, (midpoint, 0), (midpoint, 100), (0, 255, 255), 2)

        window_results = {'left': [], 'right': []}
        window_index = 0

        def _extract_candidates(contours):
            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.HEMISLICE_MIN_CONTOUR_AREA:
                    continue
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue

                cx = int(M['m10'] / M['m00'])
                cy_local = int(M['m01'] / M['m00'])
                candidates.append((cx, cy_local, area))
            return candidates

        all_points = []
        cluster_debug_info = []
        labels = None

        while y > 0:
            y0 = max(0, y - self.SLIDING_WINDOW_HEIGHT)
            y1 = y

            full_slice = mask[y0:y1, :]
            contours, _ = cv2.findContours(full_slice, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            candidates = _extract_candidates(contours)

            found_any = len(candidates) > 0
            for cx, cy_local, _ in candidates:
                point_y = y0 + cy_local
                all_points.append((cx, point_y))
                cv2.circle(msk, (cx, point_y), 2, (180, 180, 180), -1)

            window_results['left'].append((found_any, False))
            window_results['right'].append((found_any, False))

            slice_color = (0, 255, 0) if found_any else (0, 0, 255)
            cv2.rectangle(msk, (0, y1), (w - 1, y0), slice_color, 1)
            cv2.line(msk, (midpoint, y0), (midpoint, y1), (80, 80, 80), 1)
            cv2.putText(msk, f'S{window_index}', (8, max(15, y0 + 15)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            y -= self.SLIDING_WINDOW_HEIGHT
            window_index += 1

        selected_left_points = []
        selected_right_points = []
        left_base = -1
        right_base = -1
        selected_left_score = None
        selected_right_score = None

        if len(all_points) >= self.DBSCAN_MIN_SAMPLES:
            X = np.array(all_points, dtype=np.float32)
            labels = DBSCAN(eps=self.DBSCAN_EPS, min_samples=self.DBSCAN_MIN_SAMPLES).fit(X).labels_

            clusters = []
            for label_id in np.unique(labels):
                if label_id == -1:
                    continue
                pts = X[labels == label_id]
                support = len(pts)
                if support < self.MIN_CLUSTER_POINTS:
                    continue

                centroid_x = float(np.mean(pts[:, 0]))
                cluster_fit = None
                if support >= self.MIN_POINTS_FOR_FIT:
                    try:
                        cluster_fit = np.polyfit(pts[:, 1], pts[:, 0], 2)
                    except np.linalg.LinAlgError:
                        cluster_fit = None

                clusters.append({
                    'label_id': int(label_id),
                    'points': pts,
                    'centroid_x': centroid_x,
                    'support': support,
                    'fit': cluster_fit
                })

            if self.DEBUG_LANE_CLUSTER_SELECTION:
                print(f"[DBSCAN] valid_clusters={len(clusters)}")
                for c in clusters:
                    print(f"  label={c['label_id']} support={c['support']} centroid_x={c['centroid_x']:.1f}")

            # Selección ego-lane con continuidad temporal:
            # 1) calcular costo cluster<->historial (prev_left_fit / prev_right_fit)
            # 2) penalizar swaps de lado abruptos
            # 3) elegir asignación de menor costo total (izquierda y derecha)
            left_cluster = None
            right_cluster = None
            reference_center_x = 0.0 if self.USE_WORLD_COORDINATES_FOR_ORDERING else (w / 2)

            def _fit_distance_cost(candidate_fit, prev_fit):
                if candidate_fit is None or prev_fit is None:
                    return float('inf')
                y_samples = self.CLUSTER_MATCH_Y_SAMPLES
                x_candidate = candidate_fit[0] * y_samples**2 + candidate_fit[1] * y_samples + candidate_fit[2]
                x_prev = prev_fit[0] * y_samples**2 + prev_fit[1] * y_samples + prev_fit[2]
                return float(np.mean(np.abs(x_candidate - x_prev)))

            def _side_penalty(side, centroid_x):
                if side == 'L':
                    return 0.0 if centroid_x < reference_center_x else self.CLUSTER_SWAP_PENALTY
                return 0.0 if centroid_x >= reference_center_x else self.CLUSTER_SWAP_PENALTY

            def _cluster_cost_for_side(cluster, side):
                prev_fit = self.prev_left_fit if side == 'L' else self.prev_right_fit
                temporal_cost = _fit_distance_cost(cluster['fit'], prev_fit)
                if not np.isfinite(temporal_cost):
                    temporal_cost = abs(cluster['centroid_x'] - reference_center_x)
                return temporal_cost + _side_penalty(side, cluster['centroid_x'])

            if self.ENABLE_TEMPORAL_CLUSTER_MATCHING and len(clusters) > 0:
                left_candidates = sorted(
                    [(_cluster_cost_for_side(c, 'L'), idx, c) for idx, c in enumerate(clusters)],
                    key=lambda t: t[0]
                )
                right_candidates = sorted(
                    [(_cluster_cost_for_side(c, 'R'), idx, c) for idx, c in enumerate(clusters)],
                    key=lambda t: t[0]
                )

                for cost, idx, candidate in left_candidates:
                    if self.prev_left_fit is not None and cost > self.CLUSTER_MATCH_MAX_COST:
                        continue
                    left_cluster = candidate
                    selected_left_score = cost
                    left_selected_idx = idx
                    break

                for cost, idx, candidate in right_candidates:
                    if self.prev_right_fit is not None and cost > self.CLUSTER_MATCH_MAX_COST:
                        continue
                    if left_cluster is not None and idx == left_selected_idx:
                        continue
                    right_cluster = candidate
                    selected_right_score = cost
                    break

            if left_cluster is None or right_cluster is None:
                left_side_clusters = [c for c in clusters if c['centroid_x'] < reference_center_x]
                right_side_clusters = [c for c in clusters if c['centroid_x'] >= reference_center_x]

                if left_cluster is None and left_side_clusters:
                    left_cluster = min(left_side_clusters, key=lambda c: abs(reference_center_x - c['centroid_x']))
                    selected_left_score = abs(reference_center_x - left_cluster['centroid_x'])

                if right_cluster is None and right_side_clusters:
                    right_cluster = min(right_side_clusters, key=lambda c: abs(c['centroid_x'] - reference_center_x))
                    selected_right_score = abs(reference_center_x - right_cluster['centroid_x'])

            if left_cluster is not None:
                selected_left_points = [tuple(map(int, p)) for p in left_cluster['points']]
                left_base = int(round(left_cluster['centroid_x']))
                cluster_debug_info.append(('L', left_cluster['label_id'], left_cluster['centroid_x']))

            if right_cluster is not None:
                selected_right_points = [tuple(map(int, p)) for p in right_cluster['points']]
                right_base = int(round(right_cluster['centroid_x']))
                cluster_debug_info.append(('R', right_cluster['label_id'], right_cluster['centroid_x']))

            if self.DEBUG_LANE_CLUSTER_SELECTION:
                print(f"[DBSCAN] selected_left={None if left_cluster is None else left_cluster['label_id']} selected_right={None if right_cluster is None else right_cluster['label_id']}")
                if selected_left_score is not None or selected_right_score is not None:
                    print(f"[DBSCAN] score_left={selected_left_score} score_right={selected_right_score}")

            # Overlay de centroides de clusters
            for c in clusters:
                cx = int(round(c['centroid_x']))
                cy = 30 + 18 * c['label_id']
                cv2.circle(msk, (cx, 20), 5, (255, 255, 0), -1)
                cv2.putText(msk, f"C{c['label_id']}:{c['support']}", (cx + 6, 24),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

            if left_cluster is not None:
                cv2.circle(msk, (left_base, 38), 6, (0, 0, 255), -1)
                cv2.putText(msk, f"L{left_cluster['label_id']}", (left_base + 6, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            if right_cluster is not None:
                cv2.circle(msk, (right_base, 56), 6, (255, 0, 0), -1)
                cv2.putText(msk, f"R{right_cluster['label_id']}", (right_base + 6, 58),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        lx = [p[0] for p in selected_left_points]
        ly = [p[1] for p in selected_left_points]
        rx = [p[0] for p in selected_right_points]
        ry = [p[1] for p in selected_right_points]

        raw_left_base = left_base
        raw_right_base = right_base
        left_base = raw_left_base
        right_base = raw_right_base

        if left_base != -1:
            cv2.circle(histogram_viz, (left_base, 50), 8, (0, 0, 255), -1)
        if right_base != -1:
            cv2.circle(histogram_viz, (right_base, 50), 8, (255, 0, 0), -1)

        # Draw connecting lines between detected points to show trajectory
        if len(lx) > 1:
            for i in range(len(lx)-1):
                cv2.line(msk, (lx[i], ly[i]), (lx[i+1], ly[i+1]), (0, 255, 255), 1)
        if len(rx) > 1:
            for i in range(len(rx)-1):
                cv2.line(msk, (rx[i], ry[i]), (rx[i+1], ry[i+1]), (0, 255, 255), 1)

        # Add statistics overlay
        stats_y = 70
        cv2.putText(msk, f'Left points: {len(lx)}', (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        stats_y += 30
        cv2.putText(msk, f'Right points: {len(rx)}', (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        stats_y += 30
        valid_clusters_count = len(set(labels)) - (1 if labels is not None and -1 in labels else 0) if labels is not None else 0
        cv2.putText(msk, f'Valid clusters: {valid_clusters_count}', (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        stats_y += 30
        cv2.putText(msk, f'Auto thr: {auto_thr}', (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        stats_y += 30
        cv2.putText(msk, f'ROI: {auto_thr_roi}', (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        stats_y += 30
        left_status = 'VALID' if left_base != -1 else 'INVALID'
        right_status = 'VALID' if right_base != -1 else 'INVALID'
        cv2.putText(msk, f'Validation: L={left_status} R={right_status}',
                   (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                   (0, 255, 0) if left_status == 'VALID' and right_status == 'VALID' else (0, 255, 255), 2)
        stats_y += 30
        score_left_text = "-" if selected_left_score is None else f"{selected_left_score:.1f}"
        score_right_text = "-" if selected_right_score is None else f"{selected_right_score:.1f}"
        cv2.putText(msk, f'Scores: L={score_left_text} R={score_right_text}',
                   (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                   (180, 255, 180), 1)

    # --- Polyfit y Lógica de Estimación ---
        left_fit_current = None
        right_fit_current = None
        
        # ==============================================================================
        # --- FILTRO DE SANIDAD DE PUNTOS CRUDOS (HARD PARTITION) ---
        # ==============================================================================
        # Antes de calcular nada, eliminamos puntos que cruzaron la frontera central.
        # Esto impide que la ventana Derecha "robe" puntos de la línea Izquierda.
        
        MIDPOINT_X = 320  # Mitad de la imagen (640 / 2)

        # 1. Filtrar puntos por lado (opcional, desactivado por defecto en modo limpio)
        if self.ENABLE_HARD_SIDE_POINT_FILTER:
            if len(lx) > 0:
                valid_l = [(x, y) for x, y in zip(lx, ly) if x < MIDPOINT_X]
                if len(valid_l) > 0:
                    lx, ly = zip(*valid_l)
                    lx, ly = list(lx), list(ly)
                else:
                    lx, ly = [], []

            if len(rx) > 0:
                valid_r = [(x, y) for x, y in zip(rx, ry) if x > MIDPOINT_X]
                if len(valid_r) > 0:
                    rx, ry = zip(*valid_r)
                    rx, ry = list(rx), list(ry)
                else:
                    rx, ry = [], []

        # ==============================================================================
        
        if len(lx) >= self.MIN_POINTS_FOR_FIT:
            try: left_fit_current = np.polyfit(ly, lx, 2)
            except np.linalg.LinAlgError: pass

        if len(ry) >= self.MIN_POINTS_FOR_FIT:
            try: right_fit_current = np.polyfit(ry, rx, 2)
            except np.linalg.LinAlgError: pass
        
        if len(ry) >= self.MIN_POINTS_FOR_FIT:
            try: right_fit_current = np.polyfit(ry, rx, 2)
            except np.linalg.LinAlgError: pass

        # ==============================================================================
        # [NUEVO] FILTRO DE POSICIÓN (ZONA DE EXCLUSIÓN)
        # ==============================================================================
        # Esto evita que puntos a la izquierda sean identificados como carril derecho
        
        MIDPOINT_X = 320  # Mitad de tu imagen (640 / 2)
        
        # 1-2. Validación de base por lado (opcional, desactivada por defecto en modo limpio)
        if self.ENABLE_BASE_EXCLUSION_FILTER:
            if left_fit_current is not None:
                lx_base = left_fit_current[0]*480**2 + left_fit_current[1]*480 + left_fit_current[2]
                if lx_base > MIDPOINT_X:
                    print(f"🚫 RECHAZADO: Falso Izquierdo en zona derecha (x={int(lx_base)})")
                    left_fit_current = None

            if right_fit_current is not None:
                rx_base = right_fit_current[0]*480**2 + right_fit_current[1]*480 + right_fit_current[2]
                if rx_base < MIDPOINT_X:
                    print(f"🚫 RECHAZADO: Falso Derecho en zona izquierda (x={int(rx_base)})")
                    right_fit_current = None

        # Función auxiliar para calcular distancia entre dos líneas en la posición y=480
        def get_line_distance(fit1, fit2):
            """Calcular distancia horizontal entre dos ajustes polinomiales en y=480"""
            y_ref = 480
            x1 = fit1[0]*y_ref**2 + fit1[1]*y_ref + fit1[2]
            x2 = fit2[0]*y_ref**2 + fit2[1]*y_ref + fit2[2]
            return abs(x2 - x1)
        
        # Función auxiliar para calcular ángulo de curvatura inline
        def get_curvature_angle(fit):
            """Calcular ángulo de curvatura desde ajuste polinomial"""
            y_current = 480
            y_ahead = max(0, 480 - self.LOOKAHEAD_DISTANCE)
            x_current = fit[0] * y_current**2 + fit[1] * y_current + fit[2]
            x_ahead = fit[0] * y_ahead**2 + fit[1] * y_ahead + fit[2]
            dx = x_ahead - x_current
            dy = y_ahead - y_current
            angle_rad = math.atan2(dx, -dy)
            return math.degrees(angle_rad)
        
        # Función de Sanity Check para intersección de líneas
        def lines_intersect(fit1, fit2, y_start=0, y_end=480):
            """Verifica si dos polinomios cuadráticos se cruzan en el rango [y_start, y_end]"""
            # Diferencia de polinomios: f1(y) - f2(y) = (a1-a2)y^2 + (b1-b2)y + (c1-c2) = 0
            a = fit1[0] - fit2[0]
            b = fit1[1] - fit2[1]
            c = fit1[2] - fit2[2]
            
            if abs(a) < 1e-6: # Casi lineales
                if abs(b) < 1e-6: return False # Paralelas
                y_intersect = -c / b
                return y_start <= y_intersect <= y_end
            
            # Ecuación cuadrática
            delta = b**2 - 4*a*c
            if delta < 0: return False # No hay intersección real
            
            sqrt_delta = math.sqrt(delta)
            y1 = (-b + sqrt_delta) / (2*a)
            y2 = (-b - sqrt_delta) / (2*a)
            
            return (y_start <= y1 <= y_end) or (y_start <= y2 <= y_end)

        # =========================================================
        # --- LÓGICA DE DECISIÓN (ÁRBOL JERÁRQUICO) ---
        # =========================================================
        
        detection_mode = "NONE"
        final_left_fit = None
        final_right_fit = None
        
        # --- NIVEL 1: ESTEREO (Ambas líneas detectadas y válidas) ---
        if left_fit_current is not None and right_fit_current is not None:
            distance = get_line_distance(left_fit_current, right_fit_current)
            intersect = lines_intersect(left_fit_current, right_fit_current)
            distance_ok = (distance >= self.MIN_LANE_DISTANCE_PX) if self.ENABLE_MIN_LANE_DISTANCE_CHECK else True

            if distance_ok and not intersect:
                detection_mode = "STEREO"
                final_left_fit = left_fit_current
                final_right_fit = right_fit_current
                # Actualizar memoria
                self.prev_left_fit = final_left_fit
                self.prev_right_fit = final_right_fit

        # --- NIVEL 2: MONO_DERECHA (Si falló Nivel 1, intentar solo con derecha) ---
        if detection_mode == "NONE" and right_fit_current is not None:
            detection_mode = "MONO_RIGHT"
            final_right_fit = right_fit_current
            
            # Reconstruir izquierda
            curvature = abs(get_curvature_angle(final_right_fit))
            lane_width = self.LANE_WIDTH_PX if curvature >= self.CURVATURE_THRESHOLD else (self.LANE_WIDTH_PX - self.STRAIGHT_LANE_WIDTH_REDUCTION)
            final_left_fit = final_right_fit - [0, 0, lane_width]
            
            # Actualizar memoria (forzamos porque es nuestra mejor estimación actual)
            self.prev_left_fit = final_left_fit
            self.prev_right_fit = final_right_fit

        # --- NIVEL 3: MONO_IZQUIERDA (Si falló Nivel 2, intentar solo con izquierda) ---
        if detection_mode == "NONE" and left_fit_current is not None:
            detection_mode = "MONO_LEFT"
            final_left_fit = left_fit_current
            
            # Reconstruir derecha
            curvature = abs(get_curvature_angle(final_left_fit))
            lane_width = self.LANE_WIDTH_PX if curvature >= self.CURVATURE_THRESHOLD else (self.LANE_WIDTH_PX - self.STRAIGHT_LANE_WIDTH_REDUCTION)
            final_right_fit = final_left_fit + [0, 0, lane_width]
            
            # Actualizar memoria
            self.prev_left_fit = final_left_fit
            self.prev_right_fit = final_right_fit

        # --- NIVEL 4: MEMORIA (Si falló todo, usar memoria si existe y está habilitada) ---
        if detection_mode == "NONE":
            if self.ENABLE_MEMORY_MODE and self.prev_left_fit is not None and self.prev_right_fit is not None:
                detection_mode = "MEMORY"
                final_left_fit = self.prev_left_fit
                final_right_fit = self.prev_right_fit
            else:
                # FALLO TOTAL: No hay nada que hacer
                bird_view_with_lines = transformed_frame.copy()
                no_lane_text = "NO LANE DETECTED" if self.ENABLE_MEMORY_MODE else "NO LANE DETECTED (MEMORY OFF)"
                cv2.putText(bird_view_with_lines, no_lane_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                debug_images = {
                    "original": original_frame,
                    "bird_view_raw": transformed_frame.copy(),
                    "bird_view_lines": bird_view_with_lines,
                    "cenital": transformed_frame,
                    "mask": mask,
                    "histogram": histogram_viz,
                    "sliding_windows": msk,
                    "final_result": original_frame
                }
                return None, debug_images

        # Asignar los fits finales para el resto del cálculo
        left_fit = final_left_fit
        right_fit = final_right_fit
        
        # --- CÁLCULO DEL ÁNGULO DE DESVIACIÓN ---
        # Calculamos el centro del carril como la línea amarilla
        center_fit = [(left_fit[0] + right_fit[0]) / 2, 
                      (left_fit[1] + right_fit[1]) / 2, 
                      (left_fit[2] + right_fit[2]) / 2]
        
        # Posición del carro
        y_car = 480
        car_position_x = 320
        
        # Calcular el error posicional: diferencia entre el centro del carril y la posición del auto
        lane_center = center_fit[0]*y_car**2 + center_fit[1]*y_car + center_fit[2]
        error_pixels = lane_center - car_position_x
        
        # Calcular el ángulo de curvatura del carril (dirección hacia adelante)
        y_current = y_car
        y_ahead = max(0, y_car - self.LOOKAHEAD_DISTANCE)  # Hacia arriba en la imagen (dirección del vehículo)
        
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
        error_angle_rad = math.atan2(error_pixels, self.LOOKAHEAD_DISTANCE)
        error_angle_deg = math.degrees(error_angle_rad)
        
        # Combinar el error angular con el ángulo de curvatura
        # El error angular corrige la posición, el ángulo de curvatura anticipa la dirección
        angle_desviacion_deg = self.error_factor * error_angle_deg + self.curvature_factor * curvature_angle_deg
        
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
        
        # --- Definir colores según el modo de detección ---
        color_left = (0, 0, 255)   # Rojo por defecto (Real)
        color_right = (255, 0, 0)  # Azul por defecto (Real)
        color_reconstructed = (150, 150, 150) # Gris para líneas reconstruidas/virtuales
        color_memory = (0, 255, 255) # Amarillo para memoria
        
        if detection_mode == "STEREO":
            color_left = (0, 0, 255)   # Real
            color_right = (255, 0, 0)  # Real
        elif detection_mode == "MONO_RIGHT":
            color_left = color_reconstructed # Reconstruida
            color_right = (255, 0, 0)  # Real
        elif detection_mode == "MONO_LEFT":
            color_left = (0, 0, 255)   # Real
            color_right = color_reconstructed # Reconstruida
        elif detection_mode == "MEMORY":
            color_left = color_memory
            color_right = color_memory
            
        # Dibujar las líneas del carril en la vista aérea
        # Convertir puntos a formato polylines (int32)
        pts_left = np.vstack((plot_x_left, plot_y)).astype(np.int32).T
        pts_right = np.vstack((plot_x_right, plot_y)).astype(np.int32).T
        
        cv2.polylines(bird_view_with_lines, [pts_left], False, color_left, 3)
        cv2.polylines(bird_view_with_lines, [pts_right], False, color_right, 3)
        
        # Dibujar el centro del carril
        center_line_x = (plot_x_left + plot_x_right) / 2
        for i in range(len(plot_y)-1):
            cv2.line(bird_view_with_lines, 
                    (int(center_line_x[i]), int(plot_y[i])), 
                    (int(center_line_x[i+1]), int(plot_y[i+1])), 
                    (0, 255, 255), 2)  # Línea amarilla para el centro
        
        # Dibujar la posición del auto (centro de la imagen)
        cv2.circle(bird_view_with_lines, (320, 480), 10, (0, 255, 0), -1)  # Círculo verde para el auto
        
        # Mostrar el MODO DE DETECCIÓN en la pantalla
        cv2.putText(bird_view_with_lines, f"MODE: {detection_mode}", (10, 260), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # =========================================================
        
        # =========================================================
        # --- VISUALIZACION ---
        # =========================================================
        # 1. Dibujar el centro del carril en la posición del auto
        lane_center_int = int(lane_center)
        cv2.circle(bird_view_with_lines, (lane_center_int, y_car), 8, (255, 255, 0), -1)  # Círculo cyan para lane center
        
        # 2. Dibujar el punto de lookahead en el centro del carril
        x_ahead_int = int(x_ahead)
        y_ahead_int = int(y_ahead)
        cv2.circle(bird_view_with_lines, (x_ahead_int, y_ahead_int), 8, (255, 0, 255), -1)  # Círculo magenta para lookahead
        
        # 3. Dibujar línea de dirección desde posición actual hasta lookahead
        x_current_int = int(x_current)
        cv2.line(bird_view_with_lines, (x_current_int, y_car), (x_ahead_int, y_ahead_int), (255, 128, 0), 3)  # Línea azul claro
        
        # 4. Dibujar línea de error posicional (desde car_position_x hasta lane_center)
        cv2.line(bird_view_with_lines, (car_position_x, y_car), (lane_center_int, y_car), (0, 0, 255), 10)  # Línea roja horizontal
        
        # 5. Agregar flechas para indicar dirección
        
        # A. Flecha de CURVATURA (Anticipación) - Magenta
        # Muestra hacia dónde va el carril en el futuro
        arrow_length = 40
        # Nota: curvature_angle_deg es en grados, angle_rad es en radianes
        # Necesitamos convertir curvature_angle_deg a radianes para el cálculo de coordenadas
        curv_rad = math.radians(curvature_angle_deg)
        curv_end_x = int(x_ahead_int + arrow_length * math.sin(curv_rad))
        curv_end_y = int(y_ahead_int - arrow_length * math.cos(curv_rad))
        cv2.arrowedLine(bird_view_with_lines, (x_ahead_int, y_ahead_int), (curv_end_x, curv_end_y), (255, 0, 255), 2, tipLength=0.3)
        # Mostrar valor de curvatura cerca de la flecha magenta
        cv2.putText(bird_view_with_lines, f'{curvature_angle_deg:.1f} deg', (curv_end_x + 10, curv_end_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # B. Flecha de ERROR ANGULAR (Corrección de Posición) - Rojo
        # Muestra cuánto debemos girar solo para corregir la posición actual (sin mirar adelante)
        # La dibujamos desde el coche
        err_rad = math.radians(error_angle_deg)
        err_end_x = int(car_position_x + arrow_length * math.sin(err_rad))
        err_end_y = int(y_car - arrow_length * math.cos(err_rad))
        cv2.arrowedLine(bird_view_with_lines, (car_position_x, y_car), (err_end_x, err_end_y), (0, 0, 255), 2, tipLength=0.3)
        # Mostrar el valor de error angular cerca de la flecha roja
        cv2.putText(bird_view_with_lines, f'{error_angle_deg:.1f} deg', (err_end_x + 10, err_end_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 6. Visualizar el ángulo de desviación FINAL (angle_desviacion_deg) desde la posición del auto
        # Esta es la dirección que el auto debe tomar
        deviation_arrow_length = 80
        deviation_angle_rad = math.radians(angle_desviacion_deg)
        deviation_arrow_end_x = int(car_position_x + deviation_arrow_length * math.sin(deviation_angle_rad))
        deviation_arrow_end_y = int(y_car - deviation_arrow_length * math.cos(deviation_angle_rad))
        cv2.arrowedLine(bird_view_with_lines, (car_position_x, y_car - 15), (deviation_arrow_end_x, deviation_arrow_end_y), (0, 165, 255), 3, tipLength=0.25)
        
        # Agregar texto del ángulo cerca de la flecha
        text_offset_x = int(car_position_x + (deviation_arrow_length * 0.6) * math.sin(deviation_angle_rad))
        text_offset_y = int(y_car - 15 - (deviation_arrow_length * 0.6) * math.cos(deviation_angle_rad))
        cv2.putText(bird_view_with_lines, f'{angle_desviacion_deg:.1f}°', (text_offset_x + 10, text_offset_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
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
        
        # Agregar texto a la vista aérea con líneas
        # Ajustado Y +40px para no solapar con el indicador de Pausa/Frame
        cv2.putText(bird_view_with_lines, f'Result: {angle_desviacion_deg:.2f} deg', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(bird_view_with_lines, f'error_angle_deg: {error_angle_deg:.2f} deg', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(bird_view_with_lines, f'curvature_angle_deg: {curvature_angle_deg:.2f} deg', (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.putText(bird_view_with_lines, f'Lane Center: {lane_center:.1f}', (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(bird_view_with_lines, f'Car Position: {car_position_x}', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Agregar leyenda visual en el lado derecho
        legend_x = 420
        legend_y_start = 70  # Ajustado +40px para no solapar con el indicador de Pausa/Frame
        legend_spacing = 30
        font_scale = 0.5
        font_thickness = 1
        
        # Título de la leyenda
        cv2.putText(bird_view_with_lines, 'Legend:', (legend_x, legend_y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Elementos de la leyenda
        y_offset = legend_y_start + legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (0, 255, 0), -1)
        cv2.putText(bird_view_with_lines, 'Car Position', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (255, 255, 0), -1)
        cv2.putText(bird_view_with_lines, 'Lane Center', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (255, 0, 255), -1)
        cv2.putText(bird_view_with_lines, f'Lookahead: {self.LOOKAHEAD_DISTANCE}', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += legend_spacing
        cv2.line(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 5), (255, 128, 0), 2)
        cv2.putText(bird_view_with_lines, 'Direction', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += legend_spacing
        cv2.line(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 5), (0, 0, 255), 2)
        cv2.putText(bird_view_with_lines, 'Error (px)', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += legend_spacing
        cv2.arrowedLine(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 10), (0, 165, 255), 2, tipLength=0.4)
        cv2.putText(bird_view_with_lines, 'Result (Stanley)', (legend_x + 25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.putText(bird_view_with_lines, f'Lane Width: {self.LANE_WIDTH_PX}px', (legend_x, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (200, 200, 200), font_thickness)

        # Empaquetar imágenes de depuración para mostrarlas fuera
        debug_images = {
            "original": original_frame,
            "bird_view_raw": bird_view_raw,  # Vista aérea sin procesar
            "bird_view_lines": bird_view_with_lines,  # Vista aérea con líneas dibujadas
            "cenital": transformed_frame, # Muestra el overlay verde
            "mask": mask,
            "histogram": histogram_viz,  # Histogram visualization
            "sliding_windows": msk,
            "final_result": result
        }

        return angle_desviacion_deg, debug_images
