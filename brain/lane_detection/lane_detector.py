import cv2
import numpy as np
import math
from abc import ABC, abstractmethod

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

    CAMBIOS IMPLEMENTADOS:
    1) ROI BEV usa todo el ancho (trapecio con top ancho completo).
       + Threshold auto multi-ROI robusto: descarta ROIs outliers por MAD.
    2) Aumenta slices (SLIDING_WINDOW_HEIGHT más chico).
       (EPS variable NO implementado aún, como pediste.)
    3) SOLO RANSAC (sin DBSCAN) para extraer hasta 2 carriles desde all_points.
    4) Matching temporal: costo por residual punto->prev_fit (no por comparar coeficientes).
    """

    def __init__(self, threshold):
        # --- Parámetros de la lógica de tu NUEVO script ---
        self.LANE_WIDTH_PX = 800  # ¡CALIBRAR ESTE VALOR! Ancho del carril en píxeles en vista cenital
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

        # --- Parámetros de slices (AUMENTADO: más slices -> menor altura por slice) ---
        self.SLIDING_WINDOW_START_Y = 472
        self.SLIDING_WINDOW_HEIGHT = 10  # antes 20 -> más slices
        self.SLIDING_WINDOW_WIDTH = 50
        self.SLIDING_WINDOW_EXPANDED_WIDTH = 150
        self.ENABLE_EXPANDED_SEARCH = False
        self.HISTOGRAM_PEAK_THRESHOLD = 3000
        self.HISTOGRAM_SMOOTH_KERNEL_SIZE = 9

        # --- Parámetros de slices por hemisferio ---
        self.HEMISLICE_MIN_CONTOUR_AREA = 20
        self.HEMISLICE_MAX_JUMP_PX = 120
        self.HEMISLICE_TREND_TOLERANCE_PX = 90

        # --- RANSAC (reemplaza DBSCAN) ---
        self.RANSAC_DEGREE = 2
        self.RANSAC_MAX_ITERS = 220
        self.RANSAC_INLIER_THRESH_PX = 10.0
        self.RANSAC_MIN_INLIERS = 14
        self.RANSAC_REFIT_MIN_INLIERS = 8

        # --- Selección ego-lane / tracking temporal ---
        self.DEBUG_LANE_CLUSTER_SELECTION = False  # reutilizo flag para debug selección (ahora ransac)
        self.ENABLE_TEMPORAL_CLUSTER_MATCHING = True
        self.CLUSTER_MATCH_Y_SAMPLES = np.array([480, 420, 360, 300], dtype=np.float32)  # se usa para side penalty, etc.
        self.CLUSTER_MATCH_MAX_COST = 180.0
        self.CLUSTER_SWAP_PENALTY = 40.0
        self.CLUSTER_CENTER_DEADBAND_PX = 15.0

        # --- Threshold automático por múltiples ROIs + suavizado temporal ---
        self.AUTO_THR_REF_X_NORMS = [0.2, 0.5, 0.8]
        self.AUTO_THR_REF_Y_FROM_BOTTOM_PX = 8
        self.AUTO_THR_REF_ROI_SIZE = 41
        self.AUTO_THR_BG_PERCENTILE = 90.0
        self.AUTO_THR_OFFSET = 45
        self.AUTO_THR_TEMPORAL_ALPHA = 0.30
        self.AUTO_THR_MAX_DELTA_PER_FRAME = 12

        # NUEVO: rechazo de outliers en bg_values (MAD)
        self.AUTO_THR_OUTLIER_K = 3.0
        self.AUTO_THR_OUTLIER_FALLBACK_T = 18.0

        self.prev_auto_threshold = None

        # --- Parámetros de Control ---
        self.curvature_factor = 0.5
        self.error_factor = 0.3

        # --- Puntos de perspectiva ---
        # CAMBIO (1): ROI usa todo el ancho arriba (top full width)
        # Antes:
        #   tl=(160,180), tr=(480,180)
        # Ahora:
        self.tl = (0, 180)
        self.tr = (640, 180)
        self.bl = (0, 450)
        self.br = (640, 450)
        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        # --- Valores de color HSV ---
        self.hsv_lower = np.array([0, 0, threshold])
        self.hsv_upper = np.array([255, 50, 255])

    # -----------------------------
    # Helpers robustos / RANSAC
    # -----------------------------

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _compute_auto_threshold_from_multi_roi(self, gray):
        """Calcula threshold con múltiples ROIs inferiores, robusto + rechazo outliers (MAD)."""
        h, w = gray.shape[:2]
        cy = int(self._clamp(h - 1 - self.AUTO_THR_REF_Y_FROM_BOTTOM_PX, 0, h - 1))
        r = max(1, self.AUTO_THR_REF_ROI_SIZE // 2)

        rois = []
        bg_values = []

        for x_norm in self.AUTO_THR_REF_X_NORMS:
            cx = int(self._clamp(x_norm, 0.0, 1.0) * (w - 1))
            x0 = self._clamp(cx - r, 0, w - 1)
            x1 = self._clamp(cx + r, 0, w - 1)
            y0 = self._clamp(cy - r, 0, h - 1)
            y1 = self._clamp(cy + r, 0, h - 1)

            roi = gray[y0:y1 + 1, x0:x1 + 1]
            if roi.size == 0:
                continue

            bg = float(np.percentile(roi, self._clamp(self.AUTO_THR_BG_PERCENTILE, 0.0, 100.0)))
            bg_values.append(bg)
            rois.append((x0, y0, x1, y1))

        if len(bg_values) == 0:
            raw_thr = 128
            return raw_thr, rois

        bg_values = np.array(bg_values, dtype=np.float32)
        m = float(np.median(bg_values))
        mad = float(np.median(np.abs(bg_values - m)))

        if mad <= 1e-6:
            # fallback simple
            T = float(max(0.0, self.AUTO_THR_OUTLIER_FALLBACK_T))
            keep = np.abs(bg_values - m) <= T
        else:
            k = float(max(0.5, self.AUTO_THR_OUTLIER_K))
            keep = np.abs(bg_values - m) <= (k * mad)

        filtered = bg_values[keep]
        if filtered.size == 0:
            robust_bg = m
        else:
            robust_bg = float(np.median(filtered))

        raw_thr = int(round(robust_bg + float(self.AUTO_THR_OFFSET)))
        raw_thr = int(self._clamp(raw_thr, 0, 255))
        return raw_thr, rois

    @staticmethod
    def _poly_eval_x_of_y(fit, y):
        return fit[0] * y * y + fit[1] * y + fit[2]

    def _ransac_fit_quadratic_x_of_y(self, pts_xy):
        """
        RANSAC para ajustar x(y) cuadrático.
        pts_xy: array Nx2 con columnas [x, y] (float32/float64)
        Retorna: (best_fit, inlier_mask)
        """
        if pts_xy is None or len(pts_xy) < self.MIN_POINTS_FOR_FIT:
            return None, None

        pts = np.asarray(pts_xy, dtype=np.float32)
        x = pts[:, 0]
        y = pts[:, 1]

        n = len(pts)
        if n < 3:
            return None, None

        best_inliers = None
        best_fit = None
        best_score = -1
        best_err = float("inf")

        thr = float(max(1.0, self.RANSAC_INLIER_THRESH_PX))
        iters = int(max(20, self.RANSAC_MAX_ITERS))

        rng = np.random.default_rng()

        for _ in range(iters):
            # sample 3 unique points
            idx = rng.choice(n, size=3, replace=False)
            ys = y[idx]
            xs = x[idx]
            try:
                fit = np.polyfit(ys, xs, 2)
            except np.linalg.LinAlgError:
                continue
            # residuals
            x_hat = fit[0] * y * y + fit[1] * y + fit[2]
            res = np.abs(x - x_hat)
            inliers = res <= thr
            cnt = int(np.sum(inliers))
            if cnt > best_score:
                best_score = cnt
                best_inliers = inliers
                best_fit = fit
                best_err = float(np.mean(res[inliers])) if cnt > 0 else float("inf")
            elif cnt == best_score and cnt > 0:
                err = float(np.mean(res[inliers]))
                if err < best_err:
                    best_err = err
                    best_inliers = inliers
                    best_fit = fit

        if best_fit is None or best_inliers is None:
            return None, None

        if int(np.sum(best_inliers)) < int(self.RANSAC_MIN_INLIERS):
            return None, None

        # Refit con inliers (más estable)
        inlier_pts = pts[best_inliers]
        if len(inlier_pts) >= int(self.RANSAC_REFIT_MIN_INLIERS):
            try:
                best_fit = np.polyfit(inlier_pts[:, 1], inlier_pts[:, 0], 2)
            except np.linalg.LinAlgError:
                pass

        return best_fit, best_inliers

    def _extract_two_lanes_ransac(self, all_points_xy):
        """
        Extrae hasta 2 carriles usando RANSAC iterativo:
        - Ajusta un carril
        - Remueve inliers
        - Ajusta el segundo

        Retorna lista de candidatos: [{'fit', 'points', 'centroid_x', 'support'}]
        """
        if all_points_xy is None or len(all_points_xy) < max(self.RANSAC_MIN_INLIERS, 6):
            return []

        pts = np.array(all_points_xy, dtype=np.float32)
        remaining = pts.copy()

        candidates = []
        for _ in range(2):
            fit, inliers = self._ransac_fit_quadratic_x_of_y(remaining)
            if fit is None or inliers is None:
                break
            inlier_pts = remaining[inliers]
            support = int(len(inlier_pts))
            if support < self.MIN_POINTS_FOR_FIT:
                break
            centroid_x = float(np.mean(inlier_pts[:, 0]))
            candidates.append({
                'fit': fit,
                'points': inlier_pts,
                'centroid_x': centroid_x,
                'support': support
            })
            # remove inliers
            remaining = remaining[~inliers]
            if len(remaining) < max(self.RANSAC_MIN_INLIERS, 6):
                break

        return candidates

    def _temporal_residual_cost_points_to_prevfit(self, pts_xy, prev_fit):
        """
        CAMBIO (4): costo temporal como residual punto->prev_fit:
            mean(|x_pts - prevfit(y_pts)|)
        """
        if prev_fit is None or pts_xy is None or len(pts_xy) == 0:
            return float('inf')
        pts = np.asarray(pts_xy, dtype=np.float32)
        y = pts[:, 1]
        x = pts[:, 0]
        x_prev = prev_fit[0] * y * y + prev_fit[1] * y + prev_fit[2]
        return float(np.mean(np.abs(x - x_prev)))

    # ======================================================================
    # -------------------------- MAIN PIPELINE ------------------------------
    # ======================================================================

    def get_lane_metrics(self, frame):
        # --- Redimensionar y aplicar Vista Cenital ---
        frame = cv2.resize(frame, (640, 480))
        original_frame = frame.copy()

        transformed_frame = cv2.warpPerspective(frame, self.matrix, (640, 480))

        # --- Threshold automático por ROI robusto ---
        gray_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY)
        raw_auto_thr, auto_thr_rois = self._compute_auto_threshold_from_multi_roi(gray_transformed_frame)

        if self.prev_auto_threshold is None:
            auto_thr = raw_auto_thr
        else:
            alpha = float(self._clamp(self.AUTO_THR_TEMPORAL_ALPHA, 0.0, 1.0))
            ema_thr = (1.0 - alpha) * float(self.prev_auto_threshold) + alpha * float(raw_auto_thr)
            delta_max = float(max(0, self.AUTO_THR_MAX_DELTA_PER_FRAME))
            low = float(self.prev_auto_threshold) - delta_max
            high = float(self.prev_auto_threshold) + delta_max
            auto_thr = int(round(self._clamp(ema_thr, low, high)))

        auto_thr = int(self._clamp(auto_thr, 0, 255))
        self.prev_auto_threshold = auto_thr
        _, mask = cv2.threshold(gray_transformed_frame, auto_thr, 255, cv2.THRESH_BINARY)

        # --- Histograma (debug) ---
        histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)

        # --- Slices: extraer candidatos y construir all_points ---
        y = self.SLIDING_WINDOW_START_Y
        msk = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)
        h, w = mask.shape[:2]

        histogram_viz = np.zeros((100, mask.shape[1], 3), dtype=np.uint8)
        histogram_normalized = (histogram / histogram.max() * 100).astype(int) if histogram.max() > 0 else histogram
        for i, hv in enumerate(histogram_normalized):
            if hv > 0:
                cv2.line(histogram_viz, (i, 100), (i, 100 - int(hv)), (255, 255, 255), 1)
        cv2.line(histogram_viz, (midpoint, 0), (midpoint, 100), (0, 255, 255), 2)

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

        while y > 0:
            y0 = max(0, y - self.SLIDING_WINDOW_HEIGHT)
            y1 = y

            full_slice = mask[y0:y1, :]
            contours, _ = cv2.findContours(full_slice, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            candidates = _extract_candidates(contours)

            found_any = len(candidates) > 0
            for cx, cy_local, _ in candidates:
                point_y = y0 + cy_local
                all_points.append((float(cx), float(point_y)))
                cv2.circle(msk, (int(cx), int(point_y)), 2, (180, 180, 180), -1)

            slice_color = (0, 255, 0) if found_any else (0, 0, 255)
            cv2.rectangle(msk, (0, y1), (w - 1, y0), slice_color, 1)
            cv2.line(msk, (midpoint, y0), (midpoint, y1), (80, 80, 80), 1)
            cv2.putText(msk, f'S{window_index}', (8, max(15, y0 + 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            y -= self.SLIDING_WINDOW_HEIGHT
            window_index += 1

        # ======================================================================
        # 3) SOLO RANSAC (sin DBSCAN): extraer hasta 2 carriles
        # ======================================================================
        selected_left_points = []
        selected_right_points = []
        left_base = -1
        right_base = -1
        selected_left_score = None
        selected_right_score = None
        left_errors = []
        right_errors = []

        candidates = self._extract_two_lanes_ransac(all_points)

        if self.DEBUG_LANE_CLUSTER_SELECTION:
            print(f"[RANSAC] candidates={len(candidates)}")
            for i, c in enumerate(candidates):
                print(f"  cand{i}: support={c['support']} centroid_x={c['centroid_x']:.1f}")

        reference_center_x = w / 2.0

        # Selección con matching temporal + penalización por lado
        left_cand = None
        right_cand = None

        def _side_penalty(side, centroid_x):
            deadband = float(max(0.0, self.CLUSTER_CENTER_DEADBAND_PX))
            if abs(centroid_x - reference_center_x) <= deadband:
                return 0.0
            if side == 'L':
                return 0.0 if centroid_x < reference_center_x else float(self.CLUSTER_SWAP_PENALTY)
            return 0.0 if centroid_x > reference_center_x else float(self.CLUSTER_SWAP_PENALTY)

        def _cost_for_side(cand, side):
            prev_fit = self.prev_left_fit if side == 'L' else self.prev_right_fit
            if self.ENABLE_TEMPORAL_CLUSTER_MATCHING and prev_fit is not None:
                temporal = self._temporal_residual_cost_points_to_prevfit(cand['points'], prev_fit)
                if not np.isfinite(temporal):
                    temporal = float('inf')
            else:
                temporal = float('inf')

            # fallback si no hay temporal: distancia al centro
            if not np.isfinite(temporal) or temporal == float('inf'):
                temporal = abs(float(cand['centroid_x']) - reference_center_x)

            return float(temporal) + _side_penalty(side, float(cand['centroid_x']))

        # Asignación óptima (bruteforce) con hasta 2 candidatos
        if len(candidates) == 0:
            left_errors.append("ERR_NO_RANSAC_CANDIDATES")
            right_errors.append("ERR_NO_RANSAC_CANDIDATES")
        elif len(candidates) == 1:
            c = candidates[0]
            deadband = float(max(0.0, self.CLUSTER_CENTER_DEADBAND_PX))
            if c['centroid_x'] < (reference_center_x - deadband):
                left_cand = c
                selected_left_score = _cost_for_side(c, 'L')
            elif c['centroid_x'] > (reference_center_x + deadband):
                right_cand = c
                selected_right_score = _cost_for_side(c, 'R')
            else:
                # neutro: asignar al lado con menor costo
                cL = _cost_for_side(c, 'L')
                cR = _cost_for_side(c, 'R')
                if cL <= cR:
                    left_cand = c
                    selected_left_score = cL
                else:
                    right_cand = c
                    selected_right_score = cR
                left_errors.append("WARN_SINGLE_NEUTRAL_CAND")
                right_errors.append("WARN_SINGLE_NEUTRAL_CAND")
        else:
            c0, c1 = candidates[0], candidates[1]

            # dos posibles asignaciones
            cost_L0_R1 = _cost_for_side(c0, 'L') + _cost_for_side(c1, 'R')
            cost_L1_R0 = _cost_for_side(c1, 'L') + _cost_for_side(c0, 'R')

            if cost_L0_R1 <= cost_L1_R0:
                left_cand, right_cand = c0, c1
                selected_left_score = _cost_for_side(c0, 'L')
                selected_right_score = _cost_for_side(c1, 'R')
            else:
                left_cand, right_cand = c1, c0
                selected_left_score = _cost_for_side(c1, 'L')
                selected_right_score = _cost_for_side(c0, 'R')

            # sanity: evitar que ambos queden del mismo lado con fuerte penalización
            deadband = float(max(0.0, self.CLUSTER_CENTER_DEADBAND_PX))
            if left_cand is not None and left_cand['centroid_x'] > (reference_center_x + deadband):
                left_errors.append("WARN_LEFT_ASSIGNED_RIGHT_SIDE")
            if right_cand is not None and right_cand['centroid_x'] < (reference_center_x - deadband):
                right_errors.append("WARN_RIGHT_ASSIGNED_LEFT_SIDE")

        # Convertir a puntos seleccionados + bases
        if left_cand is not None:
            selected_left_points = [tuple(map(int, p)) for p in left_cand['points']]
            left_base = int(round(left_cand['centroid_x']))
        else:
            left_errors.append("ERR_LEFT_NOT_ASSIGNED")

        if right_cand is not None:
            selected_right_points = [tuple(map(int, p)) for p in right_cand['points']]
            right_base = int(round(right_cand['centroid_x']))
        else:
            right_errors.append("ERR_RIGHT_NOT_ASSIGNED")

        # debug overlay centroides y candidatos
        if left_cand is not None:
            cv2.circle(msk, (left_base, 38), 6, (0, 0, 255), -1)
            cv2.putText(msk, "L", (left_base + 6, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if right_cand is not None:
            cv2.circle(msk, (right_base, 56), 6, (255, 0, 0), -1)
            cv2.putText(msk, "R", (right_base + 6, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # histogram markers
        if left_base != -1:
            cv2.circle(histogram_viz, (left_base, 50), 8, (0, 0, 255), -1)
        if right_base != -1:
            cv2.circle(histogram_viz, (right_base, 50), 8, (255, 0, 0), -1)

        # line connections (debug)
        lx = [p[0] for p in selected_left_points]
        ly = [p[1] for p in selected_left_points]
        rx = [p[0] for p in selected_right_points]
        ry = [p[1] for p in selected_right_points]

        if len(lx) > 1:
            for i in range(len(lx) - 1):
                cv2.line(msk, (lx[i], ly[i]), (lx[i + 1], ly[i + 1]), (0, 255, 255), 1)
        if len(rx) > 1:
            for i in range(len(rx) - 1):
                cv2.line(msk, (rx[i], ry[i]), (rx[i + 1], ry[i + 1]), (0, 255, 255), 1)

        # stats overlay
        stats_y = 70
        cv2.putText(msk, f'Left points: {len(lx)}', (10, stats_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        stats_y += 30
        cv2.putText(msk, f'Right points: {len(rx)}', (10, stats_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        stats_y += 30
        cv2.putText(msk, f'Candidates: {len(candidates)}', (10, stats_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        stats_y += 30
        cv2.putText(msk, f'Auto thr: {auto_thr}', (10, stats_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        stats_y += 30
        cv2.putText(msk, f'ROIs: {auto_thr_rois}', (10, stats_y),
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
        stats_y += 25
        if len(left_errors) > 0:
            left_error_text = ",".join(left_errors[:2])
            cv2.putText(msk, f'L ERR: {left_error_text}',
                        (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 140, 255), 1)
            stats_y += 20
        if len(right_errors) > 0:
            right_error_text = ",".join(right_errors[:2])
            cv2.putText(msk, f'R ERR: {right_error_text}',
                        (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 140, 255), 1)

        # ======================================================================
        # --- Polyfit y Lógica de Estimación ---
        # ======================================================================
        left_fit_current = None
        right_fit_current = None

        MIDPOINT_X = 320  # Mitad de la imagen (640/2)

        # 1. Filtrar puntos por lado (opcional)
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

        if len(ly) >= self.MIN_POINTS_FOR_FIT:
            try:
                left_fit_current = np.polyfit(ly, lx, 2)
            except np.linalg.LinAlgError:
                pass

        if len(ry) >= self.MIN_POINTS_FOR_FIT:
            try:
                right_fit_current = np.polyfit(ry, rx, 2)
            except np.linalg.LinAlgError:
                pass

        # [NUEVO] FILTRO DE POSICIÓN (ZONA DE EXCLUSIÓN)
        if self.ENABLE_BASE_EXCLUSION_FILTER:
            if left_fit_current is not None:
                lx_base = left_fit_current[0] * 480**2 + left_fit_current[1] * 480 + left_fit_current[2]
                if lx_base > MIDPOINT_X:
                    print(f"🚫 RECHAZADO: Falso Izquierdo en zona derecha (x={int(lx_base)})")
                    left_fit_current = None

            if right_fit_current is not None:
                rx_base = right_fit_current[0] * 480**2 + right_fit_current[1] * 480 + right_fit_current[2]
                if rx_base < MIDPOINT_X:
                    print(f"🚫 RECHAZADO: Falso Derecho en zona izquierda (x={int(rx_base)})")
                    right_fit_current = None

        def get_line_distance(fit1, fit2):
            y_ref = 480
            x1 = fit1[0] * y_ref**2 + fit1[1] * y_ref + fit1[2]
            x2 = fit2[0] * y_ref**2 + fit2[1] * y_ref + fit2[2]
            return abs(x2 - x1)

        def get_curvature_angle(fit):
            y_current = 480
            y_ahead = max(0, 480 - self.LOOKAHEAD_DISTANCE)
            x_current = fit[0] * y_current**2 + fit[1] * y_current + fit[2]
            x_ahead = fit[0] * y_ahead**2 + fit[1] * y_ahead + fit[2]
            dx = x_ahead - x_current
            dy = y_ahead - y_current
            angle_rad = math.atan2(dx, -dy)
            return math.degrees(angle_rad)

        def lines_intersect(fit1, fit2, y_start=0, y_end=480):
            a = fit1[0] - fit2[0]
            b = fit1[1] - fit2[1]
            c = fit1[2] - fit2[2]

            if abs(a) < 1e-6:
                if abs(b) < 1e-6:
                    return False
                y_intersect = -c / b
                return y_start <= y_intersect <= y_end

            delta = b**2 - 4 * a * c
            if delta < 0:
                return False

            sqrt_delta = math.sqrt(delta)
            y1 = (-b + sqrt_delta) / (2 * a)
            y2 = (-b - sqrt_delta) / (2 * a)
            return (y_start <= y1 <= y_end) or (y_start <= y2 <= y_end)

        # =========================================================
        # --- LÓGICA DE DECISIÓN (ÁRBOL JERÁRQUICO) ---
        # =========================================================
        detection_mode = "NONE"
        final_left_fit = None
        final_right_fit = None

        if left_fit_current is not None and right_fit_current is not None:
            distance = get_line_distance(left_fit_current, right_fit_current)
            intersect = lines_intersect(left_fit_current, right_fit_current)
            distance_ok = (distance >= self.MIN_LANE_DISTANCE_PX) if self.ENABLE_MIN_LANE_DISTANCE_CHECK else True

            if distance_ok and not intersect:
                detection_mode = "STEREO"
                final_left_fit = left_fit_current
                final_right_fit = right_fit_current
                self.prev_left_fit = final_left_fit
                self.prev_right_fit = final_right_fit

        if detection_mode == "NONE":
            mono_candidates = []

            def _fit_temporal_distance(candidate_fit, prev_fit):
                if candidate_fit is None:
                    return float('inf')
                if prev_fit is None:
                    return 0.0
                y_samples = self.CLUSTER_MATCH_Y_SAMPLES
                x_candidate = candidate_fit[0] * y_samples**2 + candidate_fit[1] * y_samples + candidate_fit[2]
                x_prev = prev_fit[0] * y_samples**2 + prev_fit[1] * y_samples + prev_fit[2]
                return float(np.mean(np.abs(x_candidate - x_prev)))

            if right_fit_current is not None:
                mono_right_fit = right_fit_current
                curvature = abs(get_curvature_angle(mono_right_fit))
                lane_width = self.LANE_WIDTH_PX if curvature >= self.CURVATURE_THRESHOLD else (self.LANE_WIDTH_PX - self.STRAIGHT_LANE_WIDTH_REDUCTION)
                reconstructed_left = mono_right_fit - [0, 0, lane_width]
                right_score = _fit_temporal_distance(mono_right_fit, self.prev_right_fit)
                left_score = _fit_temporal_distance(reconstructed_left, self.prev_left_fit)
                mono_candidates.append((right_score + left_score, "MONO_RIGHT", reconstructed_left, mono_right_fit))

            if left_fit_current is not None:
                mono_left_fit = left_fit_current
                curvature = abs(get_curvature_angle(mono_left_fit))
                lane_width = self.LANE_WIDTH_PX if curvature >= self.CURVATURE_THRESHOLD else (self.LANE_WIDTH_PX - self.STRAIGHT_LANE_WIDTH_REDUCTION)
                reconstructed_right = mono_left_fit + [0, 0, lane_width]
                left_score = _fit_temporal_distance(mono_left_fit, self.prev_left_fit)
                right_score = _fit_temporal_distance(reconstructed_right, self.prev_right_fit)
                mono_candidates.append((left_score + right_score, "MONO_LEFT", mono_left_fit, reconstructed_right))

            if mono_candidates:
                mono_candidates.sort(key=lambda item: item[0])
                _, selected_mode, selected_left, selected_right = mono_candidates[0]
                detection_mode = selected_mode
                final_left_fit = selected_left
                final_right_fit = selected_right
                self.prev_left_fit = final_left_fit
                self.prev_right_fit = final_right_fit

        if detection_mode == "NONE":
            if self.ENABLE_MEMORY_MODE and self.prev_left_fit is not None and self.prev_right_fit is not None:
                detection_mode = "MEMORY"
                final_left_fit = self.prev_left_fit
                final_right_fit = self.prev_right_fit
            else:
                bird_view_with_lines = transformed_frame.copy()
                no_lane_text = "NO LANE DETECTED" if self.ENABLE_MEMORY_MODE else "NO LANE DETECTED (MEMORY OFF)"
                cv2.putText(bird_view_with_lines, no_lane_text, (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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

        left_fit = final_left_fit
        right_fit = final_right_fit

        # --- CÁLCULO DEL ÁNGULO DE DESVIACIÓN ---
        center_fit = [(left_fit[0] + right_fit[0]) / 2,
                      (left_fit[1] + right_fit[1]) / 2,
                      (left_fit[2] + right_fit[2]) / 2]

        y_car = 480
        car_position_x = 320

        lane_center = center_fit[0] * y_car**2 + center_fit[1] * y_car + center_fit[2]
        error_pixels = lane_center - car_position_x

        y_current = y_car
        y_ahead = max(0, y_car - self.LOOKAHEAD_DISTANCE)

        x_current = center_fit[0] * y_current**2 + center_fit[1] * y_current + center_fit[2]
        x_ahead = center_fit[0] * y_ahead**2 + center_fit[1] * y_ahead + center_fit[2]

        dx = x_ahead - x_current
        dy = y_ahead - y_current
        angle_rad = math.atan2(dx, -dy)
        curvature_angle_deg = math.degrees(angle_rad)
        curvature_angle_deg = max(min(curvature_angle_deg, 30), -30)

        error_angle_rad = math.atan2(error_pixels, self.LOOKAHEAD_DISTANCE)
        error_angle_deg = math.degrees(error_angle_rad)

        angle_desviacion_deg = self.error_factor * error_angle_deg + self.curvature_factor * curvature_angle_deg
        angle_desviacion_deg = max(min(angle_desviacion_deg, 30), -30)

        # --- Visualización ---
        bird_view_raw = transformed_frame.copy()
        bird_view_with_lines = transformed_frame.copy()

        plot_y = np.linspace(0, 479, 480)
        plot_x_left = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        plot_x_right = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]

        color_left = (0, 0, 255)
        color_right = (255, 0, 0)
        color_reconstructed = (150, 150, 150)
        color_memory = (0, 255, 255)

        if detection_mode == "STEREO":
            color_left = (0, 0, 255)
            color_right = (255, 0, 0)
        elif detection_mode == "MONO_RIGHT":
            color_left = color_reconstructed
            color_right = (255, 0, 0)
        elif detection_mode == "MONO_LEFT":
            color_left = (0, 0, 255)
            color_right = color_reconstructed
        elif detection_mode == "MEMORY":
            color_left = color_memory
            color_right = color_memory

        pts_left = np.vstack((plot_x_left, plot_y)).astype(np.int32).T
        pts_right = np.vstack((plot_x_right, plot_y)).astype(np.int32).T
        cv2.polylines(bird_view_with_lines, [pts_left], False, color_left, 3)
        cv2.polylines(bird_view_with_lines, [pts_right], False, color_right, 3)

        center_line_x = (plot_x_left + plot_x_right) / 2
        for i in range(len(plot_y) - 1):
            cv2.line(bird_view_with_lines,
                     (int(center_line_x[i]), int(plot_y[i])),
                     (int(center_line_x[i + 1]), int(plot_y[i + 1])),
                     (0, 255, 255), 2)

        cv2.circle(bird_view_with_lines, (320, 480), 10, (0, 255, 0), -1)
        cv2.putText(bird_view_with_lines, f"MODE: {detection_mode}", (10, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        lane_center_int = int(lane_center)
        cv2.circle(bird_view_with_lines, (lane_center_int, y_car), 8, (255, 255, 0), -1)

        x_ahead_int = int(x_ahead)
        y_ahead_int = int(y_ahead)
        cv2.circle(bird_view_with_lines, (x_ahead_int, y_ahead_int), 8, (255, 0, 255), -1)

        x_current_int = int(x_current)
        cv2.line(bird_view_with_lines, (x_current_int, y_car), (x_ahead_int, y_ahead_int), (255, 128, 0), 3)
        cv2.line(bird_view_with_lines, (car_position_x, y_car), (lane_center_int, y_car), (0, 0, 255), 10)

        arrow_length = 40
        curv_rad = math.radians(curvature_angle_deg)
        curv_end_x = int(x_ahead_int + arrow_length * math.sin(curv_rad))
        curv_end_y = int(y_ahead_int - arrow_length * math.cos(curv_rad))
        cv2.arrowedLine(bird_view_with_lines, (x_ahead_int, y_ahead_int), (curv_end_x, curv_end_y),
                        (255, 0, 255), 2, tipLength=0.3)
        cv2.putText(bird_view_with_lines, f'{curvature_angle_deg:.1f} deg', (curv_end_x + 10, curv_end_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        err_rad = math.radians(error_angle_deg)
        err_end_x = int(car_position_x + arrow_length * math.sin(err_rad))
        err_end_y = int(y_car - arrow_length * math.cos(err_rad))
        cv2.arrowedLine(bird_view_with_lines, (car_position_x, y_car), (err_end_x, err_end_y),
                        (0, 0, 255), 2, tipLength=0.3)
        cv2.putText(bird_view_with_lines, f'{error_angle_deg:.1f} deg', (err_end_x + 10, err_end_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        deviation_arrow_length = 80
        deviation_angle_rad = math.radians(angle_desviacion_deg)
        deviation_arrow_end_x = int(car_position_x + deviation_arrow_length * math.sin(deviation_angle_rad))
        deviation_arrow_end_y = int(y_car - deviation_arrow_length * math.cos(deviation_angle_rad))
        cv2.arrowedLine(bird_view_with_lines, (car_position_x, y_car - 15),
                        (deviation_arrow_end_x, deviation_arrow_end_y),
                        (0, 165, 255), 3, tipLength=0.25)

        text_offset_x = int(car_position_x + (deviation_arrow_length * 0.6) * math.sin(deviation_angle_rad))
        text_offset_y = int(y_car - 15 - (deviation_arrow_length * 0.6) * math.cos(deviation_angle_rad))
        cv2.putText(bird_view_with_lines, f'{angle_desviacion_deg:.1f}°', (text_offset_x + 10, text_offset_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        overlay = transformed_frame.copy()
        points_left = np.array([np.transpose(np.vstack([plot_x_left, plot_y]))])
        points_right = np.array([np.flipud(np.transpose(np.vstack([plot_x_right, plot_y])))])
        quad_points = np.hstack((points_left, points_right)).astype(np.int32)

        cv2.fillPoly(overlay, [quad_points], (0, 255, 0))
        cv2.addWeighted(overlay, 0.4, transformed_frame, 0.8, 0, transformed_frame)

        original_perpective_lane_image = cv2.warpPerspective(transformed_frame, self.inv_matrix, (640, 480))
        result = cv2.addWeighted(original_frame, 1, original_perpective_lane_image, 0.5, 0)

        cv2.putText(bird_view_with_lines, f'Result: {angle_desviacion_deg:.2f} deg', (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(bird_view_with_lines, f'error_angle_deg: {error_angle_deg:.2f} deg', (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(bird_view_with_lines, f'curvature_angle_deg: {curvature_angle_deg:.2f} deg', (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.putText(bird_view_with_lines, f'Lane Center: {lane_center:.1f}', (10, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(bird_view_with_lines, f'Car Position: {car_position_x}', (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        legend_x = 420
        legend_y_start = 70
        legend_spacing = 30
        font_scale = 0.5
        font_thickness = 1

        cv2.putText(bird_view_with_lines, 'Legend:', (legend_x, legend_y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        y_offset = legend_y_start + legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (0, 255, 0), -1)
        cv2.putText(bird_view_with_lines, 'Car Position', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (255, 255, 0), -1)
        cv2.putText(bird_view_with_lines, 'Lane Center', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.circle(bird_view_with_lines, (legend_x + 10, y_offset - 5), 5, (255, 0, 255), -1)
        cv2.putText(bird_view_with_lines, f'Lookahead: {self.LOOKAHEAD_DISTANCE}', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.line(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 5), (255, 128, 0), 2)
        cv2.putText(bird_view_with_lines, 'Direction', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.line(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 5), (0, 0, 255), 2)
        cv2.putText(bird_view_with_lines, 'Error (px)', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.arrowedLine(bird_view_with_lines, (legend_x + 5, y_offset - 5), (legend_x + 20, y_offset - 10),
                        (0, 165, 255), 2, tipLength=0.4)
        cv2.putText(bird_view_with_lines, 'Result (Stanley)', (legend_x + 25, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        y_offset += legend_spacing
        cv2.putText(bird_view_with_lines, f'Lane Width: {self.LANE_WIDTH_PX}px', (legend_x, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (200, 200, 200), font_thickness)

        debug_images = {
            "original": original_frame,
            "bird_view_raw": bird_view_raw,
            "bird_view_lines": bird_view_with_lines,
            "cenital": transformed_frame,
            "mask": mask,
            "histogram": histogram_viz,
            "sliding_windows": msk,
            "final_result": result
        }

        return angle_desviacion_deg, debug_images
