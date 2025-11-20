#!/usr/bin/env python3
"""
Script para detección YOLO desde cámara con visualización en ventana.
Muestra detecciones en tiempo real con bounding boxes.
Usa el modelo: weights/merged/best.pt
"""

import os
import cv2
import torch
from pathlib import Path
from ultralytics import YOLO
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import detect_os, choose_camera_by_OS, handle_video_capture


def detectar_y_configurar_gpu():
    """Detecta GPU y configura para máximo rendimiento"""
    if torch.cuda.is_available():
        device = "cuda"
        torch.backends.cudnn.benchmark = True
        return device
    return "cpu"


def cargar_modelo(model_path, device):
    """Carga el modelo YOLO, priorizando engine si existe"""
    if not os.path.exists(model_path):
        print(f"❌ Modelo no encontrado: {model_path}")
        return None
    
    engine_path = model_path.replace('.pt', '.engine')
    
    # Priorizar engine si existe
    if os.path.exists(engine_path):
        print(f"✅ Engine file encontrado: {engine_path}")
        print("📦 Usando modelo TensorRT optimizado")
        # Los modelos TensorRT ya están optimizados para GPU, no necesitan .to(device)
        model = YOLO(engine_path, task='detect')
        return model
    
    # Cargar .pt y exportar a engine si hay GPU
    model = YOLO(model_path, task='detect')
    if device == "cuda":
        model.to(device)
        if not os.path.exists(engine_path):
            try:
                model.export(format='engine')
                if os.path.exists(engine_path):
                    # Recargar como engine (no necesita .to(device))
                    model = YOLO(engine_path, task='detect')
            except:
                pass  # Continuar con .pt si falla exportación
    
    return model


def procesar_camara(model, device):
    """Procesa video desde la cámara web con visualización en ventana"""
    os_detected = detect_os()
    camera_path = choose_camera_by_OS()
    
    print("📹 Iniciando detección con visualización... (Presiona 'q' para salir)")
    print(f"🔍 Sistema operativo detectado: {os_detected}")
    print(f"📷 Usando path de cámara: {camera_path}")
    print(f"🚀 Dispositivo: {device}")
    
    # Abrir cámara con ventana
    cap = handle_video_capture("Detección en tiempo real - Cámara", camera_path)
    
    if cap is None:
        print("❌ Error: No se pudo abrir la cámara.")
        return
    
    print("🎥 Cámara iniciada. Presiona 'q' para salir...")
    print("📊 Umbral de confianza: 60%")
    print("-" * 80)
    
    frame_count = 0
    confidence_threshold = 0.6
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ Error al capturar frame de la cámara.")
                break
            
            frame_count += 1
            
            # Inferencia en GPU
            results = model(frame, verbose=False, device=device)[0]
            
            # Dibujar detecciones en el frame
            detections_found = False
            if results.boxes is not None and len(results.boxes) > 0:
                boxes = results.boxes
                for i in range(len(boxes)):
                    confidence = float(boxes.conf[i])
                    if confidence >= confidence_threshold:
                        detections_found = True
                        # Obtener coordenadas del bounding box
                        box = boxes.xyxy[i].cpu().numpy()  # [x1, y1, x2, y2]
                        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                        cls = int(boxes.cls[i])
                        class_name = model.names[cls]
                        
                        # Dibujar bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Dibujar label con fondo
                        label = f"{class_name} {confidence:.1%}"
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        label_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
                        cv2.rectangle(frame, (x1, label_y - label_size[1] - 5), 
                                    (x1 + label_size[0], label_y + 5), (0, 255, 0), -1)
                        cv2.putText(frame, label, (x1, label_y), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                        
                        # Imprimir en consola
                        print(f"Frame {frame_count}: {class_name} {confidence:.2%} bbox(x1={box[0]:.1f}, y1={box[1]:.1f}, x2={box[2]:.1f}, y2={box[3]:.1f})")
            
            if not detections_found:
                print(f"Frame {frame_count}: Sin detecciones")
            
            # Mostrar frame en ventana
            cv2.imshow('Detección en tiempo real - Cámara', frame)
            
            # Salir si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except KeyboardInterrupt:
        print("\n⏹️  Deteniendo...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("✅ Finalizado.")


def main():
    """Función principal"""
    script_dir = Path(__file__).parent
    model_path = script_dir.parent / "weights" / "merged" / "best.pt"
    
    # Configurar GPU
    device = detectar_y_configurar_gpu()
    
    # Cargar modelo
    model = cargar_modelo(str(model_path), device)
    if model is None:
        return
    
    # Iniciar detección
    procesar_camara(model, device)


if __name__ == "__main__":
    main()

