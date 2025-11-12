#!/usr/bin/env python3
"""
Signal Detector Module using YOLO/TensorRT

Handles traffic signal detection using a trained YOLO model or TensorRT engine.
Prioritizes TensorRT for optimal performance on NVIDIA Jetson.
Follows Single Responsibility Principle - only handles signal detection.
"""

import cv2
import numpy as np
import os
from typing import Optional, List, Dict, Tuple
import threading


class SignalDetector:
    """
    Detector for traffic signals using YOLO model.
    
    Handles:
    - Loading YOLO model
    - Processing frames for signal detection
    - Returning detected signals with confidence
    """
    
    def __init__(self, model_path: str, conf_threshold: float = 0.5):
        """
        Initialize signal detector.
        
        Args:
            model_path: Path to model file (.engine, .trt for TensorRT, or .pt for YOLO)
            conf_threshold: Confidence threshold for detections (default: 0.5)
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None
        self.model_type = None
        self.device = 'cpu'
        self.input_shape = None  # For TensorRT: (height, width)
        self.class_names = None  # Class names mapping
        self._lock = threading.Lock()
        self._load_model()
    
    def _is_tensorrt_model(self, model_path: str) -> bool:
        """Check if the model file is a TensorRT engine."""
        ext = os.path.splitext(model_path)[1].lower()
        return ext in ['.engine', '.trt', '.plan']
    
    def _load_tensorrt_model(self, model_path: str):
        """Load TensorRT engine model."""
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit
            
            # Load TensorRT engine
            TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
            
            with open(model_path, 'rb') as f:
                engine_data = f.read()
            
            runtime = trt.Runtime(TRT_LOGGER)
            engine = runtime.deserialize_cuda_engine(engine_data)
            
            if engine is None:
                raise RuntimeError("Failed to load TensorRT engine")
            
            context = engine.create_execution_context()
            
            # Get input/output shapes
            input_binding = engine.get_binding_shape(0)
            # Handle different input formats: [batch, channels, height, width] or [batch, height, width, channels]
            if len(input_binding) == 4:
                # Format: [batch, channels, height, width]
                self.input_shape = (input_binding[2], input_binding[3])  # (height, width)
            elif len(input_binding) == 3:
                # Format: [batch, height, width] (grayscale) or [height, width, channels]
                self.input_shape = (input_binding[1], input_binding[2])  # (height, width)
            else:
                # Fallback
                self.input_shape = (640, 640)  # Default YOLO input size
                print(f"⚠ Warning: Could not determine input shape, using default: {self.input_shape}")
            
            # Allocate GPU memory
            input_size = trt.volume(input_binding) * engine.max_batch_size * np.dtype(np.float32).itemsize
            output_size = trt.volume(engine.get_binding_shape(1)) * engine.max_batch_size * np.dtype(np.float32).itemsize
            
            d_input = cuda.mem_alloc(input_size)
            d_output = cuda.mem_alloc(output_size)
            
            bindings = [int(d_input), int(d_output)]
            stream = cuda.Stream()
            
            self.model = {
                'engine': engine,
                'context': context,
                'bindings': bindings,
                'stream': stream,
                'd_input': d_input,
                'd_output': d_output,
                'input_shape': self.input_shape
            }
            
            self.model_type = 'tensorrt'
            self.device = 'cuda'
            
            print(f"✓ Signal detector loaded (TensorRT): {model_path}")
            print(f"  Device: NVIDIA GPU (TensorRT)")
            print(f"  Input shape: {self.input_shape}")
            print(f"  GPU acceleration: ENABLED (TensorRT)")
            
            return True
            
        except ImportError as e:
            print(f"⚠ TensorRT dependencies not available: {e}")
            print("   Install: pip install nvidia-tensorrt pycuda")
            return False
        except Exception as e:
            print(f"⚠ Error loading TensorRT model: {e}")
            return False
    
    def _load_yolo_model(self, model_path: str):
        """Load YOLO model (Ultralytics or YOLOv5) as fallback."""
        try:
            # Try ultralytics (YOLOv8)
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
                self.model_type = 'ultralytics'
                self.device = 'cuda'  # Ultralytics auto-detects GPU
                print(f"✓ Signal detector loaded (Ultralytics YOLO): {model_path}")
                print(f"  GPU acceleration: Auto-detected")
                return True
            except ImportError:
                # Try torch with YOLOv5
                try:
                    import torch
                    self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
                    if torch.cuda.is_available():
                        self.model = self.model.to('cuda')
                        self.device = 'cuda'
                    else:
                        self.device = 'cpu'
                    self.model_type = 'yolov5'
                    print(f"✓ Signal detector loaded (YOLOv5): {model_path}")
                    print(f"  Device: {self.device}")
                    return True
                except Exception as e:
                    raise ImportError(f"Could not load YOLO model. Error: {e}")
        except Exception as e:
            print(f"⚠ Error loading YOLO model: {e}")
            return False
    
    def _load_model(self):
        """Load model - prioritizes TensorRT, falls back to YOLO."""
        if not os.path.exists(self.model_path):
            print(f"⚠ Model file not found: {self.model_path}")
            self.model = None
            return
        
        # Try TensorRT first (for Jetson optimization)
        if self._is_tensorrt_model(self.model_path):
            if self._load_tensorrt_model(self.model_path):
                return
        
        # Fallback to YOLO
        if self._load_yolo_model(self.model_path):
            return
        
        # If all failed
        print(f"⚠ Warning: Could not load signal detector")
        print("   Signal detection will be disabled.")
        self.model = None
        self.device = 'cpu'
    
    def is_available(self) -> bool:
        """Check if model is loaded and available."""
        return self.model is not None
    
    def _preprocess_tensorrt(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess frame for TensorRT inference."""
        # Resize to model input size
        height, width = self.input_shape
        resized = cv2.resize(frame, (width, height))
        
        # Convert BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1] and transpose to CHW format
        normalized = rgb.astype(np.float32) / 255.0
        transposed = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        batched = np.expand_dims(transposed, axis=0)
        
        return batched
    
    def _postprocess_tensorrt(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict]:
        """
        Postprocess TensorRT output to detections.
        
        Handles common YOLO TensorRT output formats:
        - Format 1: [batch, num_detections, 7] where 7 = [batch_id, class_id, confidence, x1, y1, x2, y2] (post-NMS)
        - Format 2: [batch, num_detections, 6] where 6 = [x1, y1, x2, y2, confidence, class_id]
        - Format 3: [batch, num_boxes, 85] where 85 = 4(bbox) + 1(objectness) + 80(classes) - requires NMS
        """
        detections = []
        h_orig, w_orig = original_shape[:2]
        h_model, w_model = self.input_shape
        
        # Scale factors
        scale_x = w_orig / w_model
        scale_y = h_orig / h_model
        
        try:
            # Remove batch dimension if present
            if len(output.shape) == 3:
                output = output[0]  # [num_detections, features]
            
            if len(output) == 0:
                return detections
            
            # Determine format based on shape
            num_features = output.shape[-1] if len(output.shape) > 1 else len(output)
            
            if num_features == 7:
                # Format: [batch_id, class_id, confidence, x1, y1, x2, y2] (post-NMS)
                for det in output:
                    batch_id, cls, conf, x1, y1, x2, y2 = det[:7]
                    if conf >= self.conf_threshold:
                        # Scale to original image size
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)
                        
                        detections.append({
                            'class': str(int(cls)),
                            'class_id': int(cls),
                            'confidence': float(conf),
                            'bbox': [x1, y1, x2, y2],
                            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                        })
            
            elif num_features == 6:
                # Format: [x1, y1, x2, y2, confidence, class_id]
                for det in output:
                    x1, y1, x2, y2, conf, cls = det[:6]
                    if conf >= self.conf_threshold:
                        # Scale to original image size
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)
                        
                        detections.append({
                            'class': str(int(cls)),
                            'class_id': int(cls),
                            'confidence': float(conf),
                            'bbox': [x1, y1, x2, y2],
                            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                        })
            
            elif num_features >= 85:
                # Format: [batch, num_boxes, 85] - YOLO raw output (needs NMS)
                # This format requires NMS, but for now we'll extract the best class
                print("⚠ Warning: Raw YOLO format detected. NMS may be needed for optimal results.")
                for det in output:
                    # Extract bbox (first 4 values)
                    x1, y1, x2, y2 = det[0:4]
                    # Extract objectness (index 4)
                    objectness = det[4]
                    # Extract class scores (indices 5:85)
                    class_scores = det[5:85] if len(det) >= 85 else det[5:]
                    
                    # Get best class
                    best_class = np.argmax(class_scores)
                    confidence = float(objectness * class_scores[best_class])
                    
                    if confidence >= self.conf_threshold:
                        # Scale to original image size
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)
                        
                        detections.append({
                            'class': str(int(best_class)),
                            'class_id': int(best_class),
                            'confidence': confidence,
                            'bbox': [x1, y1, x2, y2],
                            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                        })
            
            else:
                print(f"⚠ Warning: Unknown TensorRT output format with {num_features} features")
                print(f"   Output shape: {output.shape}")
                # Try to parse as [x1, y1, x2, y2, conf, ...] format
                if num_features >= 5:
                    for det in output:
                        x1, y1, x2, y2 = det[0:4]
                        conf = det[4] if num_features > 4 else 1.0
                        cls = int(det[5]) if num_features > 5 else 0
                        
                        if conf >= self.conf_threshold:
                            x1 = int(x1 * scale_x)
                            y1 = int(y1 * scale_y)
                            x2 = int(x2 * scale_x)
                            y2 = int(y2 * scale_y)
                            
                            detections.append({
                                'class': str(cls),
                                'class_id': cls,
                                'confidence': float(conf),
                                'bbox': [x1, y1, x2, y2],
                                'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                            })
        
        except Exception as e:
            print(f"Error parsing TensorRT output: {e}")
            import traceback
            traceback.print_exc()
        
        return detections
    
    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Detect signals in a frame.
        
        Args:
            frame: Input frame (BGR format)
        
        Returns:
            List of detections, each with:
            - 'class': class name or id
            - 'confidence': confidence score
            - 'bbox': [x1, y1, x2, y2] bounding box coordinates
            - 'center': (x, y) center point
        """
        if not self.is_available():
            return []
        
        detections = []
        
        try:
            with self._lock:
                if self.model_type == 'tensorrt':
                    # TensorRT inference
                    import pycuda.driver as cuda
                    
                    # Preprocess
                    preprocessed = self._preprocess_tensorrt(frame)
                    
                    # Copy input to GPU
                    cuda.memcpy_htod_async(self.model['d_input'], preprocessed, self.model['stream'])
                    
                    # Run inference
                    self.model['context'].execute_async_v2(
                        bindings=self.model['bindings'],
                        stream_handle=self.model['stream'].handle
                    )
                    
                    # Copy output from GPU
                    output = np.empty(self.model['engine'].get_binding_shape(1), dtype=np.float32)
                    cuda.memcpy_dtoh_async(output, self.model['d_output'], self.model['stream'])
                    self.model['stream'].synchronize()
                    
                    # Postprocess
                    detections = self._postprocess_tensorrt(output, frame.shape)
                
                elif self.model_type == 'ultralytics':
                    # YOLOv8 (ultralytics)
                    results = self.model(frame, conf=self.conf_threshold, verbose=False)
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            cls = int(box.cls[0].cpu().numpy())
                            class_name = result.names[cls] if hasattr(result, 'names') else str(cls)
                            
                            detections.append({
                                'class': class_name,
                                'class_id': cls,
                                'confidence': conf,
                                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                            })
                
                elif self.model_type == 'yolov5':
                    # YOLOv5
                    results = self.model(frame)
                    for *box, conf, cls in results.xyxy[0].cpu().numpy():
                        if conf >= self.conf_threshold:
                            x1, y1, x2, y2 = map(int, box)
                            detections.append({
                                'class': self.model.names[int(cls)] if hasattr(self.model, 'names') else str(int(cls)),
                                'class_id': int(cls),
                                'confidence': float(conf),
                                'bbox': [x1, y1, x2, y2],
                                'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                            })
        
        except Exception as e:
            print(f"Error in signal detection: {e}")
            import traceback
            traceback.print_exc()
            return []
        
        return detections
    
    def draw_detections(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        Draw detections on frame.
        
        Args:
            frame: Input frame (BGR format)
            detections: List of detections from detect()
        
        Returns:
            Frame with detections drawn
        """
        output_frame = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            confidence = det['confidence']
            
            # Draw bounding box
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(output_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), (0, 255, 0), -1)
            cv2.putText(output_frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return output_frame
    
    def get_most_confident(self, detections: List[Dict]) -> Optional[Dict]:
        """
        Get the most confident detection.
        
        Args:
            detections: List of detections
        
        Returns:
            Most confident detection or None
        """
        if not detections:
            return None
        
        return max(detections, key=lambda x: x['confidence'])

