from abc import ABC, abstractmethod
from threading import Lock

class SignStrategy(ABC):
    """Base class for sign handling strategies."""
    
    def __init__(self, controller, lock: Lock, min_confidence: float = 0.6, activation_distance: float = 1.5):
        """
        Initialize the strategy.
        
        Args:
            controller: The SignController instance
            lock: Threading lock
            min_confidence: Minimum confidence required to execute (default 0.6)
            activation_distance: Maximum distance (meters) to execute. If sign is farther, ignore. (default 1.5)
        """
        self.controller = controller
        self.lock = lock
        self.min_confidence = min_confidence
        self.activation_distance = activation_distance
        
    def validate_detection(self, detection: dict) -> bool:
        """
        Check if detection meets confidence and distance requirements.
        
        Args:
            detection: Detection dictionary
            
        Returns:
            bool: True if valid, False if should be ignored
        """
        # Check confidence
        print(f"[SignStrategy] Validating detection: {detection}")
        confidence = detection.get('confidence', 0.0)
        print(f"[SignStrategy] Confidence: {confidence}")
        if confidence < self.min_confidence:
            print(f"[SignStrategy] Confidence too low: {confidence}")
            return False
            
        # Check distance (only if available)
        print(f"[SignStrategy] Checking distance: {detection}")
        distance = detection.get('distance')
        print(f"[SignStrategy] Distance: {distance}")
        if distance is not None:
            print(f"[SignStrategy] Distance: {distance}")
            if distance > self.activation_distance:
                print(f"[SignStrategy] Distance too far: {distance}")
                return False # Too far away
                
        return True

    @abstractmethod
    def execute(self, detection: dict):
        """
        Execute the strategy for a detected sign.
        
        Args:
            detection: Dictionary containing detection info ('class', 'confidence', etc.)
            
        Returns:
            bool: True if an action was taken, False otherwise
        """
        pass

