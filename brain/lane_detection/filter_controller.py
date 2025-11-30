"""
Filter Controller Module - Filters lane detection data to reject outliers.

Following SRP: This module only handles filtering logic for lane detection data.
"""

from collections import deque
import statistics

# TODO: chequear esto, creo que no esta funcionando, solo me tira rejeted outlier todos seguido. 
class FilterController:
    """
    Filter Controller responsible for rejecting sudden large changes in steering angle.
    Uses a history buffer to compare against the average of recent valid readings.
    """
    def __init__(self, max_change: float = 10.0, history_size: int = 10):
        """
        Initialize the filter controller.
        
        Args:
            max_change: Maximum allowed change in degrees from the moving average (default: 10.0)
            history_size: Number of previous frames to keep for averaging (default: 10)
        """
        self.max_change = max_change
        self.history_size = history_size
        self.history = deque(maxlen=history_size)
        
    def filter(self, current_value: float) -> float:
        """
        Filter the current value based on history.
        
        Args:
            current_value: The current angle deviation in degrees.
            
        Returns:
            The current value if valid, or None if it should be skipped.
        """
        if current_value is None:
            return None
            
        # If history is empty, accept the first value
        if not self.history:
            self.history.append(current_value)
            return current_value
            
        # Calculate average of history
        avg_val = statistics.mean(self.history)
        
        # Check difference from average
        diff = abs(current_value - avg_val)
        
        if diff > self.max_change:
            # Reject outlier
            # We do NOT add it to history to avoid polluting the average with outliers
            return None
        else:
            # Accept value and add to history
            self.history.append(current_value)
            return current_value
            
    def reset(self):
        """Reset the filter state."""
        self.history.clear()
