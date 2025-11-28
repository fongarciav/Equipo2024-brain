"""
Filter Controller Module - Filters lane detection data to reject outliers.

Following SRP: This module only handles filtering logic for lane detection data.
"""


class FilterController:
    """
    Filter Controller responsible for rejecting sudden large changes in steering angle.
    """
    def __init__(self, max_change: float = 10.0):
        """
        Initialize the filter controller.
        
        Args:
            max_change: Maximum allowed change in degrees between consecutive frames (default: 10.0)
        """
        self.max_change = max_change
        self.prev_value = None
        
    def filter(self, current_value: float) -> float:
        """
        Filter the current value based on previous value.
        
        Args:
            current_value: The current angle deviation in degrees.
            
        Returns:
            The current value if valid, or None if it should be skipped.
        """
        if current_value is None:
            return None
            
        # If first value, accept it
        if self.prev_value is None:
            self.prev_value = current_value
            return current_value
            
        # Check difference
        diff = abs(current_value - self.prev_value)
        
        if diff > self.max_change:
            # Reject outlier
            # We keep prev_value as is
            return None
        else:
            # Accept value
            self.prev_value = current_value
            return current_value
            
    def reset(self):
        """Reset the filter state."""
        self.prev_value = None

