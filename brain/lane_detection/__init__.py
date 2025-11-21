"""
Lane Following Module - Handles lane detection, PID control, and angle conversion.

Following SRP: Each module has a single responsibility.
"""

from .lane_detector import LaneDetector, MarcosLaneDetector_Advanced
from .autopilot_controller import AutoPilotController

__all__ = ['LaneDetector', 'MarcosLaneDetector_Advanced', 'AutoPilotController']

