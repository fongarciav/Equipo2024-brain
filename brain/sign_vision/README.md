# Sign Vision & Control System

This module handles traffic sign detection and autonomous response using a Strategy Pattern. It integrates with the Autopilot system to perform complex maneuvers like intersection handling.

## Architecture

- **SignDetector**: Uses YOLO to detect signs and (optionally) Intel RealSense for distance measurement.
- **SignController**: Orchestrates the response. It receives detections and executes the appropriate strategy.
- **Strategies**: Located in `strategies/`, these define specific behaviors for each sign type.

## Strategies

All strategies inherit from `SignStrategy` and include **Execution Thresholds**:
- **`min_confidence`**: Minimum detection confidence required (default: 0.6).
- **`activation_distance`**: Maximum distance (meters) to the sign. The strategy will **ignore** signs detected farther away than this distance (default: 1.0m - 1.5m).

### 1. DefaultStopStrategy (`stop_strategy.py`)
- **Behavior**: Stops the car, waits for a cooldown period, and then resumes.
- **Usage**: Used for Stop signs, Red lights, etc.

### 2. EnterIntersectionStrategy (`intersection_strategy.py`)
- **Behavior**: 
    1. **Pauses** the Autopilot (Lane Follower).
    2. Executes a **Manual Sequence** (e.g., Drive Forward 1.5s, Turn Right 3.5s).
    3. **Resumes** the Autopilot.
- **Customization**: You can edit the manual sequence in `execute()` to match different intersection geometries.
- **Servo Constants**:
    - Center: 105
    - Right: 50 (We typically use 60 for turning)
    - Left: 160

## Autopilot Integration (Pause vs Stop)

The `SignController` is linked to the `AutoPilotController`. This allows strategies to:

- **`pause()`**: Temporarily disables lane following calculations and command sending. 
    - **Why Pause instead of Stop?**: `stop()` would kill the control thread entirely. We want to keep the thread alive so the video stream continues to update on the dashboard.
    - **Visualization**: Even when paused, the system **continues to process lane lines** (updating "Bird View" and "Sliding Windows" debug images). It simply **skips the PID control step**, preventing the Autopilot from fighting the manual strategy's commands.
- **`resume()`**: Re-enables lane following and resets PID/Filter controllers to prevent steering jumps when re-engaging.

## Camera & RealSense Integration

The system is designed to support both standard webcams and Intel RealSense RGB-D cameras seamlessly.

- **Streamer Abstraction**: We created a `RealSenseStreamer` class that mimics the interface of the standard `VideoStreamer`.
- **Automatic Detection**: In `dashboard_server.py`, the system first attempts to initialize the RealSense camera. If unavailable or initialization fails, it gracefully falls back to the standard webcam streamer.
- **Distance Measurement**: If the RealSense camera is active, the `SignDetector` queries the depth map for the center pixel of the detected sign. This allows us to display the real-time distance (in meters) to the sign on the detection window.

## Adding a New Strategy

1. Create a new file in `strategies/` (e.g., `yield_strategy.py`).
2. Inherit from `SignStrategy` (`base_strategy.py`).
3. Implement the `execute(self, detection)` method.
4. Register it in `strategies/__init__.py`.
5. Map it to a label in `SignController.__init__`.
