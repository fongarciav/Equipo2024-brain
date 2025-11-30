# Architectural Refactor Proposal

## 1. Current Architecture Overview (As of Nov 2025)

The system currently operates with two primary parallel controllers, coordinated via direct reference passing.

### Components
- **Dashboard Server**: The application entry point. It initializes hardware connections (Serial, Camera) and instantiates the controllers.
- **AutoPilotController**:
    - Runs in a background thread.
    - Fetches frames from `VideoStreamer`.
    - Uses `LaneDetector` to find lane lines.
    - Uses `PIDController` to calculate steering angles.
    - Sends commands directly to the ESP32 via `CommandSender`.
- **SignController**:
    - Runs in a separate background thread.
    - Fetches frames from `VideoStreamer` (or a dedicated source).
    - Uses `SignDetector` (YOLO) to find traffic signs.
    - Uses **Strategies** (e.g., `StopStrategy`, `IntersectionStrategy`) to decide on actions.
    - Holds a direct reference to `AutoPilotController`.

### Current Communication & Control Flow
1.  **Normal Driving**: `AutoPilotController` drives the car based on lane lines. `SignController` monitors for signs passively.
2.  **Sign Detection Event**:
    - When `SignController` detects a relevant sign (e.g., Intersection), the active **Strategy** is executed.
    - The Strategy calls `autopilot_controller.pause()`.
    - `AutoPilotController` stops sending commands but keeps the video stream alive for visualization.
    - The Strategy takes exclusive control of the `CommandSender` to execute a manual maneuver (e.g., "Forward 1.5s, Turn Right 3.5s").
    - Once finished, the Strategy calls `autopilot_controller.resume()`, and the Autopilot re-engages.

---

## 2. Problem Statement
Currently, the system has a circular-like dependency where the `SignController` holds a reference to the `AutoPilotController`.
- **Flow**: `SignController` detects a sign -> Pauses `AutoPilotController` -> Executes maneuver -> Resumes `AutoPilotController`.
- **Issue**: The "Vision/Detection" subsystem is directly controlling the "Motion/Control" subsystem. This couples them tightly and makes `SignController` responsible for vehicle state management (Pause/Resume logic).

## 3. Proposed Architecture: Centralized Orchestration
The goal is to invert the dependency. The `AutoPilotController` (or a new `VehicleController`) should be the central brain that orchestrates all subsystems.

### Structure
1.  **LaneFollowingSystem** (Child):
    *   Responsible ONLY for calculating steering angles based on lane lines.
    *   Does not send commands directly to hardware.
    *   Returns: `suggested_steering`, `confidence`.

2.  **SignDetectionSystem** (Child):
    *   Responsible ONLY for detecting signs and proposing actions (strategies).
    *   Returns: `detected_sign`, `proposed_strategy` (or `maneuver_request`).

3.  **VehicleController** (Parent/Orchestrator):
    *   Holds instances of `LaneFollowingSystem` and `SignDetectionSystem`.
    *   **Control Loop**:
        1.  Get input from SignDetector.
        2.  If **Critical Sign** (Stop, Intersection) is active:
            *   Suppress Lane Following.
            *   Execute Sign Strategy (Move Forward, Turn, Stop).
        3.  Else (Normal Driving):
            *   Get input from LaneFollower.
            *   Execute Lane Following commands.

### Benefits
- **Hierarchy**: Clear parent-child relationship. The `SignController` becomes a sensor/advisor, not a commander.
- **Safety**: The Orchestrator has a complete view of the vehicle state and can mediate conflicts (e.g., Obstacle detected + Green Light).
- **Modularity**: Easier to swap out the Lane Follower or Sign Detector without breaking the control logic.
