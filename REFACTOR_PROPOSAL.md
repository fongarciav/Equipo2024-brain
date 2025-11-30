# Architectural Refactor Proposal

## Current State
Currently, the system has a circular-like dependency where the `SignController` holds a reference to the `AutoPilotController`.
- **Flow**: `SignController` detects a sign -> Pauses `AutoPilotController` -> Executes maneuver -> Resumes `AutoPilotController`.
- **Issue**: The "Vision/Detection" subsystem is directly controlling the "Motion/Control" subsystem. This couples them tightly and makes `SignController` responsible for vehicle state management.

## Proposed Architecture: Centralized Orchestration
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

