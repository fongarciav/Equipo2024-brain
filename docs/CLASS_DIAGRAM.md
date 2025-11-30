# Diagrama de Clases del Sistema "Brain"

```mermaid
classDiagram
    %% Relaciones Principales
    DashboardServer ..> AutoPilotController : Instantiates
    DashboardServer ..> SignController : Instantiates
    
    AutoPilotController --> VideoStreamer : Uses
    AutoPilotController --> CommandSender : Uses
    AutoPilotController *-- LaneDetector : Composes
    AutoPilotController *-- PIDController : Composes
    AutoPilotController *-- FilterController : Composes
    AutoPilotController *-- AngleConverter : Composes

    SignController --> VideoStreamer : Uses
    SignController --> CommandSender : Uses
    SignController --> AutoPilotController : References (Pause/Resume)
    SignController *-- SignDetector : Composes
    SignController o-- SignStrategy : Aggregates (Strategies)

    SignStrategy <|-- EnterIntersectionStrategy : Inherits
    
    VideoStreamer <|-- RealSenseStreamer : Inherits

    %% Definiciones de Clase
    
    class DashboardServer {
        +initialize_autopilot_if_needed()
        +initialize_sign_detection_if_needed()
    }

    class AutoPilotController {
        -is_running: bool
        -is_paused: bool
        -thread: Thread
        +start()
        +stop()
        +pause()
        +resume()
        -_control_loop()
    }

    class SignController {
        -strategies: Dict
        +start()
        +stop()
        -_control_loop()
    }

    class LaneDetector {
        +get_lane_metrics(frame)
    }

    class PIDController {
        +compute(error, dt)
        +reset()
    }

    class SignDetector {
        -model: YOLO
        +confidence_threshold: float
        +initialize()
        +get_detections()
    }

    class VideoStreamer {
        +get_frame()
        +initialize()
    }

    class RealSenseStreamer {
        +get_distance(x, y)
    }

    class CommandSender {
        +send_speed_command(speed)
        +send_steering_command(angle)
    }

    class SignStrategy {
        <<Abstract>>
        +min_confidence: float
        +activation_distance: float
        +execute(detection)
        +validate_detection(detection)
    }

    class EnterIntersectionStrategy {
        +execute(detection)
    }
```

## Explicación de Relaciones

*   **Composición (`*--`)**: El controlador es "dueño" del componente (ej. `AutoPilotController` crea y posee su `PIDController`).
*   **Agregación (`o--`)**: El controlador tiene una colección de objetos (ej. `SignController` tiene un diccionario de `Strategies`).
*   **Uso/Asociación (`-->`)**: El controlador utiliza un recurso compartido (ej. `VideoStreamer`, `CommandSender`).
*   **Referencia (`-->`)**: `SignController` conoce a `AutoPilotController` para poder pausarlo (esta es la dependencia que queremos refactorizar en el futuro).

