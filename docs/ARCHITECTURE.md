# Documentación de Arquitectura del Sistema "Brain"

Este documento describe la arquitectura de software del módulo `brain` para el Robot Autónomo (BFMC). El sistema está diseñado para procesar entradas visuales, tomar decisiones de navegación y controlar los actuadores del vehículo.

## 1. Visión General

El sistema está dividido en módulos desacoplados funcionalmente pero integrados en tiempo de ejecución. El servidor del Dashboard actúa como el punto de entrada y orquestador de la inicialización.

### Principales Subsistemas
1.  **Lane Detection (Seguimiento de Carril):** Mantiene el vehículo centrado en la pista.
2.  **Sign Vision (Visión de Señales):** Detecta señales de tráfico y ejecuta maniobras complejas.
3.  **Camera & Sensors:** Abstracción de hardware para cámaras Web y RealSense.
4.  **Dashboard:** Interfaz de usuario web para control, telemetría y visualización.
5.  **Communication:** Envío de comandos al microcontrolador (ESP32) vía Serial.

---

## 2. Estructura del Proyecto

```text
brain/
├── brain/
│   ├── dashboard/           # Servidor Flask y UI
│   │   ├── dashboard_server.py  # ENTRY POINT: Inicializa todo el sistema
│   │   └── templates/       # Interfaz HTML
│   │
│   ├── lane_detection/      # Subsistema de Navegación
│   │   ├── autopilot_controller.py # Orquestador de navegación (Thread)
│   │   ├── lane_detector.py        # Visión por computador (OpenCV)
│   │   ├── pid_controller.py       # Lógica de control (PID)
│   │   ├── filter_controller.py    # Suavizado de señales
│   │   └── angle_converter.py      # Conversión Grados -> Servo
│   │
│   ├── sign_vision/         # Subsistema de Señalización
│   │   ├── sign_controller.py      # Orquestador de señales (Thread)
│   │   ├── sign_detector.py        # Detección con YOLO + RealSense Depth
│   │   └── strategies/             # PATRÓN STRATEGY
│   │       ├── base_strategy.py    # Clase base abstracta
│   │       ├── stop_strategy.py    # Lógica de Stop
│   │       └── intersection_strategy.py # Lógica de Intersección (Manual)
│   │
│   └── camera/              # Drivers de Cámara
│       ├── video_streamer.py       # Cámara Web estándar
│       └── realsense_streamer.py   # Intel RealSense (RGB + Profundidad)
│
├── command_sender.py        # Interfaz de comunicación Serial
└── main.py                  # Script de arranque (opcional, usualmente se usa dashboard)
```

---

## 3. Descripción de Módulos

### 3.1. Dashboard & Inicialización (`dashboard/`)
- **Responsabilidad:** Inicializar todos los componentes (singletons), gestionar hilos y servir la interfaz web.
- **Flujo:** Al iniciar `dashboard_server.py`, se verifica el hardware (cámaras, serial) y se instancian los controladores (`AutoPilotController`, `SignController`) inyectando las dependencias necesarias.

### 3.2. Lane Detection (`lane_detection/`)
- **AutoPilotController:** Ejecuta un bucle de control en un hilo independiente (~30 FPS).
    - Obtiene el frame.
    - Procesa la imagen para hallar líneas (`LaneDetector`).
    - Calcula el error y aplica PID (`PIDController`).
    - Filtra cambios bruscos (`FilterController`).
    - Envía comandos de dirección si es necesario.
- **Capacidad de Pausa:** Puede ser "pausado" por el `SignController`. En este estado, sigue procesando imágenes para visualización (debug), pero **no calcula PID ni envía comandos**.

### 3.3. Sign Vision (`sign_vision/`)
- **SignDetector:** Utiliza un modelo YOLO para detectar objetos. Si hay una cámara RealSense, consulta el mapa de profundidad para obtener la distancia exacta en metros.
- **SignController:** Ejecuta un hilo de detección. Cuando se confirma una señal con suficiente confianza:
    1. Selecciona la **Estrategia** adecuada basada en la etiqueta (label).
    2. Ejecuta la estrategia.
- **Strategies (Patrón de Diseño):**
    - Cada tipo de señal tiene su propia clase lógica.
    - **IntersectionStrategy:** Ejemplo de maniobra compleja. Pausa el Autopilot, ejecuta una secuencia de movimiento "ciega" (por tiempo/odometría) y luego reanuda el Autopilot.

### 3.4. Hardware Abstraction (`camera/` & `command_sender.py`)
- **Streamers:** Proveen una interfaz unificada (`get_frame()`). `RealSenseStreamer` añade `get_distance(x,y)`.
- **CommandSender:** Gestiona la comunicación serial con el ESP32, evitando saturación de buffer.

---

## 4. Flujo de Datos y Control

### Pipeline de Navegación Normal
1.  **Cámara** -> Frame RGB
2.  **LaneDetector** -> `angle_deviation` (Desviación del centro)
3.  **FilterController** -> `filtered_angle` (Eliminación de ruido)
4.  **PIDController** -> `steering_angle` (Ángulo de dirección ideal)
5.  **AngleConverter** -> `servo_pwm` (Valor para el servo 50-160)
6.  **CommandSender** -> UART -> **ESP32**

### Pipeline de Intersección (Interrupción)
1.  **SignDetector** detecta señal "Intersection" o "Stop" (distancia < 1.5m).
2.  **SignController** activa `EnterIntersectionStrategy`.
3.  **Estrategia**:
    *   Llama a `autopilot.pause()` (El seguimiento de carril deja de enviar comandos).
    *   Envía comandos manuales directamente (ej. Velocidad 15, Dirección Recta 1.5s, Giro Derecha 3.5s).
    *   Llama a `autopilot.resume()` (Se reinician los controladores PID).
4.  El sistema vuelve a Navegación Normal.

---

## 5. Configuración Global

Las estrategias de señales utilizan umbrales configurables para su ejecución:
- **`min_confidence`**: Confianza mínima del modelo YOLO (ej. 0.6).
- **`activation_distance`**: Distancia máxima en metros para activar la maniobra (ej. 1.5m). Si la señal está más lejos, se ignora.

---

## 6. Roadmap y Refactorización

Actualmente existe un acoplamiento donde `SignController` conoce y manipula a `AutoPilotController`.
Existe una propuesta de refactorización (`REFACTOR_PROPOSAL.md`) para mover hacia una arquitectura de **Orquestador Central (VehicleController)**, donde un cerebro central gestiona tanto la detección de carril como las señales, eliminando la dependencia circular.

---

## 7. Comunicación UART y Manejo de Errores

La comunicación serial con el ESP32 (115200 baudios) puede sufrir de fragmentación o concatenación de mensajes cuando el microcontrolador envía ráfagas de datos (ej. durante el arranque).

### Problema: Concatenación de Mensajes
Es posible recibir múltiples eventos en una sola línea leída debido al buffering, por ejemplo:
`EVENT:MODE_CHANGED:MANEVENT:STATE_CHANGED:RUNNING`

### Solución: Sanitización de Líneas
En `read_available_lines` (dentro de `dashboard_server.py`), se implementa una lógica de sanitización que:
1.  Detecta si una línea contiene múltiples marcadores `EVENT:`.
2.  Si es así, divide la línea en múltiples eventos independientes.
3.  Reinyecta estos eventos como líneas separadas en el sistema.

Esto previene que el parser de eventos (`parse_system_events`) intente procesar una cadena corrupta y falle al actualizar el estado del sistema.
