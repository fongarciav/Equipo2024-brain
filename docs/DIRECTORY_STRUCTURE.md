# Estructura de Directorios y Módulos del Proyecto "Brain"

Este documento detalla la organización física del código fuente del proyecto.

```text
brain/                          # Raíz del proyecto
│
├── camera/                     # Módulo de abstracción de hardware de cámara
│   ├── __init__.py
│   ├── camera_config.py        # Utilidades para selección de cámara según SO
│   ├── realsense_streamer.py   # Driver para Intel RealSense (RGB + Depth)
│   ├── test_realsense.py       # Script de prueba aislado para RealSense
│   └── video_streamer.py       # Driver genérico para Webcam y streaming MJPEG
│
├── dashboard/                  # Servidor Web y UI de control
│   ├── static/                 # Archivos estáticos (CSS, JS, Iconos)
│   ├── templates/              # Plantillas HTML (dashboard.html)
│   ├── __init__.py
│   └── dashboard_server.py     # ENTRY POINT: Inicializa controladores y servidor Flask
│
├── lane_detection/             # Subsistema de Navegación Autónoma
│   ├── __init__.py
│   ├── angle_converter.py      # Conversión de ángulo de dirección a valores PWM servo
│   ├── autopilot_controller.py # Controlador principal (Hilo de control PID)
│   ├── filter_controller.py    # Filtro de media móvil para suavizar dirección
│   ├── lane_detector.py        # Procesamiento de imagen OpenCV (líneas, bird-view)
│   └── pid_controller.py       # Implementación del algoritmo PID
│
├── sign_vision/                # Subsistema de Detección de Señales
│   ├── strategies/             # Implementación del Patrón Strategy
│   │   ├── __init__.py         # Registro de estrategias disponibles
│   │   ├── base_strategy.py    # Clase abstracta con lógica de validación (distancia/confianza)
│   │   ├── intersection_strategy.py # Maniobra manual para intersecciones
│   │   └── stop_strategy.py    # Maniobra de parada y espera
│   ├── weights/                # Archivos de modelos YOLO (.pt, .engine)
│   ├── __init__.py
│   ├── README.md               # Documentación específica del subsistema
│   ├── sign_controller.py      # Controlador de lógica de señales (Hilo de decisión)
│   └── sign_detector.py        # Inferencia YOLO y cálculo de distancias
│
├── docs/                       # Documentación técnica
│   ├── ARCHITECTURE.md         # Visión general de arquitectura y flujo de datos
│   ├── CLASS_DIAGRAM.md        # Diagrama de clases UML (Mermaid)
│   ├── REFACTOR_PROPOSAL.md    # Propuesta para desacoplar controladores
│   └── DIRECTORY_STRUCTURE.md  # Este archivo
│
├── command_sender.py           # Interfaz de bajo nivel para comunicación Serial (UART) con ESP32
├── main.py                     # Script de arranque alternativo (CLI/Test)
├── requirements.txt            # Lista de dependencias de Python
└── __init__.py                 # Archivo de paquete raíz
```

## Descripción de Carpetas Principales

*   **`camera/`**: Encapsula la complejidad de capturar video desde diferentes fuentes (Webcam o Cámara de Profundidad) y proveer un stream uniforme al resto del sistema.
*   **`dashboard/`**: Contiene toda la lógica de interacción con el usuario. Es responsable de "levantar" el sistema completo al iniciarse.
*   **`lane_detection/`**: Contiene la lógica matemática y de visión para mantener el coche en la carretera. Es el sistema "reactivo" rápido (30Hz).
*   **`sign_vision/`**: Contiene la lógica de alto nivel ("cognitiva") para interpretar señales de tráfico y ejecutar secuencias de acciones complejas.
