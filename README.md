# Nodo de Control Jetson Brain para Comunicación ESP32

Módulo de control UART de alta velocidad para comunicación Jetson-ESP32 en sistemas de vehículos autónomos.

## Resumen

Este módulo implementa un protocolo binario para comunicación de control en tiempo real entre un Jetson (cerebro) y ESP32 (ejecutor). El Jetson maneja la percepción y la toma de decisiones, mientras que el ESP32 ejecuta el control de motores, dirección y otras operaciones de hardware usando FreeRTOS.

## Especificación del Protocolo

### Formato de Mensaje

Cada mensaje tiene un total de **16 bytes**:

```
Byte 0:     Byte de inicio (0xAA)
Byte 1:     Topic (1-4)
Byte 2:     ID de comando
Bytes 3-6:  Valor (int32_t, little-endian)
Bytes 7-8:  Secuencia (uint16_t, little-endian)
Bytes 9-10: TTL (uint16_t, milisegundos, little-endian)
Bytes 11-12: (reservado/no usado)
Bytes 13-14: CRC16 (little-endian)
```

### Topics

- `T_DRIVE = 1`: Comandos de motor/conducción
- `T_STEER = 2`: Comandos de dirección
- `T_LIGHTS = 3`: Control de luces
- `T_SYS = 4`: Gestión del sistema

### Comandos

#### Comandos de Conducción (Topic 1)
- `DRIVE_SET_SPEED = 1`: Establecer punto de consigna de velocidad
- `DRIVE_EBRAKE = 2`: Freno de emergencia
- `DRIVE_STOP = 3`: Comando de parada

#### Comandos de Dirección (Topic 2)
- `STEER_SET_ANGLE = 10`: Establecer ángulo de dirección

#### Comandos de Luces (Topic 3)
- `LIGHTS_ON = 30`: Encender luces
- `LIGHTS_OFF = 31`: Apagar luces

#### Comandos del Sistema (Topic 4)
- `SYS_HEARTBEAT = 20`: Mensaje de latido
- `SYS_MODE = 21`: Establecer modo (0=AUTO, 1=MANUAL)
- `SYS_ARM = 22`: Armar el sistema
- `SYS_DISARM = 23`: Desarmar el sistema

## Instalación

```bash
pip install -r requirements.txt
```

## Uso

### Uso Básico

```python
from jetson_control import JetsonControlNode, SystemMode

# Crear nodo de control
node = JetsonControlNode(
    port='/dev/ttyUSB0',
    baudrate=921600,
    control_hz=100.0,      # Actualizaciones de control a 100 Hz
    heartbeat_hz=10.0      # Latido a 10 Hz
)

# Iniciar comunicación
node.start()

# Establecer modo del sistema y armar
node.set_mode(SystemMode.AUTO)
node.arm()

# Actualizar puntos de consigna (se envían automáticamente a 100 Hz)
node.set_control(speed=0.5, steer_angle=10.0)

# Freno de emergencia (se envía inmediatamente)
node.emergency_brake()

# Cierre limpio
node.stop_all()
```

### Pruebas con CLI Interactivo

Usa la CLI interactiva para pruebas manuales:

```bash
python test_cli.py --port /dev/ttyUSB0 --baudrate 921600
```

**Controles:**
- `W/S`: Aumentar/Disminuir velocidad
- `A/D`: Aumentar/Disminuir ángulo de dirección
- `M`: Alternar modo AUTO/MANUAL
- `R`: Armar sistema
- `U`: Desarmar sistema
- `L`: Alternar luces
- `E`: Freno de emergencia
- `Q`: Salir
- `H/?`: Mostrar ayuda

### Integración con IA/Planificador

```python
import cv2
from jetson_control import JetsonControlNode, SystemMode

node = JetsonControlNode()
node.start()
node.set_mode(SystemMode.AUTO)
node.arm()

# Bucle principal de percepción/planificación
while True:
    # Tu código de percepción (ej. YOLO, OpenCV)
    frame = camera.read()
    detections = yolo_model(frame)
    
    # Tu código de planificación
    speed, angle = planner.compute_control(detections)
    
    # Enviar comandos de control
    node.set_control(speed=speed, steer_angle=angle)
    
    # Verificar condiciones de emergencia
    if emergency_detected:
        node.emergency_brake()
```

## Arquitectura

### Modelo de Threading

El nodo de control usa múltiples hilos de trabajo:

1. **Send Thread**: Procesa la cola de mensajes y envía por UART
2. **Receive Thread**: Recibe y valida mensajes entrantes
3. **Control Thread**: Envía actualizaciones de control a 100 Hz
4. **Heartbeat Thread**: Envía latido a 10 Hz

### Cola de Mensajes

Todos los mensajes se encolan y se envían de forma asíncrona para evitar bloqueos. Los mensajes de emergencia se pueden enviar inmediatamente mediante manejo de prioridad.

### Estadísticas

El nodo rastrea:
- Conteo de mensajes enviados/recibidos
- Errores CRC
- Latencia de ida y vuelta (si se implementan mensajes ACK)
- Tamaño de la cola

Acceso mediante `node.get_stats()`.

## Temporización de Mensajes

- **Actualizaciones de Control**: 100 Hz (cada 10 ms)
- **Latido**: 10 Hz (cada 100 ms)
- **Emergencia**: Inmediato (prioridad)
- **Valores TTL**:
  - Control: 100 ms
  - Latido: 200 ms
  - Emergencia: 80 ms

## Cálculo CRC16

El protocolo usa CRC16-XMODEM (polinomio 0x1021) calculado sobre el payload de 13 bytes (desde topic hasta TTL).

## Manejo de Errores

- Validación automática de CRC en mensajes recibidos
- Recuperación de errores del puerto serie
- Protección contra desbordamiento de cola
- Gestión de estado thread-safe

## Estructura de Archivos

```
brain/
├── protocol.py          # Definiciones de protocolo, packing/unpacking, CRC16
├── jetson_control.py    # Implementación principal del nodo de control
├── test_cli.py          # CLI interactivo para pruebas
├── requirements.txt     # Dependencias de Python
└── README.md           # Este archivo
```

## Configuración

### Configuración del Puerto Serie

- **Baud Rate**: 921600 (por defecto, configurable)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Timeout**: 100 ms (lectura), 1 s (escritura)

### Parámetros Ajustables

- `control_hz`: Frecuencia de actualización de control (por defecto: 100 Hz)
- `heartbeat_hz`: Frecuencia de latido (por defecto: 10 Hz)
- `control_ttl`: TTL para mensajes de control (por defecto: 100 ms)
- `heartbeat_ttl`: TTL para latido (por defecto: 200 ms)
- `emergency_ttl`: TTL para emergencia (por defecto: 80 ms)

## Solución de Problemas

### Puerto Serie No Encontrado

Verificar puertos disponibles:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Asegurar que el usuario tenga permisos:
```bash
sudo usermod -a -G dialout $USER
# Luego cerrar sesión/iniciar sesión
```

### Alta Latencia

- Reducir `control_hz` si el ESP32 no puede procesar a 100 Hz
- Verificar que la velocidad de baudios UART coincida en ambos lados
- Monitorear el tamaño de la cola mediante `get_stats()`

### Errores CRC

- Verificar que la velocidad de baudios coincida con la configuración del ESP32
- Revisar cableado/conexiones
- Asegurar que el ESP32 esté enviando mensajes con formato correcto

## Mejoras Futuras

- [ ] Manejo de mensajes ACK y medición de latencia de ida y vuelta
- [ ] Panel de visualización de telemetría
- [ ] Cola de prioridad de mensajes para comandos de emergencia
- [ ] Reenvío automático del último control si no se recibe actualización
- [ ] Soporte para archivo de configuración
- [ ] Wrapper de integración ROS2

## Licencia

Parte del proyecto Robot Autónomo de Laboratorio BFMC.
