# Protocolo de Comunicación ESP32 - Guía para el Equipo Brain

Este documento describe cómo enviar comandos desde el sistema Brain (Jetson/PC) hacia el ESP32 que controla los motores, dirección y luces del vehículo.

## Formato de Comandos

Todos los comandos deben seguir el formato:

```
CHANNEL:COMMAND:VALUE\n
```

- **CHANNEL**: Un solo carácter que identifica el canal
- **COMMAND**: Nombre del comando (mayúsculas)
- **VALUE**: Valor numérico entero
- **Terminación**: Cada comando debe terminar con `\n` (newline)

## Canales Disponibles

### `C` - CONTROL
Comandos de control del vehículo (velocidad, dirección)

### `E` - EMERGENCY  
Comandos de emergencia (frenado inmediato)

### `M` - MANAGEMENT
Comandos de gestión del sistema (armado, modo, etc.)

## Comandos por Canal

### Canal CONTROL (`C`)

#### `C:SET_SPEED:<valor>`
Establece la velocidad del motor de tracción.

- **Valor**: 0-255 (0 = detenido, 255 = máxima velocidad)
- **TTL**: 200ms (el comando expira si no se renueva)
- **Ejemplo**: `C:SET_SPEED:120`
- **⚠️ Comportamiento por defecto**: Si no se envía ningún comando o el comando expira, el vehículo avanza automáticamente a velocidad 100 (hacia adelante). Esto permite que el vehículo siga moviéndose sin GPS cuando solo se controla la dirección.

```python
# Ejemplo Python
ser.write(b"C:SET_SPEED:120\n")
```

#### `C:SET_STEER:<valor>`
Establece el ángulo de dirección del servo.

- **Valor**: 50-135 (50 = izquierda máxima, 105 = centro, 135 = derecha máxima)
- **TTL**: 200ms
- **Ejemplo**: `C:SET_STEER:105` (centro)

**⚠️ IMPORTANTE - Conversión de Grados a Valores de Servo:**

Si tu lane detector envía grados (ej: -45° a +45°), debes convertir así:

```python
# Servo range: 50 (izquierda) a 135 (derecha), centro = 105
# Range total: 85 unidades
SERVO_LEFT = 50
SERVO_CENTER = 105
SERVO_RIGHT = 135
SERVO_RANGE = 85  # 135 - 50

def degrees_to_servo(degrees, max_degrees=45):
    """
    Convierte grados de lane detector a valor de servo.
    
    Args:
        degrees: Ángulo en grados (-max_degrees a +max_degrees)
        max_degrees: Máximo ángulo permitido (default: 45°)
    
    Returns:
        Valor de servo (50-135)
    """
    # Normalizar a -1.0 a +1.0
    normalized = degrees / max_degrees
    # Limitar al rango [-1, 1]
    normalized = max(-1.0, min(1.0, normalized))
    # Convertir a valor de servo
    servo_value = SERVO_CENTER + (normalized * (SERVO_RANGE / 2))
    return int(round(servo_value))

# Ejemplos:
# degrees_to_servo(0)    -> 105 (centro)
# degrees_to_servo(-45) -> 50  (izquierda máxima)
# degrees_to_servo(45)  -> 135 (derecha máxima)
# degrees_to_servo(-20) -> ~82 (izquierda suave)
```

**Ejemplo completo con lane detector:**

```python
# En lugar de solo imprimir "turn left" o "turn right":
lane_angle = -25  # grados desde tu detector

# Convertir y enviar:
servo_value = degrees_to_servo(lane_angle)
command = f"C:SET_STEER:{servo_value}\n"
ser.write(command.encode())
```

### Canal EMERGENCY (`E`)

#### `E:BRAKE_NOW:0`
Freno de emergencia inmediato. Detiene el motor instantáneamente (<1ms de respuesta).

- **Valor**: Siempre 0 (ignorado)
- **Ejemplo**: `E:BRAKE_NOW:0`

```python
ser.write(b"E:BRAKE_NOW:0\n")
```

#### `E:STOP:0`
Alias para freno de emergencia (mismo comportamiento que BRAKE_NOW).

### Canal MANAGEMENT (`M`)

#### `M:SYS_ARM:0`
Arma el sistema (prepara para operación).

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_ARM:0`

#### `M:SYS_DISARM:0`
Desarma el sistema (modo seguro).

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_DISARM:0`

#### `M:SYS_MODE:<valor>`
Establece el modo del sistema.

- **Valor**: 0 = MANUAL, 1 = AUTO
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_MODE:1` (modo AUTO)

## Ejemplos de Uso

### Ejemplo 1: Control Básico

```python
import serial

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

# Armar sistema
ser.write(b"M:SYS_ARM:0\n")

# Establecer velocidad
ser.write(b"C:SET_SPEED:150\n")

# Centrar dirección
ser.write(b"C:SET_STEER:105\n")

# Frenar de emergencia
ser.write(b"E:BRAKE_NOW:0\n")
```

### Ejemplo 2: Lane Following

```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

def degrees_to_servo(degrees, max_degrees=45):
    SERVO_CENTER = 105
    SERVO_RANGE = 85
    normalized = max(-1.0, min(1.0, degrees / max_degrees))
    return int(round(SERVO_CENTER + (normalized * (SERVO_RANGE / 2))))

# Loop de control
while True:
    # Obtener ángulo del lane detector
    lane_angle = get_lane_angle()  # Tu función
    
    # Convertir y enviar comando de dirección
    servo_value = degrees_to_servo(lane_angle)
    command = f"C:SET_STEER:{servo_value}\n"
    ser.write(command.encode())
    
    # Mantener velocidad constante
    ser.write(b"C:SET_SPEED:120\n")
    
    time.sleep(0.05)  # 20 Hz update rate
```

### Ejemplo 3: Control con Velocidad Variable

```python
def control_vehicle(speed, steering_degrees):
    """
    Función helper para controlar el vehículo.
    
    Args:
        speed: Velocidad 0-255
        steering_degrees: Ángulo de dirección en grados (-45 a +45)
    """
    # Validar velocidad
    speed = max(0, min(255, int(speed)))
    
    # Convertir dirección
    servo_value = degrees_to_servo(steering_degrees)
    
    # Enviar comandos
    ser.write(f"C:SET_SPEED:{speed}\n".encode())
    ser.write(f"C:SET_STEER:{servo_value}\n".encode())
```

## Consideraciones Importantes

### Time-to-Live (TTL)
Los comandos tienen un tiempo de vida limitado:
- **CONTROL**: 200ms - Debes enviar comandos periódicamente (mínimo 5 Hz)
- **MANAGEMENT**: 5000ms - Comandos de sistema duran más

**Recomendación**: Envía comandos de velocidad y dirección a **10-20 Hz** para mantener el control.

### Last-Writer-Wins
El sistema usa patrón "last-writer-wins". Si envías múltiples comandos rápidamente, solo el último es válido. No hay cola de comandos.

### Respuestas del ESP32
El ESP32 puede enviar mensajes de debug por serial. Puedes leerlos para debugging:

```python
if ser.in_waiting > 0:
    response = ser.readline().decode('utf-8', errors='ignore')
    print(f"ESP32: {response}")
```

### Manejo de Errores
- Si el comando no se parsea correctamente, el ESP32 imprime: `[LinkRxTask] Failed to parse message: ...`
- Verifica que el formato sea exacto: `CHANNEL:COMMAND:VALUE\n`
- Asegúrate de que el baud rate coincida (115200 para USB, 921600 para UART externo)

## Checklist de Integración

- [ ] Configurar conexión serial (USB o UART externo)
- [ ] Implementar función de conversión grados → servo
- [ ] Enviar comandos periódicamente (10-20 Hz)
- [ ] Manejar freno de emergencia en caso de detección de obstáculos
- [ ] Implementar heartbeat/supervisión si es necesario
- [ ] Probar con el simulador UART antes de integrar con lane detector

## Simulador de Pruebas

Puedes probar tus comandos con el simulador incluido:

```bash
cd embedded
pip install -r test/python/requirements.txt
python3 test/python/test_uart_simulator.py /dev/ttyUSB0 --baud 115200
```

Luego prueba comandos como:
- `C:SET_SPEED:120`
- `C:SET_STEER:105`
- `E:BRAKE_NOW:0`

## Soporte

Si tienes dudas sobre el protocolo o encuentras problemas, consulta:
- `embedded/src/link_rx_task.cpp` - Implementación del parser
- `embedded/include/messages.h` - Definiciones de canales y comandos
- `embedded/include/hardware.h` - Valores de servo (SERVO_CENTER, etc.)

