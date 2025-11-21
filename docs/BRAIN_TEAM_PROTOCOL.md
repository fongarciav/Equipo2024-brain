# Protocolo de Comunicación ESP32 - Guía para el Equipo Brain

Este documento describe cómo enviar comandos desde el sistema Brain (Jetson/PC) hacia el ESP32 que controla los motores, dirección y luces del vehículo.

## ⚠️ IMPORTANTE: Sistema de Seguridad (ARM/DISARM)

**El sistema inicia DESARMADO por defecto.** Debes armar el sistema antes de que los comandos de control funcionen.

**Secuencia de inicio:**
1. Armar el sistema: `M:SYS_ARM:0`
2. Establecer modo: `M:SYS_MODE:1` (AUTO) o `M:SYS_MODE:0` (MANUAL)
3. En modo AUTO: el sistema pasa a RUNNING automáticamente cuando recibe heartbeat
4. En modo MANUAL: el sistema necesita estar en estado RUNNING (requiere ARM + modo MANUAL)

**Los comandos de control (SET_SPEED, SET_STEER) solo funcionan cuando el sistema está ARMED y RUNNING.**

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

### Canal EMERGENCY (`E`)

#### `E:BRAKE_NOW:0`
Freno de emergencia inmediato. Detiene el motor instantáneamente (<1ms de respuesta).

- **Valor**: Siempre 0 (ignorado)
- **Ejemplo**: `E:BRAKE_NOW:0`

#### `E:STOP:0`
Alias para freno de emergencia (mismo comportamiento que BRAKE_NOW).

### Canal MANAGEMENT (`M`)

#### `M:SYS_ARM:0`
**⚠️ REQUERIDO** - Arma el sistema (prepara para operación). Debe enviarse antes de cualquier comando de control.

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_ARM:0`
- **Nota**: Sin ARM, los comandos SET_SPEED y SET_STEER serán ignorados

#### `M:SYS_DISARM:0`
Desarma el sistema (modo seguro). Detiene el vehículo inmediatamente.

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_DISARM:0`
- **Efecto**: Detiene motor y centra dirección

#### `M:SYS_MODE:<valor>`
Establece el modo del sistema.

- **Valor**: 0 = MANUAL, 1 = AUTO
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_MODE:1` (modo AUTO)
- **Nota**: En modo AUTO, el sistema pasa a RUNNING automáticamente cuando recibe heartbeat

## Ejemplos de Uso

### Ejemplo 1: Control Básico

### Ejemplo 2: Lane Following


### Ejemplo 3: Control con Velocidad Variable

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

