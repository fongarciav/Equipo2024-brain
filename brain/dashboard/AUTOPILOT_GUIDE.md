# Guía de Uso del Autopilot

Esta guía explica cómo ejecutar el sistema de autopilot que controla el vehículo usando detección de carriles.

## 📋 Requisitos Previos

1. **Hardware necesario:**
   - ESP32 conectado vía USB/UART
   - Cámara USB conectada (para detección de carriles)
   - Vehículo con ESP32 programado y funcionando

2. **Dependencias Python:**
   ```bash
   cd brain/dashboard
   pip install -r requirements.txt
   ```
   
   O instalar manualmente:
   ```bash
   pip install Flask flask-cors pyserial opencv-python numpy
   ```

## 🚀 Ejecución del Autopilot

### Opción 1: Inicio Automático (Recomendado)

El autopilot se inicia automáticamente cuando:
- El puerto serial está conectado
- La cámara está disponible
- El servidor se ejecuta

```bash
cd brain/dashboard
python dashboard_server.py --port-name /dev/ttyUSB0
```

**En Windows:**
```bash
python dashboard_server.py --port-name COM3
```

### Opción 2: Selección Interactiva de Puerto

```bash
cd brain/dashboard
python dashboard_server.py --auto-connect
```

Esto te permitirá seleccionar el puerto serial interactivamente.

### Opción 3: Control Manual desde el Dashboard

1. Inicia el servidor sin conectar automáticamente:
   ```bash
   python dashboard_server.py
   ```

2. Abre el dashboard en el navegador: `http://localhost:5000`

3. Conecta el UART desde la interfaz web

4. El autopilot se inicializará automáticamente si:
   - El UART está conectado
   - La cámara está disponible

5. Usa los endpoints para controlar el autopilot:
   - `POST /autopilot/start` - Iniciar autopilot
   - `POST /autopilot/stop` - Detener autopilot
   - `GET /autopilot/status` - Ver estado del autopilot

## ⚙️ Parámetros de Configuración

El autopilot acepta varios parámetros para ajustar el comportamiento:

```bash
python dashboard_server.py \
  --port-name /dev/ttyUSB0 \
  --pid-kp 0.43 \          # Ganancia proporcional del PID
  --pid-ki 0.002 \          # Ganancia integral del PID
  --pid-kd 0.12 \           # Ganancia derivativa del PID
  --pid-tolerance 40 \      # Tolerancia para detección de recta
  --threshold 180            # Umbral de procesamiento de imagen
```

### Valores por Defecto:
- `--pid-kp`: 0.43
- `--pid-ki`: 0.002
- `--pid-kd`: 0.12
- `--pid-tolerance`: 40
- `--threshold`: 180

## 🔧 Cómo Funciona

1. **Video Streamer**: Captura frames de la cámara USB
2. **Lane Detector**: Detecta carriles en cada frame usando visión por computadora
3. **Angle Converter**: Convierte el ángulo detectado a valores de servo (50-135)
4. **Command Sender**: Envía comandos de dirección al ESP32 vía UART
5. **Control Loop**: Ejecuta a ~30 FPS para control suave

## 📡 Endpoints del Autopilot

### Iniciar Autopilot
```bash
curl -X POST http://localhost:5000/autopilot/start
```

### Detener Autopilot
```bash
curl -X POST http://localhost:5000/autopilot/stop
```

### Ver Estado
```bash
curl http://localhost:5000/autopilot/status
```

Respuesta ejemplo:
```json
{
  "is_running": true,
  "last_pid_angle": -15.5,
  "last_servo_angle": 95,
  "command_count": 1234,
  "error_count": 2
}
```

## 🎮 Control del Vehículo

Antes de iniciar el autopilot, asegúrate de:

1. **Armar el sistema:**
   ```bash
   curl -X POST http://localhost:5000/arm
   ```

2. **Establecer modo AUTO:**
   ```bash
   curl "http://localhost:5000/mode?value=AUTO"
   ```

3. **Configurar velocidad (opcional):**
   El autopilot solo controla la dirección. Puedes establecer una velocidad constante:
   ```bash
   curl "http://localhost:5000/changeSpeed?speed=150&direction=forward"
   ```

4. **Iniciar autopilot:**
   ```bash
   curl -X POST http://localhost:5000/autopilot/start
   ```

## 🐛 Solución de Problemas

### El autopilot no se inicia

1. **Verifica que el UART esté conectado:**
   ```bash
   curl http://localhost:5000/health
   ```
   Debe mostrar `"uart_connected": true`

2. **Verifica que la cámara esté disponible:**
   - En Linux: `ls /dev/video*`
   - En Windows: Verifica que la cámara esté conectada y funcionando

3. **Revisa los logs del servidor:**
   - Busca mensajes como "Video streamer initialized"
   - Busca mensajes como "Auto-pilot controller started"

### El autopilot no detecta carriles

1. **Ajusta los parámetros PID:**
   - Aumenta `--pid-kp` para respuesta más rápida
   - Ajusta `--threshold` para mejor detección de bordes

2. **Verifica la iluminación:**
   - El detector funciona mejor con buena iluminación
   - Evita sombras y reflejos fuertes

3. **Revisa las imágenes de debug:**
   - El dashboard puede mostrar imágenes de debug del detector
   - Usa los endpoints de debug para ver qué está detectando

## 📊 Monitoreo

El dashboard muestra:
- Estado del autopilot (running/stopped)
- Último ángulo PID detectado
- Último ángulo de servo enviado
- Contador de comandos enviados
- Contador de errores

## 🔒 Seguridad

- El autopilot requiere que el sistema esté ARMADO
- El modo debe estar en AUTO para funcionar correctamente
- Siempre puedes usar `POST /brake` para freno de emergencia
- El autopilot se detiene automáticamente si hay errores críticos

## 📝 Notas

- El autopilot controla **solo la dirección**. La velocidad debe establecerse manualmente.
- El loop de control corre a ~30 FPS para suavidad
- Los comandos se envían vía UART al ESP32 usando el protocolo `C:SET_STEER:<angle>`
- El sistema requiere heartbeat continuo en modo AUTO (el autopilot lo proporciona implícitamente)

