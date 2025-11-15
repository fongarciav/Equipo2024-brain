# ESP32 Car Control Dashboard

Dashboard web completo para controlar el ESP32 vía **UART** desde cualquier dispositivo en la red.

## 📁 Contenido

- `dashboard.html` - Interfaz web del dashboard
- `dashboard_server.py` - Servidor Flask que traduce comandos HTTP a UART
- `requirements.txt` - Dependencias de Python (Flask, pyserial)
- `start_dashboard.sh` - Script de inicio rápido
- `DASHBOARD_README.md` - Documentación del dashboard
- `DASHBOARD_SERVER_README.md` - Documentación del servidor

## 🚀 Inicio Rápido

### Opción 1: Conectar UART automáticamente al iniciar
```bash
cd dashboard
pip install -r requirements.txt
python dashboard_server.py --uart-port /dev/ttyUSB0
# En Windows: python dashboard_server.py --uart-port COM3
```

### Opción 2: Conectar UART desde la interfaz web
```bash
cd dashboard
pip install -r requirements.txt
python dashboard_server.py
```
Luego en el navegador:
1. Abre el dashboard (http://localhost:5000)
2. Haz clic en "Refresh Ports"
3. Selecciona el puerto UART
4. Haz clic en "Connect UART"

### Opción 3: Usar el script de inicio
```bash
cd dashboard
./start_dashboard.sh --uart-port /dev/ttyUSB0
```

## 🌐 Acceso

Una vez iniciado el servidor, verás algo como:

```
============================================================
ESP32 Car Control Dashboard Server (UART Mode)
============================================================
Dashboard available at:
  Local:   http://127.0.0.1:5000
  Network: http://192.168.1.100:5000
============================================================
UART: Connected to /dev/ttyUSB0
============================================================
```

- **Local**: Acceso desde la misma máquina
- **Network**: Acceso desde cualquier dispositivo en la misma red (usar esta IP)
- **UART**: Conexión serial al ESP32 (921600 baud)

## ✨ Características

- ✅ **Comunicación UART**: Conecta directamente al ESP32 vía puerto serial
- ✅ Control completo del ESP32 (motor, dirección, sistema)
- ✅ Interfaz web moderna y responsive
- ✅ Selección de puerto UART desde la interfaz
- ✅ Log de eventos con comandos UART mostrados
- ✅ Accesible desde cualquier dispositivo en la red
- ✅ Fácil de migrar (todo en esta carpeta)

## 🔌 Configuración UART

- **Baud rate**: 921600 (configurado automáticamente)
- **Puerto**: Configurable (ej: `/dev/ttyUSB0` en Linux, `COM3` en Windows)
- **Protocolo**: `CHANNEL:COMMAND:VALUE\n` (ver `docs/BRAIN_TEAM_PROTOCOL.md`)

## 📖 Documentación

- Ver `DASHBOARD_README.md` para detalles sobre el uso del dashboard
- Ver `DASHBOARD_SERVER_README.md` para detalles sobre el servidor Flask

## 🔧 Migración

Para migrar el dashboard a otra máquina:

1. Copiar toda la carpeta `dashboard`
2. Instalar dependencias: `pip install -r requirements.txt`
3. Ejecutar: `python dashboard_server.py`

Todo está contenido en esta carpeta, por lo que es fácil de mover.

