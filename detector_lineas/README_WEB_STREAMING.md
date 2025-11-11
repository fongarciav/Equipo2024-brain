# Web Streaming para Lane Detector - Guía de Configuración en Raspberry Pi

Esta guía explica cómo configurar el streaming web del detector de carriles para acceder desde internet.

## 🚀 Inicio Rápido (Sin Port Forwarding)

**La forma más fácil de acceder desde internet:**

```bash
# 1. Configurar ngrok (solo una vez)
./setup_tunnel.sh ngrok

# 2. Iniciar detector con túnel automático
./start_with_tunnel.sh ngrok 5000
```

¡Eso es todo! El script te mostrará la URL pública para acceder desde cualquier lugar.

**Opciones disponibles**: ngrok, cloudflare, localtunnel, tailscale, zerotier

## Requisitos Previos

1. **Raspberry Pi** con sistema operativo instalado (Raspberry Pi OS recomendado)
2. **Cámara** conectada a la Raspberry Pi
3. **Conexión a internet** (WiFi o Ethernet)
4. **Python 3.7+** instalado

## Instalación

### 1. Instalar Dependencias

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar dependencias del sistema
sudo apt install -y python3-pip python3-opencv python3-numpy

# Instalar dependencias de Python
pip3 install -r requirements.txt
```

### 2. Verificar Cámara

```bash
# Listar dispositivos de video disponibles
ls -l /dev/video*

# Probar cámara con v4l2
v4l2-ctl --list-devices
```

## Uso Básico

### Modo Local (con ventanas OpenCV)

```bash
cd detector_lineas
python3 deteccion_carril.py
```

### Modo Web Streaming (sin ventanas locales)

```bash
cd detector_lineas
python3 deteccion_carril.py --web-stream --no-display
```

### Modo Web Streaming + Ventanas Locales

```bash
cd detector_lineas
python3 deteccion_carril.py --web-stream
```

### Con Control UART

```bash
cd detector_lineas
python3 lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --web-stream
```

## Acceso desde la Red Local

Una vez iniciado el streaming, verás un mensaje como:

```
🌐 Web streamer started!
   Access at: http://192.168.1.100:5000
   Or from network: http://<raspberry-pi-ip>:5000
```

### Encontrar la IP de tu Raspberry Pi

```bash
# Opción 1: Usando hostname
hostname -I

# Opción 2: Usando ip
ip addr show

# Opción 3: Desde otro dispositivo en la misma red
ping raspberrypi.local
```

### Acceder desde el Navegador

1. Abre un navegador web en cualquier dispositivo conectado a la misma red
2. Navega a: `http://<IP-DE-RASPBERRY-PI>:5000`
3. Deberías ver dos ventanas de video:
   - **Lane Detection**: Vista con detección de carriles y ángulo de dirección
   - **Canny Edge Detection**: Vista del procesamiento de bordes

## Acceso desde Internet (SIN Port Forwarding)

**¡No necesitas configurar port forwarding!** Hay varias alternativas más fáciles y seguras:

### 🚀 Opción Rápida: Script Automático

El método más fácil es usar el script helper incluido:

```bash
# Opción 1: Configurar y usar ngrok (recomendado)
./setup_tunnel.sh ngrok
# Luego en otra terminal:
python3 deteccion_carril.py --web-stream --no-display

# Opción 2: Iniciar todo automáticamente
./start_with_tunnel.sh ngrok 5000
```

### 📋 Opciones Disponibles

#### 1. ngrok (⭐ Recomendado - Más Fácil)

**Ventajas**: Muy fácil de usar, HTTPS incluido, gratis con limitaciones

```bash
# Instalar y configurar (solo una vez)
./setup_tunnel.sh ngrok

# Usar: En una terminal, inicia el detector:
python3 deteccion_carril.py --web-stream --no-display

# En otra terminal, inicia el túnel:
ngrok http 5000
```

O usa el script automático:
```bash
./start_with_tunnel.sh ngrok 5000
```

**Setup inicial**:
1. Crea cuenta gratis en https://dashboard.ngrok.com/signup
2. Obtén tu authtoken de https://dashboard.ngrok.com/get-started/your-authtoken
3. Ejecuta: `ngrok config add-authtoken <TU_TOKEN>`

**Límites del plan gratis**: 
- 1 túnel simultáneo
- URLs temporales (cambian al reiniciar)
- Límite de conexiones

---

#### 2. Cloudflare Tunnel (☁️ Más Estable)

**Ventajas**: Gratis, sin límites, URLs más estables, HTTPS incluido

```bash
# Instalar (solo una vez)
./setup_tunnel.sh cloudflare

# Usar
cloudflared tunnel --url http://localhost:5000
```

O con el script automático:
```bash
./start_with_tunnel.sh cloudflare 5000
```

**Ventajas sobre ngrok**:
- Sin límites de conexiones
- URLs más estables
- Completamente gratis

---

#### 3. LocalTunnel (🌐 Simple)

**Ventajas**: Muy simple, no requiere cuenta

```bash
# Requiere Node.js instalado
./setup_tunnel.sh localtunnel

# Usar
lt --port 5000
```

O con el script automático:
```bash
./start_with_tunnel.sh localtunnel 5000
```

**Nota**: Requiere Node.js. Instalar con:
```bash
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

---

#### 4. Tailscale VPN (🔒 Más Seguro)

**Ventajas**: VPN privada, muy seguro, acceso desde cualquier dispositivo

```bash
# Instalar (solo una vez)
./setup_tunnel.sh tailscale

# Conectar
sudo tailscale up

# Acceder desde cualquier dispositivo con Tailscale instalado
# Usa la IP de la Raspberry Pi en la red Tailscale
```

**Setup**:
1. Instala Tailscale en tu Raspberry Pi y en tus dispositivos
2. Conecta todos a la misma red Tailscale
3. Accede usando la IP privada de Tailscale: `http://<tailscale-ip>:5000`

**Ventajas**:
- Muy seguro (cifrado end-to-end)
- Acceso a toda la red local, no solo el puerto 5000
- Funciona detrás de NAT/firewalls sin configuración

---

#### 5. ZeroTier VPN (🌍 Alternativa a Tailscale)

Similar a Tailscale, otra opción de VPN mesh:

```bash
# Instalar
./setup_tunnel.sh zerotier

# Unirse a una red (necesitas crear una en zerotier.com)
sudo zerotier-cli join <NETWORK_ID>
```

---

### Comparación de Opciones

| Opción | Dificultad | Seguridad | Estabilidad | Gratis | Requiere Cuenta |
|--------|-----------|-----------|-------------|--------|-----------------|
| **ngrok** | ⭐ Muy Fácil | ⭐⭐ Media | ⭐⭐ Media | ✅ Sí | ✅ Sí |
| **Cloudflare** | ⭐⭐ Fácil | ⭐⭐⭐ Alta | ⭐⭐⭐ Alta | ✅ Sí | ✅ Sí |
| **LocalTunnel** | ⭐ Muy Fácil | ⭐ Baja | ⭐⭐ Media | ✅ Sí | ❌ No |
| **Tailscale** | ⭐⭐ Fácil | ⭐⭐⭐⭐ Muy Alta | ⭐⭐⭐ Alta | ✅ Sí | ✅ Sí |
| **ZeroTier** | ⭐⭐ Fácil | ⭐⭐⭐⭐ Muy Alta | ⭐⭐⭐ Alta | ✅ Sí | ✅ Sí |

### 🎯 Recomendación

- **Para empezar rápido**: Usa **ngrok** con el script automático
- **Para uso continuo**: Usa **Cloudflare Tunnel** (más estable)
- **Para máxima seguridad**: Usa **Tailscale** (VPN privada)

### ⚠️ Port Forwarding (Solo si realmente lo necesitas)

Si prefieres usar port forwarding tradicional (no recomendado por seguridad):

1. Configura IP estática en tu router
2. Configura port forwarding del puerto 5000 a tu Raspberry Pi
3. Accede usando tu IP pública: `http://<TU-IP-PUBLICA>:5000`

**Desventajas**:
- Expone tu IP pública directamente
- Requiere configuración del router
- Menos seguro
- Puede no funcionar si tu ISP usa NAT

## Seguridad

⚠️ **ADVERTENCIA**: Exponer el streaming a internet sin protección puede ser un riesgo de seguridad.

### Recomendaciones de Seguridad

1. **Usar HTTPS**: Configura un certificado SSL (Let's Encrypt) o usa ngrok/Cloudflare que ya incluyen HTTPS
2. **Autenticación**: Considera agregar autenticación básica HTTP si expones directamente
3. **Firewall**: Configura reglas de firewall para limitar acceso
4. **VPN**: La opción más segura es usar una VPN para acceder a tu red local

### Agregar Autenticación Básica (Opcional)

Puedes modificar `web_streamer.py` para agregar autenticación básica usando Flask-BasicAuth:

```bash
pip3 install flask-basicauth
```

Luego modifica `web_streamer.py` para incluir autenticación.

## Solución de Problemas

### El streaming no se inicia

```bash
# Verificar que Flask está instalado
pip3 show flask

# Verificar que el puerto no está en uso
sudo netstat -tulpn | grep 5000

# Probar con otro puerto
python3 deteccion_carril.py --web-stream --web-port 8080
```

### No puedo acceder desde otro dispositivo

1. Verifica que ambos dispositivos están en la misma red
2. Verifica el firewall de la Raspberry Pi:
   ```bash
   sudo ufw allow 5000/tcp
   ```
3. Verifica que el servidor está escuchando en `0.0.0.0` (no solo `localhost`)

### La cámara no funciona

```bash
# Verificar permisos de usuario
groups  # Debe incluir 'video'

# Agregar usuario al grupo video si es necesario
sudo usermod -a -G video $USER
# Luego cerrar sesión y volver a iniciar

# Probar acceso a la cámara
v4l2-ctl --device=/dev/video0 --all
```

### Alto uso de CPU

- Reduce la resolución del streaming modificando `target_width` y `target_height`
- Reduce la calidad JPEG en `web_streamer.py` (línea con `cv2.IMWRITE_JPEG_QUALITY`)
- Usa `--no-display` si no necesitas las ventanas locales

## Rendimiento

- **Resolución recomendada**: 640x480 para mejor rendimiento
- **FPS**: ~30 FPS en Raspberry Pi 4
- **Ancho de banda**: ~1-2 Mbps por stream

## Ejemplos de Uso

### Ejemplo 1: Streaming solo web (headless)

```bash
python3 deteccion_carril.py --web-stream --no-display --web-port 5000
```

### Ejemplo 2: Streaming con control UART

```bash
python3 lane_follower.py \
  --uart-port /dev/ttyUSB0 \
  --uart-baud 115200 \
  --arm-system \
  --speed 200 \
  --web-stream \
  --web-port 5000 \
  --no-display
```

### Ejemplo 3: Ejecutar como servicio del sistema

Crea `/etc/systemd/system/lane-detector.service`:

```ini
[Unit]
Description=Lane Detector Web Streaming
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/projects/bfmc/brain/detector_lineas
ExecStart=/usr/bin/python3 deteccion_carril.py --web-stream --no-display
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Luego:

```bash
sudo systemctl enable lane-detector.service
sudo systemctl start lane-detector.service
sudo systemctl status lane-detector.service
```

## Soporte

Si encuentras problemas:
1. Revisa los logs del sistema
2. Verifica que todas las dependencias están instaladas
3. Asegúrate de que la cámara funciona correctamente
4. Verifica la conectividad de red

