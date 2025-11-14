# Dashboard Server - Flask

Servidor Flask simple para servir el dashboard de control del ESP32 desde cualquier dispositivo en la red.

## Instalación

1. Instalar las dependencias:
```bash
cd test
pip install -r requirements.txt
```

O instalar manualmente:
```bash
pip install Flask flask-cors requests
```

## Uso

### Inicio básico
```bash
cd test
python dashboard_server.py
```

### Opciones avanzadas
```bash
# Especificar puerto personalizado
python dashboard_server.py --port 8080

# Habilitar modo debug
python dashboard_server.py --debug

# Especificar host (por defecto 0.0.0.0 para todas las interfaces)
python dashboard_server.py --host 0.0.0.0 --port 5000
```

## Acceso

Una vez iniciado el servidor, verás algo como:

```
============================================================
ESP32 Car Control Dashboard Server
============================================================
Dashboard available at:
  Local:   http://127.0.0.1:5000
  Network: http://192.168.1.100:5000
============================================================
```

- **Local**: Acceso desde la misma máquina
- **Network**: Acceso desde cualquier dispositivo en la misma red (usar esta IP)

## Características

- ✅ Sirve el dashboard HTML
- ✅ Habilita CORS para evitar problemas de acceso
- ✅ Endpoint de proxy opcional para requests al ESP32 (`/proxy/<endpoint>?ip=<esp32_ip>`)
- ✅ Health check endpoint (`/health`)
- ✅ Fácil de migrar (todo en el directorio `test`)

## Endpoints

- `GET /` - Sirve el dashboard HTML
- `GET /health` - Health check del servidor
- `GET /proxy/<endpoint>?ip=<esp32_ip>` - Proxy para requests al ESP32 (opcional)

## Migración

Para migrar el servidor a otra máquina:

1. Copiar el directorio `test` completo
2. Instalar dependencias: `pip install -r requirements.txt`
3. Ejecutar: `python dashboard_server.py`

Todo está contenido en el directorio `test`, por lo que es fácil de mover.

## Notas

- El servidor se ejecuta en modo desarrollo por defecto
- Para producción, considera usar un servidor WSGI como Gunicorn o uWSGI
- El puerto por defecto es 5000, cámbialo si hay conflictos

