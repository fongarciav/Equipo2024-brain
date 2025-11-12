# Instalación de TensorRT en Jetson

Este documento explica cómo instalar TensorRT y PyCUDA en NVIDIA Jetson para usar modelos `.engine`.

## Verificar instalación actual

```bash
# Verificar si TensorRT está instalado
python3 -c "import tensorrt as trt; print('TensorRT version:', trt.__version__)"

# Verificar si PyCUDA está instalado
python3 -c "import pycuda.driver as cuda; print('PyCUDA available')"
```

## Instalación en Jetson

### Opción 1: Instalación via apt (Recomendado)

TensorRT normalmente viene preinstalado con JetPack, pero si falta:

```bash
# Actualizar repositorios
sudo apt-get update

# Instalar TensorRT
sudo apt-get install python3-libnvinfer python3-libnvinfer-dev

# Instalar PyCUDA
sudo apt-get install python3-pycuda

# Verificar instalación
python3 -c "import tensorrt as trt; print('TensorRT:', trt.__version__)"
python3 -c "import pycuda.driver as cuda; print('PyCUDA: OK')"
```

### Opción 2: Instalación manual de PyCUDA (si apt no funciona)

```bash
# Instalar dependencias del sistema
sudo apt-get install python3-dev python3-pip

# Instalar PyCUDA desde pip (puede requerir compilación)
pip3 install pycuda

# O desde fuente si es necesario
# git clone https://github.com/inducer/pycuda.git
# cd pycuda
# python3 configure.py --cuda-root=/usr/local/cuda
# python3 setup.py build
# sudo python3 setup.py install
```

### Opción 3: Verificar JetPack completo

Si TensorRT no está disponible, puede que necesites reinstalar JetPack o actualizar:

```bash
# Verificar versión de JetPack
cat /etc/nv_tegra_release

# Verificar CUDA
nvcc --version

# Verificar TensorRT
dpkg -l | grep tensorrt
```

## Verificar que funciona

Después de instalar, prueba:

```bash
cd detector_lineas
python3 deteccion_carril.py --signal-detection --signal-model /ruta/a/tu/best.engine
```

Deberías ver:
```
✓ Signal detector loaded (TensorRT): /ruta/a/tu/best.engine
  Device: NVIDIA GPU (TensorRT)
  Input shape: (640, 640)
  GPU acceleration: ENABLED (TensorRT)
```

## Troubleshooting

### Error: "No module named 'tensorrt'"

1. Verifica que TensorRT esté instalado:
   ```bash
   dpkg -l | grep tensorrt
   ```

2. Verifica la ruta de Python:
   ```bash
   python3 -c "import sys; print(sys.path)"
   ```

3. Si TensorRT está instalado pero Python no lo encuentra, puede ser un problema de PYTHONPATH:
   ```bash
   export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
   ```

### Error: "No module named 'pycuda'"

1. Instala PyCUDA:
   ```bash
   sudo apt-get install python3-pycuda
   ```

2. O instala desde pip:
   ```bash
   pip3 install pycuda
   ```

### Error al compilar PyCUDA

Si `pip install pycuda` falla, usa la versión de apt:
```bash
sudo apt-get install python3-pycuda
```

## Notas importantes

- En Jetson, TensorRT normalmente viene con JetPack y está optimizado para la GPU integrada
- PyCUDA es necesario para la comunicación con CUDA desde Python
- Si tienes problemas, asegúrate de estar usando Python 3.10 (compatible con JetPack 5.x)

