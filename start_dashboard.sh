#!/bin/bash

# Script para iniciar el dashboard de forma simple

cd "$(dirname "$0")/brain" || exit 1

echo "=================================================="
echo "Iniciando Dashboard del Robot"
echo "=================================================="
echo ""
echo "El dashboard estará disponible en:"
echo "  http://localhost:5000"
echo ""
echo "Presiona Ctrl+C para detener el servidor"
echo "=================================================="
echo ""

# Kill any existing instance
pkill -f "dashboard_server.py" 2>/dev/null

sleep 1

# Run the dashboard
python dashboard/dashboard_server.py --host 0.0.0.0 --port 5000

