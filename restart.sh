#!/bin/bash
cd "$(dirname "$0")/brain" || exit 1
pkill -f "dashboard_server.py" 2>/dev/null
sleep 1
exec python dashboard/dashboard_server.py --host 0.0.0.0 --port 5000
