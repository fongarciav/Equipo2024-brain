#!/bin/bash
cd "$(dirname "$0")/brain" || exit 1
pkill -f "dashboard_server.py" 2>/dev/null
# Kill any process using port 5000
lsof -ti:5000 | xargs kill -9 2>/dev/null || true
sleep 1
exec python dashboard/dashboard_server.py --host 0.0.0.0 --port 5000
