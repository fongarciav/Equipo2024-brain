#!/bin/bash
# Start lane detector with automatic tunnel setup
# This script starts the lane detector and sets up a tunnel automatically

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TUNNEL_TYPE="${1:-ngrok}"
PORT="${2:-5000}"
DETECTOR_SCRIPT="${3:-deteccion_carril.py}"

echo "🚗 Starting Lane Detector with Tunnel"
echo "======================================"
echo "Tunnel type: $TUNNEL_TYPE"
echo "Port: $PORT"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $DETECTOR_PID $TUNNEL_PID 2>/dev/null || true
    wait $DETECTOR_PID $TUNNEL_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start the detector in background
echo "📹 Starting lane detector..."
python3 "$DETECTOR_SCRIPT" --web-stream --no-display --web-port "$PORT" &
DETECTOR_PID=$!

# Wait a bit for the server to start
sleep 3

# Check if detector is running
if ! kill -0 $DETECTOR_PID 2>/dev/null; then
    echo "❌ Failed to start detector"
    exit 1
fi

echo "✅ Detector started (PID: $DETECTOR_PID)"
echo ""

# Start tunnel based on type
case "$TUNNEL_TYPE" in
    ngrok)
        if ! command -v ngrok &> /dev/null; then
            echo "❌ ngrok not found. Run: $SCRIPT_DIR/setup_tunnel.sh ngrok"
            cleanup
            exit 1
        fi
        echo "🚇 Starting ngrok tunnel..."
        ngrok http "$PORT" &
        TUNNEL_PID=$!
        sleep 2
        echo "✅ Tunnel started!"
        echo ""
        echo "🌐 Access your stream:"
        curl -s http://localhost:4040/api/tunnels | grep -o 'https://[^"]*\.ngrok[^"]*' | head -1 || echo "   Check ngrok web interface: http://localhost:4040"
        ;;
        
    cloudflare)
        if ! command -v cloudflared &> /dev/null; then
            echo "❌ cloudflared not found. Run: $SCRIPT_DIR/setup_tunnel.sh cloudflare"
            cleanup
            exit 1
        fi
        echo "☁️  Starting Cloudflare tunnel..."
        cloudflared tunnel --url "http://localhost:$PORT" &
        TUNNEL_PID=$!
        sleep 2
        echo "✅ Tunnel started!"
        echo "   Check the URL shown above"
        ;;
        
    localtunnel)
        if ! command -v lt &> /dev/null; then
            echo "❌ localtunnel not found. Run: $SCRIPT_DIR/setup_tunnel.sh localtunnel"
            cleanup
            exit 1
        fi
        echo "🌐 Starting localtunnel..."
        lt --port "$PORT" &
        TUNNEL_PID=$!
        sleep 2
        echo "✅ Tunnel started!"
        echo "   Check the URL shown above"
        ;;
        
    *)
        echo "⚠️  Unknown tunnel type: $TUNNEL_TYPE"
        echo "   Running detector only (local network access)"
        TUNNEL_PID=""
        ;;
esac

echo ""
echo "✅ Everything is running!"
echo "   Press Ctrl+C to stop"
echo ""

# Wait for processes
wait $DETECTOR_PID

