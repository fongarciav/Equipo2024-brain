#!/bin/bash
# Setup script for tunnel services to avoid port forwarding
# This script helps set up various tunnel services for web streaming

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TUNNEL_TYPE="${1:-ngrok}"
PORT="${2:-5000}"

echo "🚇 Tunnel Setup Script for Lane Detector"
echo "=========================================="
echo ""

case "$TUNNEL_TYPE" in
    ngrok)
        echo "📦 Setting up ngrok..."
        
        # Check if ngrok is already installed
        if command -v ngrok &> /dev/null; then
            echo "✅ ngrok is already installed"
        else
            echo "📥 Installing ngrok..."
            
            # Detect architecture
            ARCH=$(uname -m)
            if [ "$ARCH" = "armv7l" ] || [ "$ARCH" = "aarch64" ]; then
                echo "Detected ARM architecture"
                NGROK_URL="https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-arm.tgz"
            else
                echo "Detected x86_64 architecture"
                NGROK_URL="https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-amd64.tgz"
            fi
            
            cd /tmp
            wget -q "$NGROK_URL" -O ngrok.tgz
            tar xzf ngrok.tgz
            sudo mv ngrok /usr/local/bin/
            rm ngrok.tgz
            echo "✅ ngrok installed successfully"
        fi
        
        # Check if authtoken is configured
        if ngrok config check &> /dev/null; then
            echo "✅ ngrok is configured"
        else
            echo ""
            echo "⚠️  ngrok requires an authtoken"
            echo "1. Sign up at https://dashboard.ngrok.com/signup (free)"
            echo "2. Get your authtoken from https://dashboard.ngrok.com/get-started/your-authtoken"
            echo "3. Run: ngrok config add-authtoken <YOUR_TOKEN>"
            echo ""
            read -p "Do you want to configure it now? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                read -p "Enter your ngrok authtoken: " token
                ngrok config add-authtoken "$token"
                echo "✅ ngrok configured!"
            fi
        fi
        
        echo ""
        echo "🚀 Starting ngrok tunnel on port $PORT..."
        echo "   Access your stream at the URL shown below"
        echo "   Press Ctrl+C to stop"
        echo ""
        ngrok http "$PORT"
        ;;
        
    cloudflare)
        echo "☁️  Setting up Cloudflare Tunnel..."
        
        # Check if cloudflared is installed
        if command -v cloudflared &> /dev/null; then
            echo "✅ cloudflared is already installed"
        else
            echo "📥 Installing cloudflared..."
            
            ARCH=$(uname -m)
            if [ "$ARCH" = "armv7l" ] || [ "$ARCH" = "aarch64" ]; then
                CLOUDFLARE_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm"
            else
                CLOUDFLARE_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64"
            fi
            
            wget -q "$CLOUDFLARE_URL" -O /tmp/cloudflared
            chmod +x /tmp/cloudflared
            sudo mv /tmp/cloudflared /usr/local/bin/
            echo "✅ cloudflared installed successfully"
        fi
        
        echo ""
        echo "🚀 Starting Cloudflare Tunnel on port $PORT..."
        echo "   Access your stream at the URL shown below"
        echo "   Press Ctrl+C to stop"
        echo ""
        cloudflared tunnel --url "http://localhost:$PORT"
        ;;
        
    localtunnel)
        echo "🌐 Setting up localtunnel..."
        
        # Check if node/npm is installed
        if ! command -v node &> /dev/null; then
            echo "❌ Node.js is required for localtunnel"
            echo "Install with: curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt-get install -y nodejs"
            exit 1
        fi
        
        # Check if localtunnel is installed globally
        if npm list -g localtunnel &> /dev/null; then
            echo "✅ localtunnel is already installed"
        else
            echo "📥 Installing localtunnel..."
            sudo npm install -g localtunnel
            echo "✅ localtunnel installed successfully"
        fi
        
        echo ""
        echo "🚀 Starting localtunnel on port $PORT..."
        echo "   Access your stream at the URL shown below"
        echo "   Press Ctrl+C to stop"
        echo ""
        lt --port "$PORT"
        ;;
        
    tailscale)
        echo "🔒 Setting up Tailscale VPN..."
        
        # Check if tailscale is installed
        if command -v tailscale &> /dev/null; then
            echo "✅ Tailscale is already installed"
        else
            echo "📥 Installing Tailscale..."
            curl -fsSL https://tailscale.com/install.sh | sh
            echo "✅ Tailscale installed successfully"
        fi
        
        # Check if tailscale is running
        if sudo tailscale status &> /dev/null; then
            echo "✅ Tailscale is running"
            IP=$(hostname -I | awk '{print $1}')
            echo ""
            echo "🌐 Your Raspberry Pi IP in Tailscale network: $IP"
            echo "   Access your stream at: http://$IP:$PORT"
            echo "   Or use the Tailscale hostname: http://$(hostname):$PORT"
        else
            echo ""
            echo "⚠️  Tailscale is not connected"
            echo "Run: sudo tailscale up"
            echo "Then access your stream from any device connected to your Tailscale network"
        fi
        ;;
        
    zerotier)
        echo "🌍 Setting up ZeroTier VPN..."
        
        # Check if zerotier is installed
        if command -v zerotier-cli &> /dev/null; then
            echo "✅ ZeroTier is already installed"
        else
            echo "📥 Installing ZeroTier..."
            curl -s https://install.zerotier.com | sudo bash
            echo "✅ ZeroTier installed successfully"
        fi
        
        # Check if zerotier is running
        if sudo zerotier-cli status &> /dev/null; then
            NETWORK_ID=$(sudo zerotier-cli listnetworks | grep -oP '^\d+' | head -1)
            if [ -n "$NETWORK_ID" ]; then
                echo "✅ ZeroTier is connected to network: $NETWORK_ID"
                IP=$(hostname -I | awk '{print $1}')
                echo ""
                echo "🌐 Your Raspberry Pi IP in ZeroTier network: $IP"
                echo "   Access your stream at: http://$IP:$PORT"
            else
                echo "⚠️  ZeroTier is not connected to any network"
                echo "Join a network with: sudo zerotier-cli join <NETWORK_ID>"
            fi
        else
            echo "⚠️  ZeroTier service is not running"
            echo "Start with: sudo systemctl start zerotier-one"
        fi
        ;;
        
    *)
        echo "❌ Unknown tunnel type: $TUNNEL_TYPE"
        echo ""
        echo "Available options:"
        echo "  ngrok        - Easy to use, free tier available (recommended)"
        echo "  cloudflare   - Free, stable, from Cloudflare"
        echo "  localtunnel  - Simple, requires Node.js"
        echo "  tailscale    - VPN mesh network (most secure)"
        echo "  zerotier     - VPN mesh network (alternative)"
        echo ""
        echo "Usage: $0 [tunnel_type] [port]"
        echo "Example: $0 ngrok 5000"
        exit 1
        ;;
esac

