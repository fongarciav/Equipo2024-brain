# ESP32 Car Control Dashboard

A simple web-based dashboard to control the ESP32 car from any computer on the network.

## Features

This dashboard provides controls for all available ESP32 car events:

### System Control
- **ARM/DISARM**: Arm or disarm the system (required before control commands work)
- **MODE**: Switch between MANUAL and AUTO modes
- **EMERGENCY BRAKE**: Immediate emergency stop

### Motor Control
- **Forward/Backward/Stop**: Basic directional controls
- **Speed Slider**: Set speed from 0-255
- **Speed with Direction**: Set specific speed in forward or backward direction

### Steering Control
- **Left/Center/Right**: Quick steering buttons
- **Steering Slider**: Set precise steering angle (50-135 degrees)
  - 50° = Full left
  - 105° = Center
  - 135° = Full right
- **Preset Angles**: Quick buttons for common angles

### Lights Control
- **Lights On/Off/Auto**: Control the car's lights

### Status Monitoring
- Real-time system status (Mode and State)
- Connection status indicator
- Event log showing all sent commands

## Usage

1. **Open the dashboard**: Double-click `dashboard.html` or open it in any web browser

2. **Connect to ESP32**:
   - Enter the ESP32's IP address (default: `192.168.4.1` if connected to ESP32's WiFi AP)
   - Click "Connect"
   - The connection status indicator will turn green when connected

3. **Control the car**:
   - **IMPORTANT**: First ARM the system, then set the mode (MANUAL or AUTO)
   - Use the various controls to send commands to the ESP32
   - Monitor the status panel for real-time system state
   - Check the event log to see all commands being sent

## Network Setup

### Option 1: Connect to ESP32 WiFi AP
- The ESP32 creates a WiFi Access Point named `RC-Car-ESP32`
- Connect your computer to this WiFi network
- Use IP address: `192.168.4.1` (default ESP32 AP IP)

### Option 2: Connect via Local Network
- If ESP32 is connected to your local WiFi network
- Find the ESP32's IP address (check Serial Monitor or router admin)
- Use that IP address in the dashboard

## Available Commands

The dashboard sends HTTP GET requests to the following endpoints:

- `/arm` - Arm the system
- `/disarm` - Disarm the system
- `/mode?value=MANUAL` or `/mode?value=AUTO` - Set system mode
- `/brake` - Emergency brake
- `/forward` - Move forward at max speed
- `/back` - Move backward at max speed
- `/driveStop` - Stop motor
- `/changeSpeed?speed=X&direction=forward` or `backward` - Set speed with direction
- `/left` - Turn left
- `/right` - Turn right
- `/steerStop` - Center steering
- `/steer?angle=X` - Set steering angle (50-135)
- `/LightsOn` - Turn lights on
- `/LightsOff` - Turn lights off
- `/LightsAuto` - Automatic lights mode
- `/status` - Get system status (JSON)

## Troubleshooting

### Commands not working?
1. Make sure the ESP32 is ARMED (check status panel)
2. Make sure the system is in RUNNING state (check status panel)
3. Verify the IP address is correct
4. Check that you're on the same network as the ESP32
5. Check the browser console (F12) for any errors

### CORS Errors?
- Some browsers may show CORS warnings in the console
- This is normal and commands should still work
- The dashboard uses `no-cors` mode to send requests even if CORS headers are missing

### Status not updating?
- Status updates require CORS headers from the ESP32
- If status doesn't update, commands may still work
- Check the event log to verify commands are being sent

## Notes

- This dashboard is designed to work from any computer on the network
- No installation required - just open the HTML file in a browser
- Works best with Chrome, Firefox, or Edge browsers
- Mobile browsers may work but desktop is recommended for better control

