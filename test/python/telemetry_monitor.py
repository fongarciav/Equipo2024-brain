#!/usr/bin/env python3
"""
ESP32 Telemetry Monitor

A simple serial monitor that continuously reads and displays all output
from the ESP32. Similar to a serial port monitor, but specifically
designed for the ESP32 telemetry and logs.

Usage:
    python telemetry_monitor.py [port] [--baud RATE]

Example:
    python telemetry_monitor.py /dev/ttyUSB0
    python telemetry_monitor.py COM5 --baud 115200
"""
import sys
import time
import argparse

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    print("pyserial not installed. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def select_port_interactive() -> str:
    """Interactively select a serial port."""
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Connect the ESP32 and try again.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx}] {p.device} - {p.description}")
    while True:
        sel = input("Select port index: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Try again.")


def monitor_serial(port: str, baud: int) -> None:
    """Continuously read and display serial output."""
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        print(f"Connected to {port} at {baud} baud")
        print("Monitoring ESP32 telemetry and logs...")
        print("Press Ctrl+C to exit")
        print("-" * 60)
        
        while True:
            try:
                # Read line by line
                data = ser.readline()
                if data:
                    line = data.decode(errors="ignore").rstrip()
                    if line:
                        # Add timestamp (optional, can be removed if not needed)
                        timestamp = time.strftime("%H:%M:%S", time.localtime())
                        print(f"[{timestamp}] {line}")
            except KeyboardInterrupt:
                print("\n\nStopping monitor...")
                break
            except Exception as e:
                print(f"\nError reading serial: {e}", file=sys.stderr)
                break
                
    except serial.SerialException as se:
        print(f"Failed to open {port}: {se}", file=sys.stderr)
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ESP32 Telemetry Monitor - Continuously monitor serial output from ESP32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "port",
        nargs="?",
        help="Serial port (e.g., /dev/ttyUSB0, COM5). If omitted, you'll be prompted."
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200 for USB; use 921600 for Serial1)"
    )
    args = parser.parse_args()

    port = args.port or select_port_interactive()
    monitor_serial(port, args.baud)


if __name__ == "__main__":
    main()

