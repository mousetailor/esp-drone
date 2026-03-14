#!/usr/bin/env python3
"""
ESP32 Drone — Automatic MAC Address Configuration Helper
==========================================================

This script automatically:
1. Reads the MAC addresses from both ESPs via serial monitor
2. Extracts the hex values
3. Updates the firmware files with correct MACs
4. Tells you what to rebuild

Usage:
    python3 fix_mac_addresses.py /dev/ttyACM0 /dev/ttyUSB0

Or (auto-detect):
    python3 fix_mac_addresses.py
"""

import os
import re
import subprocess
import sys
import time
from pathlib import Path

import serial


def find_available_ports():
    """Try to find ESP32 ports automatically"""
    import glob

    ports = []
    # Common patterns
    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*", "COM*"]:
        ports.extend(glob.glob(pattern))
    return sorted(list(set(ports)))


def read_mac_from_esp(port, timeout=10.0):
    """
    Connect to an ESP32 and extract its MAC address from startup logs.
    Returns (mac_address, role) or (None, None) if not found.
    """
    try:
        ser = serial.Serial(port, 115200, timeout=2.0)
    except Exception as e:
        print(f"  ERROR: Cannot connect to {port}: {e}")
        return None, None

    print(f"  Reading from {port}...")

    start_time = time.time()
    mac_pattern = re.compile(r"([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5})")
    role = None
    mac = None

    while time.time() - start_time < timeout:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Identify role
            if "Drone MAC:" in line:
                role = "DRONE"
                match = mac_pattern.search(line)
                if match:
                    mac = match.group(1)
                    break
            elif "Controller MAC:" in line or "CTRL" in line.upper():
                role = "CONTROLLER"
                match = mac_pattern.search(line)
                if match:
                    mac = match.group(1)
                    break
        except Exception:
            pass

    ser.close()
    return mac, role


def mac_str_to_hex_bytes(mac_str):
    """Convert 'AA:BB:CC:DD:EE:FF' to '{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}'"""
    if not mac_str or ":" not in mac_str:
        return None
    parts = mac_str.split(":")
    if len(parts) != 6:
        return None
    hex_bytes = ", ".join(f"0x{p.upper()}" for p in parts)
    return f"{{{hex_bytes}}}"


def update_controller_firmware(drone_mac_hex):
    """Update controller_main.c with drone MAC"""
    ctrl_path = (
        Path(__file__).parent.parent / "controller" / "main" / "controller_main.c"
    )

    if not ctrl_path.exists():
        print(f"  ERROR: Cannot find {ctrl_path}")
        return False

    content = ctrl_path.read_text()

    # Find and replace the drone_mac line
    pattern = r"static uint8_t drone_mac\[6\]\s*=\s*\{[^}]+\};"
    replacement = f"static uint8_t drone_mac[6] = {drone_mac_hex};"

    new_content = re.sub(pattern, replacement, content)

    if new_content == content:
        print(f"  WARNING: Could not find drone_mac pattern in {ctrl_path}")
        return False

    ctrl_path.write_text(new_content)
    print(f"  ✅ Updated {ctrl_path}")
    return True


def update_drone_firmware(controller_mac_hex):
    """Update drone_main.c with controller MAC"""
    drone_path = Path(__file__).parent.parent / "drone" / "main" / "drone_main.c"

    if not drone_path.exists():
        print(f"  ERROR: Cannot find {drone_path}")
        return False

    content = drone_path.read_text()

    # Find and replace the controller_mac line
    pattern = r"static uint8_t controller_mac\[6\]\s*=\s*\{[^}]+\};"
    replacement = f"static uint8_t controller_mac[6] = {controller_mac_hex};"

    new_content = re.sub(pattern, replacement, content)

    if new_content == content:
        print(f"  WARNING: Could not find controller_mac pattern in {drone_path}")
        return False

    drone_path.write_text(new_content)
    print(f"  ✅ Updated {drone_path}")
    return True


def main():
    print("=" * 70)
    print("ESP32 Drone — Automatic MAC Address Configuration")
    print("=" * 70)

    # Determine ports
    ctrl_port = None
    drone_port = None

    if len(sys.argv) >= 3:
        ctrl_port = sys.argv[1]
        drone_port = sys.argv[2]
    elif len(sys.argv) == 2:
        print("ERROR: Please provide both controller and drone ports")
        print(f"Usage: python3 {sys.argv[0]} /dev/ttyACM0 /dev/ttyUSB0")
        sys.exit(1)
    else:
        # Try to auto-detect
        available = find_available_ports()
        if len(available) < 2:
            print(
                "ERROR: Could not auto-detect ports. Please connect both ESPs and try again."
            )
            print(f"Available ports: {available}")
            print(f"Usage: python3 {sys.argv[0]} /dev/ttyACM0 /dev/ttyUSB0")
            sys.exit(1)

        print(f"Auto-detected ports: {available}")
        ctrl_port = available[0]
        drone_port = available[1] if len(available) > 1 else available[0]
        print(f"Using: controller={ctrl_port}, drone={drone_port}")
        print()

    # Read controller MAC
    print("[1/2] Reading CONTROLLER ESP32...")
    ctrl_mac, ctrl_role = read_mac_from_esp(ctrl_port)

    if not ctrl_mac:
        print(f"  ERROR: Could not read MAC from {ctrl_port}")
        print("  Make sure the ESP32 is connected and flashed")
        sys.exit(1)

    print(f"  ✅ Found: {ctrl_mac} ({ctrl_role})")

    # Read drone MAC
    print("\n[2/2] Reading DRONE ESP32...")
    drone_mac, drone_role = read_mac_from_esp(drone_port)

    if not drone_mac:
        print(f"  ERROR: Could not read MAC from {drone_port}")
        print("  Make sure the ESP32 is connected and flashed")
        sys.exit(1)

    print(f"  ✅ Found: {drone_mac} ({drone_role})")

    # Validate
    if ctrl_role == drone_role:
        print("\n⚠️  WARNING: Both ESPs detected as the same role!")
        print(f"   Controller detected as: {ctrl_role}")
        print(f"   Drone detected as: {drone_role}")
        print("   This is probably wrong. Check your ports!")
        response = input("Continue anyway? (y/N): ")
        if response.lower() != "y":
            sys.exit(1)

    # Convert to hex format
    ctrl_hex = mac_str_to_hex_bytes(ctrl_mac)
    drone_hex = mac_str_to_hex_bytes(drone_mac)

    if not ctrl_hex or not drone_hex:
        print("\nERROR: Could not parse MAC addresses")
        sys.exit(1)

    # Update firmware files
    print("\n" + "=" * 70)
    print("Updating firmware files...")
    print("=" * 70)

    print(f"\nController firmware:")
    print(f"  drone_mac = {drone_hex}")
    update_controller_firmware(drone_hex)

    print(f"\nDrone firmware:")
    print(f"  controller_mac = {ctrl_hex}")
    update_drone_firmware(ctrl_hex)

    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! MAC addresses have been updated.")
    print("=" * 70)
    print(f"\nController: {ctrl_mac}")
    print(f"Drone:      {drone_mac}")
    print("\nNext steps:")
    print("\n1. Rebuild and flash the controller:")
    print("   cd esp/controller")
    print("   idf.py build")
    print(f"   idf.py -p {ctrl_port} flash")
    print("\n2. Rebuild and flash the drone:")
    print("   cd esp/drone")
    print("   idf.py build")
    print(f"   idf.py -p {drone_port} flash")
    print("\n3. Power on the drone, then run:")
    print("   cd esp/pc")
    print(f"   python3 controller_pc.py {ctrl_port}")
    print("\nDone! 🚁")


if __name__ == "__main__":
    main()
