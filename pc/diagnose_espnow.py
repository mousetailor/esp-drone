#!/usr/bin/env python3
"""
ESP32 Drone — ESP-NOW Diagnostic Tool
======================================
Helps identify why the controller and drone aren't communicating.

Run this BEFORE trying to fly:
1. Flash both ESPs with their respective firmware
2. Note the MAC addresses from the serial monitor output
3. Run this script to verify communication
4. Fix any issues it identifies
5. Then try the controller_pc.py script

Usage:
    python3 diagnose_espnow.py CONTROLLER_PORT DRONE_PORT

Example:
    python3 diagnose_espnow.py /dev/ttyACM0 /dev/ttyUSB0
"""

import re
import struct
import sys
import time
from dataclasses import dataclass

import serial


@dataclass
class DeviceInfo:
    """Information parsed from ESP32 startup logs"""

    port: str
    mac: str
    role: str
    target_mac: str
    channel: int = 1
    long_range: bool = False


def connect_serial(port: str) -> serial.Serial:
    """Connect to an ESP32 and return the serial object."""
    try:
        ser = serial.Serial(port, 115200, timeout=2.0)
        return ser
    except Exception as e:
        print(f"ERROR: Cannot connect to {port}")
        print(f"  {e}")
        return None


def extract_device_info(port: str, timeout_sec: float = 10.0) -> DeviceInfo:
    """
    Connect to an ESP32 and extract:
    - Its MAC address
    - Its role (DRONE or CTRL)
    - The target MAC it's configured to talk to
    """
    ser = connect_serial(port)
    if ser is None:
        return None

    print(f"\n[{port}] Reading startup logs (waiting {timeout_sec}s)...")

    start_time = time.time()
    lines = []
    mac_pattern = re.compile(r"([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5})")

    device = DeviceInfo(port=port, mac=None, role=None, target_mac=None)

    while time.time() - start_time < timeout_sec:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            lines.append(line)
            print(f"  {line}")

            # Try to identify device role and MACs
            if "Drone MAC:" in line or "DRONE" in line:
                device.role = "DRONE"
                match = mac_pattern.search(line)
                if match:
                    device.mac = match.group(1)
            elif "Controller MAC:" in line or "CTRL" in line:
                device.role = "CONTROLLER"
                match = mac_pattern.search(line)
                if match:
                    device.mac = match.group(1)

            # Look for target MAC configurations
            if "Target drone MAC:" in line or "drone_mac" in line:
                matches = mac_pattern.findall(line)
                if matches:
                    device.target_mac = matches[-1]
            elif "controller_mac" in line:
                matches = mac_pattern.findall(line)
                if matches:
                    device.target_mac = matches[-1]

            # Check for Long Range mode
            if "Long-Range" in line and "enabled" in line:
                device.long_range = True

        except Exception as e:
            pass

    ser.close()
    return device


def check_mac_format(mac_str: str) -> bool:
    """Verify MAC address format"""
    if not mac_str:
        return False
    parts = mac_str.split(":")
    if len(parts) != 6:
        return False
    for part in parts:
        if len(part) != 2 or not all(c in "0123456789ABCDEFabcdef" for c in part):
            return False
    return True


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        print("ERROR: Please provide both port names")
        print(f"Usage: python3 {sys.argv[0]} /dev/ttyACM0 /dev/ttyUSB0")
        sys.exit(1)

    ctrl_port = sys.argv[1]
    drone_port = sys.argv[2]

    print("=" * 70)
    print("ESP-NOW Communication Diagnostic")
    print("=" * 70)

    # Read controller startup info
    print(f"\n[STEP 1] Reading CONTROLLER ESP32 from {ctrl_port}")
    ctrl_info = extract_device_info(ctrl_port)

    if ctrl_info is None or ctrl_info.mac is None:
        print(f"  ERROR: Could not read MAC from {ctrl_port}")
        sys.exit(1)

    # Read drone startup info
    print(f"\n[STEP 2] Reading DRONE ESP32 from {drone_port}")
    drone_info = extract_device_info(drone_port)

    if drone_info is None or drone_info.mac is None:
        print(f"  ERROR: Could not read MAC from {drone_port}")
        sys.exit(1)

    # Diagnostics
    print("\n" + "=" * 70)
    print("DIAGNOSTIC RESULTS")
    print("=" * 70)

    print(f"\nController ESP32:")
    print(f"  Port: {ctrl_port}")
    print(f"  MAC:  {ctrl_info.mac}")
    print(
        f"  Target (drone): {ctrl_info.target_mac if ctrl_info.target_mac else '(not logged)'}"
    )

    print(f"\nDrone ESP32:")
    print(f"  Port: {drone_port}")
    print(f"  MAC:  {drone_info.mac}")
    print(
        f"  Target (controller): {drone_info.target_mac if drone_info.target_mac else '(not logged)'}"
    )

    # Check for issues
    print("\n" + "-" * 70)
    print("CHECKS")
    print("-" * 70)

    issues = []

    # Check 1: Controller configured with correct drone MAC
    if not ctrl_info.target_mac:
        issues.append(
            "⚠️  Cannot determine controller's configured drone MAC from logs.\n"
            "    Make sure the controller firmware has the correct drone_mac[]."
        )
    elif ctrl_info.target_mac != drone_info.mac:
        issues.append(
            f"❌ MISMATCH: Controller is configured for drone MAC {ctrl_info.target_mac}\n"
            f"    but drone's actual MAC is {drone_info.mac}\n"
            f"    → Update drone_mac[] in controller_main.c to {drone_info.mac}\n"
            f"    → Rebuild and reflash the controller"
        )
    else:
        print("✅ Controller's drone MAC is correct")

    # Check 2: Drone configured with correct controller MAC
    if not drone_info.target_mac:
        print("⚠️  Cannot determine drone's configured controller MAC from logs.")
        print("    The drone should print its controller_mac[] during startup.")
        print("    Make sure you set it correctly in drone_main.c")
    elif drone_info.target_mac != ctrl_info.mac:
        issues.append(
            f"❌ MISMATCH: Drone is configured for controller MAC {drone_info.target_mac}\n"
            f"    but controller's actual MAC is {ctrl_info.mac}\n"
            f"    → Update controller_mac[] in drone_main.c to {ctrl_info.mac}\n"
            f"    → Rebuild and reflash the drone"
        )
    else:
        print("✅ Drone's controller MAC is correct")

    # Check 3: MAC format validation
    if not check_mac_format(ctrl_info.mac):
        issues.append(f"❌ Controller MAC format invalid: {ctrl_info.mac}")
    if not check_mac_format(drone_info.mac):
        issues.append(f"❌ Drone MAC format invalid: {drone_info.mac}")

    if not issues:
        print("✅ All MAC addresses match correctly!")

    # Print summary
    print("\n" + "=" * 70)
    if issues:
        print("ISSUES FOUND:")
        print("=" * 70)
        for i, issue in enumerate(issues, 1):
            print(f"\n{i}. {issue}")
        print("\n" + "-" * 70)
        print("FIX INSTRUCTIONS:")
        print("-" * 70)
        print("\n1. Edit the firmware files:")
        if any("controller_main.c" in issue for issue in issues):
            print(f"   esp/controller/main/controller_main.c")
            print(f"   Change: static uint8_t drone_mac[6] = {{...}}")
            print(f"   To:     static uint8_t drone_mac[6] = {{")
            parts = drone_info.mac.split(":")
            print(
                f"              0x{parts[0]}, 0x{parts[1]}, 0x{parts[2]}, "
                f"0x{parts[3]}, 0x{parts[4]}, 0x{parts[5]}"
            )
            print(f"           }};")

        if any("drone_main.c" in issue for issue in issues):
            print(f"   esp/drone/main/drone_main.c")
            print(f"   Change: static uint8_t controller_mac[6] = {{...}}")
            print(f"   To:     static uint8_t controller_mac[6] = {{")
            parts = ctrl_info.mac.split(":")
            print(
                f"              0x{parts[0]}, 0x{parts[1]}, 0x{parts[2]}, "
                f"0x{parts[3]}, 0x{parts[4]}, 0x{parts[5]}"
            )
            print(f"           }};")

        print("\n2. Rebuild and flash both ESPs:")
        print("   cd esp/controller")
        print("   idf.py build flash -p /dev/ttyACM0")
        print("   cd ../drone")
        print("   idf.py build flash -p /dev/ttyUSB0")
        print("\n3. Run this diagnostic again to verify the fix")
        print("\n4. Then try: python3 controller_pc.py /dev/ttyACM0")
        sys.exit(1)
    else:
        print("SUCCESS!")
        print("=" * 70)
        print("\nAll checks passed! Your ESPs should communicate.")
        print("\nNext steps:")
        print("1. Power on the drone (battery + ESP32)")
        print("2. Plug the controller ESP32 into your PC")
        print("3. Run the controller script:")
        print("   python3 controller_pc.py /dev/ttyACM0")
        sys.exit(0)


if __name__ == "__main__":
    main()
