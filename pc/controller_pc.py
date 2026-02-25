#!/usr/bin/env python3
"""
ESP32 Drone — PC Keyboard Controller
=====================================
Sends control packets to the Controller ESP32 over USB/UART.
Receives and displays telemetry coming back from the drone.

Usage:
    python3 controller_pc.py [PORT] [BAUD]

Examples:
    python3 controller_pc.py                     # defaults: /dev/ttyUSB0 @ 115200
    python3 controller_pc.py /dev/ttyUSB1
    python3 controller_pc.py /dev/ttyACM0 115200
    python3 controller_pc.py COM3 115200          # Windows

Keys:
    W / S       Throttle up / down
    A / D       Yaw left (CCW) / right (CW)
    I / K       Pitch forward / back
    J / L       Roll left / right
    SPACE       Arm / Disarm toggle
    C           Center roll, pitch, yaw (sticks to neutral)
    R           Reset throttle to minimum
    Q / ESC     Quit (auto-disarms)
"""

import os
import signal
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
#  Protocol constants — MUST match the ESP32 firmware
# ---------------------------------------------------------------------------

MAGIC_CTRL = 0xAB
MAGIC_TELE = 0xCD

# control_packet_t layout:  magic(1) + throttle(2) + roll(2) + pitch(2) + yaw(2) + armed(1) + checksum(1) = 11
CTRL_PACK_FMT = "<BHhhhBB"
CTRL_PACK_SIZE = struct.calcsize(CTRL_PACK_FMT)  # 11

# telemetry_packet_t layout: magic(1) + voltage_mv(2) + roll_deg(2) + pitch_deg(2) + yaw_deg(2) + flags(1) + checksum(1) = 11
TELE_PACK_FMT = "<BHhhhBB"
TELE_PACK_SIZE = struct.calcsize(TELE_PACK_FMT)  # 11

# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------


def xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs


def build_ctrl_packet(
    throttle: int, roll: int, pitch: int, yaw: int, armed: int
) -> bytes:
    """Build a control_packet_t byte string with checksum."""
    # Pack everything except checksum
    payload = struct.pack(
        "<BHhhhB",
        MAGIC_CTRL,
        int(throttle),
        int(roll),
        int(pitch),
        int(yaw),
        int(armed),
    )
    cs = xor_checksum(payload)
    return payload + bytes([cs])


# ---------------------------------------------------------------------------
#  Cross-platform non-blocking key reading
# ---------------------------------------------------------------------------

if os.name == "nt":
    # Windows
    import msvcrt

    def get_key() -> str:
        """Block until a key is pressed, return it as a string."""
        while True:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    return ch.decode("utf-8", errors="replace")
                except Exception:
                    return ""
            time.sleep(0.01)

    def setup_terminal():
        pass

    def restore_terminal():
        pass

else:
    # POSIX (Linux / macOS)
    import select
    import termios
    import tty

    _old_settings = None

    def setup_terminal():
        global _old_settings
        fd = sys.stdin.fileno()
        _old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

    def restore_terminal():
        global _old_settings
        if _old_settings is not None:
            fd = sys.stdin.fileno()
            termios.tcsetattr(fd, termios.TCSADRAIN, _old_settings)
            _old_settings = None

    def get_key() -> str:
        """Block until a key is pressed (with 50 ms poll), return it."""
        fd = sys.stdin.fileno()
        while True:
            rlist, _, _ = select.select([fd], [], [], 0.05)
            if rlist:
                return sys.stdin.read(1)


# ---------------------------------------------------------------------------
#  Telemetry receiver thread
# ---------------------------------------------------------------------------


class TelemetryReceiver:
    """Reads telemetry packets from the serial port in a background thread."""

    def __init__(self, ser):
        self.ser = ser
        self.voltage_mv = 0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.flags = 0
        self.last_update = 0.0
        self.packet_count = 0
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                data = self.ser.read(64)
                if not data:
                    continue
                buf.extend(data)

                # Scan for telemetry packets in the buffer
                while len(buf) >= TELE_PACK_SIZE:
                    # Look for magic byte
                    idx = buf.find(MAGIC_TELE)
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        # Discard bytes before magic
                        del buf[:idx]
                    if len(buf) < TELE_PACK_SIZE:
                        break

                    candidate = bytes(buf[:TELE_PACK_SIZE])
                    expected_cs = xor_checksum(candidate[:-1])
                    if candidate[-1] == expected_cs:
                        # Valid packet
                        magic, vmv, r, p, y, flags, cs = struct.unpack(
                            TELE_PACK_FMT, candidate
                        )
                        with self.lock:
                            self.voltage_mv = vmv
                            self.roll_deg = r / 10.0
                            self.pitch_deg = p / 10.0
                            self.yaw_deg = y / 10.0
                            self.flags = flags
                            self.last_update = time.time()
                            self.packet_count += 1
                        del buf[:TELE_PACK_SIZE]
                    else:
                        # Bad checksum — skip the magic byte and keep scanning
                        del buf[:1]

            except Exception:
                if self._stop.is_set():
                    break
                time.sleep(0.01)

    def get(self):
        """Return a snapshot of the latest telemetry data."""
        with self.lock:
            return {
                "voltage_mv": self.voltage_mv,
                "roll": self.roll_deg,
                "pitch": self.pitch_deg,
                "yaw": self.yaw_deg,
                "flags": self.flags,
                "age": time.time() - self.last_update if self.last_update else None,
                "count": self.packet_count,
            }


# ---------------------------------------------------------------------------
#  Display
# ---------------------------------------------------------------------------


def clear_screen():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def print_status(throttle, roll, pitch, yaw, armed, tele):
    """Redraw the status display (moves cursor to top)."""
    sys.stdout.write("\033[H")  # cursor to top-left

    armed_str = "\033[1;31m[ARMED]\033[0m" if armed else "\033[1;32m[SAFE] \033[0m"

    lines = [
        "\033[1;36m╔══════════════════════════════════════════════╗\033[0m",
        "\033[1;36m║\033[0m   ESP32 Drone — Keyboard Controller          \033[1;36m║\033[0m",
        "\033[1;36m╠══════════════════════════════════════════════╣\033[0m",
        f"\033[1;36m║\033[0m  Status: {armed_str}                          \033[1;36m║\033[0m",
        "\033[1;36m╠══════════════════════════════════════════════╣\033[0m",
        f"\033[1;36m║\033[0m  Throttle : {throttle:4d}  (W/S ±20)              \033[1;36m║\033[0m",
        f"\033[1;36m║\033[0m  Roll     : {roll:+5d}  (J/L ±50)              \033[1;36m║\033[0m",
        f"\033[1;36m║\033[0m  Pitch    : {pitch:+5d}  (I/K ±50)              \033[1;36m║\033[0m",
        f"\033[1;36m║\033[0m  Yaw      : {yaw:+5d}  (A/D ±50)              \033[1;36m║\033[0m",
        "\033[1;36m╠══════════════════════════════════════════════╣\033[0m",
    ]

    # Telemetry section
    if tele["count"] > 0 and tele["age"] is not None and tele["age"] < 2.0:
        voltage = tele["voltage_mv"] / 1000.0
        drone_armed = "YES" if (tele["flags"] & 0x01) else "NO"
        failsafe = "YES" if (tele["flags"] & 0x02) else "no"
        lines += [
            f"\033[1;36m║\033[0m  Battery  : {voltage:5.2f} V                       \033[1;36m║\033[0m",
            f"\033[1;36m║\033[0m  IMU Roll : {tele['roll']:+6.1f}°                     \033[1;36m║\033[0m",
            f"\033[1;36m║\033[0m  IMU Pitch: {tele['pitch']:+6.1f}°                     \033[1;36m║\033[0m",
            f"\033[1;36m║\033[0m  IMU Yaw  : {tele['yaw']:+6.1f}°                     \033[1;36m║\033[0m",
            f"\033[1;36m║\033[0m  D-Armed  : {drone_armed:3s}  Failsafe: {failsafe:3s}        \033[1;36m║\033[0m",
            f"\033[1;36m║\033[0m  Tele pkts: {tele['count']}                            \033[1;36m║\033[0m",
        ]
    else:
        lines += [
            "\033[1;36m║\033[0m  Telemetry: \033[33mwaiting for drone...\033[0m         \033[1;36m║\033[0m",
        ]

    lines += [
        "\033[1;36m╠══════════════════════════════════════════════╣\033[0m",
        "\033[1;36m║\033[0m  SPACE=arm  C=center  R=reset thr  Q=quit    \033[1;36m║\033[0m",
        "\033[1;36m╚══════════════════════════════════════════════╝\033[0m",
        "",  # blank line to clear any leftover text
    ]

    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------


def main():
    # Parse CLI arguments
    port = "/dev/ttyUSB0"
    baud = 115200

    args = sys.argv[1:]
    if len(args) >= 1:
        port = args[0]
    if len(args) >= 2:
        try:
            baud = int(args[1])
        except ValueError:
            print(f"Invalid baud rate: {args[1]}")
            sys.exit(1)

    if "--help" in sys.argv or "-h" in sys.argv:
        print(__doc__)
        sys.exit(0)

    # Import serial here so we get a clear error if pyserial is missing
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed.")
        print("  Install it with:  pip install pyserial")
        sys.exit(1)

    # Connect
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        print(f"Connected to {port} @ {baud} baud")
    except Exception as e:
        print(f"ERROR: cannot open {port}: {e}")
        print("  - Is the ESP32 plugged in?")
        print("  - Do you have permission? Try:  sudo chmod 666 " + port)
        print("  - Is another program using the port (e.g. idf.py monitor)?")
        sys.exit(1)

    # Wait a moment for the ESP32 to finish its startup logging
    print("Waiting for ESP32 startup logs to finish...")
    time.sleep(3.5)
    ser.reset_input_buffer()

    # State
    throttle = 1000
    roll = 0
    pitch = 0
    yaw = 0
    armed = 0

    # Start telemetry receiver
    tele_rx = TelemetryReceiver(ser)
    tele_rx.start()

    # Setup terminal for raw key input
    setup_terminal()

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, signal_handler)

    try:
        clear_screen()
        print_status(throttle, roll, pitch, yaw, armed, tele_rx.get())

        while True:
            k = get_key()
            if k is None:
                continue
            k = k.lower()

            changed = True

            if k == "q" or k == "\x1b":  # q or ESC
                # Disarm before quitting
                armed = 0
                pkt = build_ctrl_packet(1000, 0, 0, 0, 0)
                ser.write(pkt)
                break

            elif k == " ":
                # Safety: only allow arming if throttle is at minimum
                if not armed and throttle > 1050:
                    # Refuse to arm — throttle too high
                    pass  # could beep or show warning
                else:
                    armed ^= 1

            elif k == "w":
                throttle = min(2000, throttle + 20)
            elif k == "s":
                throttle = max(1000, throttle - 20)

            elif k == "a":
                yaw = max(-500, yaw - 50)
            elif k == "d":
                yaw = min(500, yaw + 50)

            elif k == "i":
                pitch = min(500, pitch + 50)
            elif k == "k":
                pitch = max(-500, pitch - 50)

            elif k == "j":
                roll = max(-500, roll - 50)
            elif k == "l":
                roll = min(500, roll + 50)

            elif k == "c":
                roll = pitch = yaw = 0

            elif k == "r":
                throttle = 1000

            else:
                changed = False

            if changed:
                pkt = build_ctrl_packet(throttle, roll, pitch, yaw, armed)
                ser.write(pkt)

            print_status(throttle, roll, pitch, yaw, armed, tele_rx.get())

    except KeyboardInterrupt:
        # Disarm on Ctrl+C
        armed = 0
        pkt = build_ctrl_packet(1000, 0, 0, 0, 0)
        try:
            ser.write(pkt)
        except Exception:
            pass

    finally:
        restore_terminal()
        tele_rx.stop()
        ser.close()

        # Move below the UI box
        sys.stdout.write("\n\n")
        print("Disarmed and disconnected. Goodbye!")


if __name__ == "__main__":
    main()
