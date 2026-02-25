# ESP32 Drone Project

Two-ESP32 drone system: one ESP32 on the drone (flight controller), one with the
pilot connected to a PC (ground controller). Communication via ESP-NOW wireless.

```
PC (keyboard) ──USB/UART──▶ Controller ESP32 ──ESP-NOW──▶ Drone ESP32 ──PWM──▶ Motors
                                              ◀──ESP-NOW──           ◀──I2C── IMU
                                                        (telemetry)  ◀──ADC── Battery
```

---

## Folder Structure

```
esp/
├── controller/             # ESP-IDF project — controller ESP32 (with pilot)
│   ├── CMakeLists.txt
│   └── main/
│       ├── CMakeLists.txt
│       └── controller_main.c
├── drone/                  # ESP-IDF project — drone ESP32 (on the aircraft)
│   ├── CMakeLists.txt
│   └── main/
│       ├── CMakeLists.txt
│       └── drone_main.c
├── pc/                     # PC-side software (Python)
│   ├── controller_pc.py    # Keyboard controller + telemetry display
│   └── requirements.txt
└── README.md               # You are here
```

---

## Hardware Required

### Controller Side (Ground)
- 1× ESP32 DevKit (any variant with USB)
- 1× USB cable to PC

### Drone Side (Aircraft)
- 1× ESP32 DevKit (or ESP32 module for weight savings)
- 1× MPU6050 IMU module (GY-521 breakout)
- 4× Brushless ESCs (or 4-in-1 ESC)
- 4× Brushless motors
- 4× Propellers (2× CW, 2× CCW)
- 1× LiPo battery (3S or 4S depending on motors)
- 1× Voltage divider for battery monitoring (2× 10kΩ resistors)
- 1× Drone frame (X-configuration)
- Wires, solder, connectors

---

## Wiring — Drone ESP32

### Motor ESCs (PWM output)

| Motor | Position        | Rotation | GPIO | LEDC Channel |
|-------|-----------------|----------|------|--------------|
| M1    | Front-Right     | CW       | 25   | 0            |
| M2    | Back-Right      | CCW      | 26   | 1            |
| M3    | Front-Left      | CCW      | 27   | 2            |
| M4    | Back-Left       | CW       | 14   | 3            |

Connect each ESC signal wire to the corresponding GPIO.  
Connect each ESC ground to ESP32 GND.  
**Do NOT connect ESC 5V to ESP32** (power the ESP32 from its own regulator or USB).

### MPU6050 IMU (I2C)

| MPU6050 Pin | ESP32 GPIO | Notes                        |
|-------------|------------|------------------------------|
| VCC         | 3.3V       | **NOT 5V** (some boards have a regulator, check yours) |
| GND         | GND        |                              |
| SDA         | GPIO 21    |                              |
| SCL         | GPIO 22    |                              |
| AD0         | GND        | Sets I2C address to 0x68     |
| INT         | (unused)   | We poll instead of interrupt |

### Battery Voltage Monitor (ADC)

```
Battery (+) ──┬── 10kΩ ──┬── 10kΩ ──┬── Battery (-)
              |          |          |
              |       GPIO 34      GND
              |     (ADC1_CH6)
              |
         To ESC power
```

This voltage divider halves the voltage so a 2S LiPo (8.4V max) reads as ~4.2V,
within the ESP32 ADC range. Adjust `BATT_DIVIDER_RATIO` in the firmware if you
use different resistor values.

### Motor Layout (top view)

```
           FRONT
    M3 (FL,CCW)    M1 (FR,CW)
         \\          //
          \\        //
           ╔══════╗
           ║ ESP32║
           ╚══════╝
          //        \\
         //          \\
    M4 (BL,CW)    M2 (BR,CCW)
           BACK
```

---

## Prerequisites

### ESP-IDF (v5.5.x)

Make sure ESP-IDF is installed and sourced:

```bash
# If installed at /opt/esp-idf:
source /opt/esp-idf/export.sh

# Verify:
idf.py --version
# Should show: ESP-IDF v5.5.x
```

### Python (for PC controller)

```bash
cd esp/pc
pip install -r requirements.txt
```

---

## Build, Flash & Run — Step by Step

### Step 1: Flash the Drone ESP32

Plug the **drone** ESP32 into your PC via USB.

```bash
cd esp/drone

# Set the target chip (only needed once)
idf.py set-target esp32

# Build
idf.py build

# Flash (change port if needed)
idf.py -p /dev/ttyUSB0 flash

# Monitor — watch for the MAC address!
idf.py -p /dev/ttyUSB0 monitor
```

**Look for this line in the output:**

```
║  Drone MAC: B0:CB:D8:CD:85:68              ║
║  Put this in controller's drone_mac[]      ║
```

**Write down the MAC address!** Press `Ctrl+]` to exit the monitor.

### Step 2: Update the Controller with the Drone's MAC

Open `esp/controller/main/controller_main.c` and replace the `drone_mac` array:

```c
// Replace the FF:FF:FF:FF:FF:FF with your drone's actual MAC
static uint8_t drone_mac[6] = {0xB0, 0xCB, 0xD8, 0xCD, 0x85, 0x68};
```

### Step 3: Flash the Controller ESP32

Unplug the drone ESP32. Plug in the **controller** ESP32.

```bash
cd esp/controller

# Set target (only needed once)
idf.py set-target esp32

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor to see its MAC address
idf.py -p /dev/ttyUSB0 monitor
```

**Look for the Controller MAC address.** Write it down. Press `Ctrl+]` to exit.

### Step 4: (Optional) Update the Drone with the Controller's MAC

Open `esp/drone/main/drone_main.c` and set `controller_mac`:

```c
static uint8_t controller_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

> **Note:** If you leave it as `FF:FF:FF:FF:FF:FF`, the drone will send
> telemetry to the broadcast address, which works but is less reliable.
> Setting the actual MAC is recommended.

Re-flash the drone if you changed this:

```bash
cd esp/drone
idf.py -p /dev/ttyUSB0 build flash
```

### Step 5: Power Up & Fly

1. **Power the drone** (battery + ESP32). Keep it on a flat surface for gyro calibration (2 seconds).
2. **Plug the controller ESP32** into the PC via USB.
3. **Wait 3–4 seconds** for the controller's startup logs to finish.
4. **Close `idf.py monitor`** if it's running — it will conflict with the Python script.
5. **Run the PC controller:**

```bash
cd esp/pc
python3 controller_pc.py /dev/ttyUSB0
```

6. **Controls:**

| Key       | Action                          |
|-----------|---------------------------------|
| `SPACE`   | Arm / Disarm toggle             |
| `W` / `S` | Throttle up / down (±20)        |
| `I` / `K` | Pitch forward / back (±50)      |
| `J` / `L` | Roll left / right (±50)         |
| `A` / `D` | Yaw left / right (±50)          |
| `C`       | Center all sticks (roll/pitch/yaw = 0) |
| `R`       | Reset throttle to minimum       |
| `Q`/`ESC` | Quit (auto-disarms)             |

---

## Troubleshooting

### "Permission denied" on serial port

```bash
sudo chmod 666 /dev/ttyUSB0
# or add yourself to the dialout group permanently:
sudo usermod -aG dialout $USER
# then log out and back in
```

### "ESP-NOW send failed"

- Make sure both ESP32s are powered on
- Verify the MAC addresses are correct in both firmwares
- Both must be on the same Wi-Fi channel (default: channel 1)
- Both must have the same `USE_LONG_RANGE` setting (default: enabled)
- Try reducing distance for testing (start within 1 meter)

### "MPU6050 not found"

- Check wiring: SDA→GPIO21, SCL→GPIO22, VCC→3.3V, GND→GND
- Make sure AD0 pin is connected to GND (address 0x68)
- Try adding 4.7kΩ pull-up resistors on SDA and SCL to 3.3V
- The drone will still respond to throttle without the IMU, but won't stabilize

### Python script shows no telemetry

- The controller suppresses logs after 3 seconds — the script waits for this
- Make sure `idf.py monitor` is NOT running (it locks the serial port)
- Check that telemetry is being sent: run `idf.py monitor` on the drone ESP32 and look for telemetry log lines

### Motors don't spin

- ESCs need to see minimum throttle (1000 µs) at power-on to arm
- Power the drone ESP32 FIRST, wait for startup, THEN connect the battery to the ESCs
- Check that ESC signal wires go to the correct GPIOs (25, 26, 27, 14)
- Verify ESC ground is connected to ESP32 ground

### Drone flips on takeoff

- Motor rotation directions are wrong — swap any two motor wires to reverse direction
- Motors are in wrong positions — verify the X-configuration layout
- PID gains are too aggressive — reduce `KP` values in `drone_main.c`
- IMU orientation doesn't match expected axes — check MPU6050 mounting direction

---

## PID Tuning Guide

The default PID gains are conservative starting values. You **will** need to tune them.

1. **Start with only P gain.** Set `KI` and `KD` to 0.
2. **Increase `KP` slowly** until the drone starts to oscillate, then back off ~30%.
3. **Add `KD`** to dampen oscillations. Increase until you see high-frequency vibration, then reduce.
4. **Add a small `KI`** to eliminate steady-state error (drift). Keep it small to avoid windup.

Tune roll and pitch together (same gains) since the drone should be symmetric.
Tune yaw separately.

**Safety:** Always tune with the drone **tethered** or in a test rig where it can't fly away!

---

## Safety Warnings

> ⚠️ **SPINNING PROPELLERS ARE DANGEROUS.** They can cause serious injury.

- **REMOVE PROPELLERS** while testing firmware, PID tuning on the bench, or debugging.
- **Never arm the drone while holding it.**
- **Always have a clear disarm procedure** (press Q or ESC on the PC controller).
- **Test in a large open area** away from people, animals, and obstacles.
- **Start with very low throttle** and work up gradually.
- **Failsafe is your friend:** if the drone loses signal for 500 ms, it auto-disarms. Do not disable this.
- **LiPo batteries are a fire hazard.** Never over-discharge, puncture, or charge unattended.
- **Follow your local drone regulations.**

---

## Architecture Notes

### Communication Protocol

Both directions use an 11-byte packed binary protocol:

**Control Packet (PC → Controller → Drone):**
```
Byte  0:    Magic (0xAB)
Bytes 1-2:  Throttle (uint16, little-endian, 1000–2000)
Bytes 3-4:  Roll     (int16, -500 to +500)
Bytes 5-6:  Pitch    (int16, -500 to +500)
Bytes 7-8:  Yaw      (int16, -500 to +500)
Byte  9:    Armed    (0 or 1)
Byte  10:   Checksum (XOR of bytes 0–9)
```

**Telemetry Packet (Drone → Controller → PC):**
```
Byte  0:    Magic (0xCD)
Bytes 1-2:  Battery mV  (uint16, little-endian)
Bytes 3-4:  Roll × 10   (int16, degrees)
Bytes 5-6:  Pitch × 10  (int16, degrees)
Bytes 7-8:  Yaw × 10    (int16, degrees)
Byte  9:    Flags        (bit0=armed, bit1=failsafe)
Byte  10:   Checksum     (XOR of bytes 0–9)
```

### FreeRTOS Tasks (Drone)

| Task      | Priority | Frequency | Purpose                        |
|-----------|----------|-----------|--------------------------------|
| IMU       | 6        | 250 Hz    | Read MPU6050, complementary filter |
| Flight    | 5        | 250 Hz    | PID controller, motor mixing   |
| Telemetry | 3        | 5 Hz      | Battery ADC, send telemetry    |

### FreeRTOS Tasks (Controller)

| Task       | Priority | Purpose                              |
|------------|----------|--------------------------------------|
| UART RX    | 5        | Read packets from PC, validate, queue |
| ESP-NOW TX | 5        | Send queued packets to drone          |

ESP-NOW receive callback handles incoming telemetry and forwards it to UART.

---

## License

This project is provided as-is for educational purposes. Use at your own risk.