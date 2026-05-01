# gpio_rosbag_controller

Lightweight ROS2 C++ node for Raspberry Pi that controls `ros2 bag record`
recording lifecycle using a **physical switch** and an **LED** for visual
feedback. Designed for standalone field use — no laptop required.

---

## Features

- Physical switch controls recording start/stop
- LED provides clear visual feedback (OFF / SOLID / BLINK)
- `ros2 bag record` spawned as a child process (robust, isolated)
- Graceful stop via SIGINT; SIGKILL only as last resort
- Automatic recovery if rosbag crashes unexpectedly
- Debounced GPIO input (no false triggers)
- All parameters configurable via YAML — no recompilation needed
- systemd service for automatic boot start

---

## LED States

| LED state | Meaning                        |
|-----------|-------------------------------|
| OFF       | Idle — not recording           |
| SOLID ON  | Transition (starting/stopping) |
| BLINKING  | Actively recording             |

---

## GPIO Wiring

### Requirements

| Signal | Direction | BCM GPIO | Physical Pin | Notes                    |
|--------|-----------|----------|--------------|--------------------------|
| Switch | Input     | BCM 17   | Pin 11       | External 10kΩ pull-down  |
| LED    | Output    | BCM 27   | Pin 13       | 330Ω series resistor     |

> BCM numbers are configurable via `params.yaml` — see `switch_gpio` and
> `led_gpio` parameters.

---

### Switch Wiring (BCM 17 — Pin 11)

```
3.3V (Pin 1) ─────────────────── one side of switch
                                        │
                              (switch closes here)
                                        │
BCM 17 (Pin 11) ────────────────── other side of switch
     │
    10kΩ pull-down resistor
     │
   GND (Pin 6)
```

**Why a pull-down?**
When the switch is open, BCM 17 would float to an undefined voltage without a
pull-down resistor, causing false trigger events. The 10kΩ pull-down holds the
line at logic LOW (0) when the switch is open, and the 3.3V rail pulls it HIGH
(1) when the switch closes.

**Can I use the internal pull-down instead?**
Yes. The Raspberry Pi has internal ~50kΩ pull-down resistors. You can use them
by modifying the GpioInterface code to use
`gpiod_line_request_input_flags(line_, consumer, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN)`.
However, an external 10kΩ is more reliable in noisy field environments.

**ASCII wiring diagram:**

```
  3.3V ──┬── [ SWITCH ] ──── BCM 17 (Pin 11)
         │                       │
         │                      10kΩ
         │                       │
        GND ─────────────────── GND (Pin 6)
```

---

### LED Wiring (BCM 27 — Pin 13)

```
BCM 27 (Pin 13) ── 330Ω ── [LED anode (+)] ── [LED cathode (-)] ── GND (Pin 9)
```

**Why 330Ω?**
The Raspberry Pi GPIO outputs ~3.3V at a maximum of ~16mA per pin.
A standard 5mm red LED has a forward voltage of ~2.0V.

```
R = (V_supply - V_f) / I_forward = (3.3 - 2.0) / 0.010 = 130Ω (minimum)
```

330Ω gives ~4mA — clearly visible and safe. Do not omit the resistor or you
risk permanently damaging the GPIO pin.

**ASCII wiring diagram:**

```
  BCM 27 (Pin 13) ──[ 330Ω ]──┤► GND (Pin 9)
                              LED
                           (flat side = cathode = GND)
```

---

### Raspberry Pi 40-pin header reference (relevant pins)

```
         3.3V │  1 │  2 │ 5V
     (GPIO 2) │  3 │  4 │ 5V
     (GPIO 3) │  5 │  6 │ GND  ◄─ GND for LED cathode
     (GPIO 4) │  7 │  8 │ GPIO 14
          GND │  9 │ 10 │ GPIO 15
  GPIO 17 ◄── │ 11 │ 12 │ GPIO 18   ◄─ SWITCH signal
     GPIO 27 ◄│ 13 │ 14 │ GND       ◄─ LED signal
     ...
```

---

### Electrical safety guidelines

1. **Never exceed 3.3V on GPIO pins.** The RPi GPIO is NOT 5V tolerant.
2. **Never draw more than 16mA from a single GPIO pin.**
3. **Always use a resistor in series with an LED.**
4. **Avoid connecting inductive loads directly** (motors, relays) without
   protection diodes.
5. **Disconnect power before wiring.**
6. **Use short wires in field conditions** to reduce noise pickup.

---

## Software Architecture

### Why `ros2 bag record` subprocess instead of `rosbag2_cpp` API?

| Criterion               | subprocess (`ros2 bag record`) | `rosbag2_cpp` API        |
|------------------------|-------------------------------|--------------------------|
| Isolation              | **✓ separate process**        | same process             |
| Crash protection       | **✓ controller survives**     | crash kills controller   |
| Simplicity             | **✓ no extra deps**           | complex API              |
| SIGINT shutdown        | **✓ clean bag close**         | needs manual flush       |
| Debugging              | **✓ separate logs/PID**       | mixed with node logs     |
| ROS spinning blocked?  | ✓ No — runs in child process  | depends on implementation|

**Conclusion:** subprocess is the correct choice for this field-recording use
case. The controller node is completely decoupled from rosbag failures.

---

### State Machine

```
         switch=1
  IDLE ──────────────► STARTING
   ▲                      │ bag alive (watchdog)
   │                      ▼
   │  bag crash   RECORDING
   │◄────────────────     │ switch=0
   │                      ▼
   │◄──────────────── STOPPING
      bag exited
```

---

## Build

```bash
# Install system dependency
sudo apt install libgpiod-dev

# Build
cd ~/nav_mapper
colcon build --packages-select gpio_rosbag_controller
source install/setup.bash
```

---

## Run

```bash
ros2 launch gpio_rosbag_controller gpio_rosbag_controller.launch.py
```

Override parameters on the fly:

```bash
ros2 run gpio_rosbag_controller gpio_rosbag_controller_node \
  --ros-args -p switch_gpio:=17 -p led_gpio:=27 -p bag_output_dir:=/mnt/usb/bags
```

---

## systemd auto-start

```bash
# 1. Copy the service file
sudo cp scripts/gpio_rosbag_controller.service /etc/systemd/system/

# 2. Edit it to match your username and workspace path
sudo nano /etc/systemd/system/gpio_rosbag_controller.service

# 3. Enable and start
sudo systemctl daemon-reload
sudo systemctl enable gpio_rosbag_controller.service
sudo systemctl start gpio_rosbag_controller.service

# 4. Check status and logs
sudo systemctl status gpio_rosbag_controller.service
journalctl -u gpio_rosbag_ctrl -f
```

---

## Bags

Bags are stored in `~/bags/` (configurable) with names like:

```
field_20260501_143022/
```

---

## Parameters reference

| Parameter        | Default      | Description                              |
|-----------------|--------------|------------------------------------------|
| `gpio_chip`     | `gpiochip0`  | GPIO chip device name                    |
| `switch_gpio`   | `17`         | BCM GPIO number for switch input         |
| `led_gpio`      | `27`         | BCM GPIO number for LED output           |
| `bag_output_dir`| `~/bags`     | Directory to store rosbags               |
| `poll_period_ms`| `50`         | Switch poll interval (ms)                |
| `debounce_count`| `5`          | Samples needed to confirm state change   |
| `blink_period_ms`| `500`       | LED blink half-period (ms)               |
| `stop_timeout_s`| `10`         | Seconds before SIGKILL if still running  |
| `topics`        | (see yaml)   | List of topics to record                 |
