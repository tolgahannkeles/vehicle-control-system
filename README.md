# vehicle-control-system — ESP32 Tracked Motor Controller

Embedded firmware running on the **ESP32 side** of a tracked (skid-steer) tactical ground vehicle. Built on ESP-IDF; it drives the motor controller via PWM, talks to a Raspberry Pi 5 over UART using a binary protocol, and exposes a standalone Wi-Fi web interface for joystick control and over-the-air (OTA) firmware updates.

This repository is the **counterpart** to the `tracked_hardware/esp32_bridge` node in the main ROS 2 workstation repository: the Raspberry Pi side converts `/cmd_vel` into a UART frame and sends it here; this firmware decodes the frame, drives the motors, and sends IMU/GPS telemetry back.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Components (`components/`)](#components-components)
- [UART Protocol](#uart-protocol)
- [Motor Driver Layer](#motor-driver-layer)
- [Wi-Fi Web Control Interface](#wi-fi-web-control-interface)
- [OTA (Over-the-Air Update)](#ota-over-the-air-update)
- [Hardware Pin Map](#hardware-pin-map)
- [Partition Table](#partition-table)
- [Setup and Build](#setup-and-build)
- [Known Gaps and Risks](#known-gaps-and-risks)

## Architecture Overview

The firmware merges two independent control paths into the same motor driver layer:

```
┌───────────────────────┐          UART2 (115200, 8N1)         ┌──────────────────────────┐
│  Raspberry Pi 5 / ROS2 │ ◄──────────────────────────────────► │  ESP32 (this repo)        │
│  tracked_hardware      │   AA 55 | ID | LEN | PAYLOAD | CRC   │  serial_bridge + protocol  │
└───────────────────────┘                                       └────────────┬─────────────┘
                                                                              │ PKT_ID_CMD_VEL
                                                                              ▼
┌───────────────────────┐        WebSocket /ws (not JSON,       ┌──────────────────────────┐
│  Browser (joystick     │ ◄──────────────────────────────────► │  web_ui (esp_http_server) │
│  on the Wi-Fi network) │        "token;throttle,steer")        │                            │
└───────────────────────┘                                       └────────────┬─────────────┘
                                                                              │ motor_update()
                                                                              ▼
                                                                   ┌──────────────────────────┐
                                                                   │      motor_driver          │
                                                                   │  (LEDC PWM + smoothing)    │
                                                                   └──────────────────────────┘
```

On boot, `main.c` initializes, in order: `motor_init()` → `serial_bridge_init()` → `telemetry_init()` → `wifi_init_sta()` → `start_webserver()`. `PKT_ID_CMD_VEL` packets from UART and joystick commands from the web interface both feed the **same** `motor_driver` target variables (`s_target_left` / `s_target_right`); there is **no arbitration** between them (no `twist_mux`-style lock) — whichever source writes last wins.

## Components (`components/`)

| Component | Responsibility |
|---|---|
| [`protocol`](components/protocol) | Defines the UART frame format: a state-machine-based byte-by-byte parser, XOR checksum computation, and the `cmd_vel_payload_t` / `imu_payload_t` / `gps_payload_t` structs. Hardware-independent. |
| [`serial_bridge`](components/serial_bridge) | Wires `protocol` to the UART2 peripheral. Runs separate RX/TX FreeRTOS tasks (Core 0), delivers decoded packets to the upper layer via callback, and queues outgoing packets on a FreeRTOS queue. |
| [`motor_driver`](components/motor_driver) | Manages PWM (LEDC) drive and direction GPIOs for both tracks, plus a deadband and exponential smoothing filter (50 Hz task, Core 1). Accepts both `motor_set_targets()` (m/s, ROS side) and `motor_update()` (-1000..1000, web joystick side) inputs. |
| [`telemetry`](components/telemetry) | Generates placeholder (dummy) IMU data at 50 Hz and a dummy GPS coordinate at 1 Hz, sent to the Raspberry Pi via `serial_bridge`. **Not yet wired to a real IMU/GPS sensor** — see [Known Gaps](#known-gaps-and-risks). |
| [`wifi_ap`](components/wifi_ap) | Starts Wi-Fi in **station (STA) mode** and connects to a hardcoded home/office network (despite the name `wifi_ap`, it sets up STA, not AP, mode). Retries up to 10 times on disconnect. |
| [`web_ui`](components/web_ui) | Serves `/` (static `index.html`, embedded in flash), `/ws` (WebSocket joystick control), and `/update` (OTA upload) via `esp_http_server`. Enforces a simple token-based "single-user lock" (session lock). |
| [`ota_manager`](components/ota_manager) | A thin wrapper over `esp_ota_ops`: writes the raw firmware binary POSTed to `/update` into the inactive OTA partition, verifies it, and activates it via `esp_restart()`. |

## UART Protocol

The link between the Raspberry Pi and the ESP32 uses a simple, checksummed frame defined in `protocol.h`:

```
Byte:     0     1     2      3       4 .. 4+LEN-1      4+LEN
        [0xAA][0x55][PKT_ID][LEN]  [PAYLOAD (LEN bytes)] [CRC]
```

- **Header**: fixed `0xAA 0x55` — the parser resyncs by scanning for these two bytes.
- **CRC**: `id XOR len XOR payload[0] XOR payload[1] XOR ...` (a plain XOR checksum, not a real CRC-8/16).
- **Parser**: `protocol_parse_byte()` is a byte-by-byte state machine (`SM_WAIT_H1` → `SM_WAIT_H2` → `SM_WAIT_ID` → `SM_WAIT_LEN` → `SM_WAIT_PAYLOAD` → `SM_WAIT_CRC`) that automatically falls back to `SM_WAIT_H1` on UART noise or an interrupted frame.

Defined packet IDs (`PKT_ID_*`):

| ID | Direction | Payload struct | Content |
|---|---|---|---|
| `0x01` `CMD_VEL` | RPi → ESP32 | `cmd_vel_payload_t { float v_left, v_right; }` | Independent left/right track speed (m/s) |
| `0x10` `IMU` | ESP32 → RPi | `imu_payload_t { float ax,ay,az,gx,gy,gz; }` | Acceleration (m/s²) and angular rate (rad/s), 50 Hz |
| `0x11` `GPS` | ESP32 → RPi | `gps_payload_t { double lat,lon; uint8_t fix; }` | Latitude/longitude and fix status, 1 Hz |

Payload structs are packed with `#pragma pack(push, 1)`; both ends of the link (RPi C++ and ESP32 C) rely on the same memory layout — there is **no endianness conversion**, so both sides must be little-endian architectures (ARM/Xtensa).

## Motor Driver Layer

- **Hardware**: two brushed DC motor drivers (H-bridge), each with `ENA/ENB` (PWM) + `IN1..IN4` (direction) pins.
- **PWM**: `LEDC`, 5 kHz, 10-bit resolution (0-1023 duty).
- **Deadband**: commands with `|speed| < 25` (out of 1000) are rounded to zero — prevents noisy jitter in the range where the motor doesn't actually turn.
- **Smoothing**: a dedicated FreeRTOS task running at 50 Hz eases the target speed toward the current PWM value with an exponential filter (`alpha = 0.08`), preventing sudden joystick/command jumps from slamming the motors.
- **Two input paths, one output**:
  - `motor_set_targets(left_mps, right_mps)` — the ROS/UART side, in m/s, normalized against `ROBOT_MAX_MPS = 0.60`. **Note:** left/right are deliberately swapped in code (`s_target_left = norm_r`) — a software fix compensating for reversed wiring on the hardware.
  - `motor_update(throttle, steer)` — the web UI side, in the -1000..1000 range; `left = throttle - steer`, `right = throttle + steer`, a classic arcade-drive mix.
- `motor_stop()` zeroes both targets; it is called automatically before an OTA update begins and when a WebSocket session times out.

## Wi-Fi Web Control Interface

The `web_ui` component embeds `index.html` into flash at build time (`EMBED_FILES`) and exposes three HTTP endpoints:

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Returns the embedded single-page control panel (joystick + throttle slider + power limit). |
| `/ws` | WebSocket | Client sends a text frame formatted as `"<token>;<throttle>,<steer>"`; the server calls `motor_update()` and replies with `GRANTED`/`LOCKED`. |
| `/update` | POST | Streams the raw firmware binary into `ota_manager` (see below). |

**Single-user lock (session lock):** the first connecting client's randomly generated browser-side `myToken` (`"C_" + a random 6-digit number`) is stored server-side as the "active token." Commands from a different token receive a `LOCKED` response and never reach the motors. If the active client stops sending a heartbeat for **more than 1.5 seconds** (`SESSION_TIMEOUT_US`), the lock is released and `motor_stop()` is called — a simple "deadman switch" that prevents a dropped joystick connection from leaving the vehicle moving indefinitely.

## OTA (Over-the-Air Update)

- The raw `.bin` file POSTed to `/update` is read in 1 KB chunks via `httpd_req_recv` and streamed directly into the inactive OTA partition (`ota_0`/`ota_1`, alternating) via `ota_manager_write()`.
- **`motor_stop()` is called as a safety measure** before the upload starts — the vehicle stays stationary while firmware is being updated.
- Once writing completes, `esp_ota_end()` verifies it, `esp_ota_set_boot_partition()` marks the new partition for boot, and a task with a 1.5-second delay triggers `esp_restart()` (giving the HTTP response time to reach the browser).
- There is **no authentication or signature verification** — any device on the network can POST to `/update` and replace the firmware (see [Known Gaps](#known-gaps-and-risks)).

## Hardware Pin Map

| Signal | GPIO | Description |
|---|---|---|
| `MOTOR_L_ENA` | 18 | Left track PWM (LEDC channel 0) |
| `MOTOR_L_IN1` / `IN2` | 19 / 21 | Left track direction |
| `MOTOR_R_ENB` | 22 | Right track PWM (LEDC channel 1) |
| `MOTOR_R_IN3` / `IN4` | 23 / 25 | Right track direction |
| `UART_TX_PIN` | 17 | UART2 TX → Raspberry Pi RX |
| `UART_RX_PIN` | 16 | UART2 RX ← Raspberry Pi TX |

## Partition Table

`partitions.csv` defines a layout supporting dual OTA images:

| Name | Type | Offset | Size |
|---|---|---|---|
| `nvs` | data/nvs | `0x9000` | 16 KB |
| `otadata` | data/ota | `0xd000` | 8 KB |
| `phy_init` | data/phy | `0xf000` | 4 KB |
| `ota_0` | app/ota_0 | `0x10000` | 1.5 MB |
| `ota_1` | app/ota_1 | `0x190000` | 1.5 MB |

Each app partition allows firmware up to 1.5 MB; `ota_manager` automatically selects the inactive partition via `esp_ota_get_next_update_partition()`.

## Setup and Build

Target chip: **ESP32** (Xtensa), set in `sdkconfig` as `CONFIG_IDF_TARGET_ESP32=y`.

```bash
# After activating the ESP-IDF environment
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

After the initial flash, wireless updates can be pushed by POSTing the `.bin` produced by `idf.py app-flash` to the `/update` endpoint (e.g. `curl -X POST --data-binary @build/vehicle-control-system.bin http://<esp32-ip>/update`).

Web interface access: `http://<esp32-ip>/`, via the IP address the ESP32 obtains on its network.

## Known Gaps and Risks

- **Wi-Fi credentials are stored as plaintext in source code** (`WIFI_SSID`/`WIFI_PASS` in `wifi_ap.c`). Moving these to `Kconfig`/`menuconfig` or a separate NVS provisioning step would prevent the home/office network password from leaking if the repository is ever shared unintentionally.
- **The `telemetry` module does not produce real sensor data** — the IMU and GPS payloads are fixed/dummy values (`az = 9.81`, a steadily incrementing latitude/longitude). Integration with a real IMU (e.g. MPU6050/BNO055) and GPS module is still pending.
- **No arbitration between the UART and WebSocket command paths**: if a `cmd_vel` from ROS and a command from the web joystick are both active, whichever writes last wins; there is no equivalent here to the `twist_mux` used in the main ROS repository.
- **The `/update` endpoint performs no authentication or signature verification** — any device on the network can overwrite the firmware. Before production use, at least a simple shared-secret check or signed-image verification is recommended.
- **The UART CRC is XOR-based** (not a real CRC-8/16); it can mask multi-bit errors that cancel each other out. A noisy line (long cable, EMI) would benefit from a stronger CRC.
- **The `wifi_ap` component runs in STA mode despite its name**, and does not actually set up an Access Point — the naming doesn't match the behavior.
