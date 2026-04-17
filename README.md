# AC Packet Extractor

**BASIC/AUX AC RS-485 Protocol Sniffer & Decoder for ESP32-S3**

A PlatformIO-based firmware for the **KinCony B16 (ESP32-S3)** that passively sniffs the RS-485 communication bus between a BASIC/AUX air conditioner indoor unit and its wall controller. It decodes HDLC-framed packets in real time, extracting operating mode, temperatures, fan speed, compressor status, and coil sensor readings — all output over USB-CDC serial at 115200 baud.

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Wiring](#wiring)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Building & Flashing](#building--flashing)
- [Usage](#usage)
  - [Serial Commands](#serial-commands)
  - [Output Modes](#output-modes)
- [Protocol Reference](#protocol-reference)
  - [Frame Format](#frame-format)
  - [TYPE 1 — Status Heartbeat](#type-1--status-heartbeat)
  - [TYPE 2 — Temperature Report](#type-2--temperature-report)
  - [TYPE 3 — Sensor Frame](#type-3--sensor-frame)
  - [TYPE 4 — Mode Frame](#type-4--mode-frame)
  - [Tail Pattern](#tail-pattern)
- [Decoded AC State](#decoded-ac-state)
- [Temperature Display Logic](#temperature-display-logic)
- [KinCony B16 I/O Control](#kincony-b16-io-control)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Features

- **Passive RS-485 sniffing** — reads the AC ↔ wall-controller bus without interfering with normal operation.
- **HDLC frame decoding** — automatically detects `0x7E`-delimited frames and classifies them into four known packet types plus Modbus.
- **Real-time AC state tracking** — continuously maintains a decoded view of:
  - Operating mode (Cool, Heat, Dry, Fan-Only, Off)
  - Fan speed (High, Medium, Low, Auto)
  - Set temperature (when applicable)
  - Room temperature from two independent NTC sensors (wall controller and indoor unit)
  - Heat-exchanger coil temperatures (two sensors)
  - Compressor on/off status
- **Intelligent fan-only handling** — correctly suppresses stale set-temperature bytes that the protocol carries in fan-only mode.
- **Polarity detection** — warns when RS-485 A/B lines are swapped.
- **KinCony B16 relay/input control** — built-in Modbus RTU master for the B16's 16-channel output relays and 16-channel digital inputs.
- **Interactive serial console** — human-friendly commands for on-demand status, verbose/quiet toggling, and relay control.

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Board** | KinCony B16 (ESP32-S3-based) |
| **RS-485 transceiver** | Built into the B16; directly wired to the AC bus |
| **AC system** | BASIC / AUX brand split-system air conditioner with RS-485 communication between indoor unit and wall controller |
| **USB cable** | USB-C (or micro-USB, depending on B16 revision) for flashing and serial monitor |

---

## Wiring

The RS-485 bus uses two wires — **A** and **B** — connected between the air conditioner's indoor unit and the wall controller. The KinCony B16 taps into this bus in parallel.

| B16 Pin | ESP32-S3 GPIO | Function |
|---------|---------------|----------|
| RS-485 TX | GPIO 39 | Transmit (used for Modbus relay commands) |
| RS-485 RX | GPIO 38 | Receive (sniffs AC packets) |

**Bus parameters:** 9600 baud, 8 data bits, no parity, 1 stop bit (8N1).

> **Polarity note:** If the serial monitor floods with `[POLARITY]` warnings (most bytes are `0xFF`), swap the A and B wires at the B16's RS-485 terminal.

---

## Getting Started

### Prerequisites

1. **PlatformIO** — Install via [VS Code extension](https://platformio.org/install/ide?install=vscode) or the [CLI](https://docs.platformio.org/en/latest/core/installation.html).
2. **USB-CDC driver** — The firmware enables USB CDC on boot (`ARDUINO_USB_CDC_ON_BOOT=1`). On Windows you may need to install the ESP32-S3 USB driver; macOS and Linux typically work out of the box.

### Building & Flashing

```bash
# Clone the repository
git clone https://github.com/naveedullah41/ac-packet-extractor.git
cd ac-packet-extractor

# Build
pio run

# Flash (connect the B16 via USB first)
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

> **Tip:** PlatformIO will automatically download the ESP32-S3 toolchain and Arduino framework on the first build.

---

## Usage

Once flashed, open a serial terminal at **115200 baud**. The firmware immediately begins listening on the RS-485 bus and printing decoded frames.

### Serial Commands

Type any of the following commands and press Enter:

| Command | Description |
|---------|-------------|
| `status` | Print a formatted snapshot of the current decoded AC state |
| `sniff` | Enable verbose output — all frame types are printed with hex dumps (default) |
| `quiet` | Suppress most output; only TYPE 2 temperature reports and state-change lines are shown |
| `ro` | Read the state of all 16 B16 output relays (ON / OFF) |
| `ri` | Read the state of all 16 B16 digital inputs (HIT / idle) |
| `on N` | Turn on output relay N (1–16) |
| `off N` | Turn off output relay N (1–16) |
| `help` | Show the list of available commands |

### Output Modes

- **Verbose (default / `sniff`)** — Every decoded frame is printed with its classification tag (`[S]`, `[T2]`, `[SENSOR]`, `[MODE]`, `[MODBUS]`, `[?]`), decoded values, and raw hex. Tail patterns are expanded in a bordered box.
- **Quiet (`quiet`)** — Only TYPE 2 temperature report lines are printed, keeping the output clean for logging.

---

## Protocol Reference

The AC system communicates over RS-485 using **HDLC-style framing** with `0x7E` as the frame delimiter. Frames appear in repeating cycles with four identified types.

### Frame Format

All frames begin with `0x7E`. The firmware collects bytes between delimiters and flushes incomplete frames after a 200 ms timeout.

### TYPE 1 — Status Heartbeat

A 10-byte frame sent roughly three times per cycle.

```
7E  F1  [seq]  [room_raw]  [wall_cap]  00  00  21  [seq2]  [cs]
```

| Byte | Field | Description |
|------|-------|-------------|
| 0 | `0x7E` | Frame delimiter |
| 1 | `0xF1` | Frame type marker |
| 2 | `seq` | Sequence number (upper 2 bits = `0xC0`) |
| 3 | `room_raw` | Wall controller NTC raw value → `(raw − 0x28) × 0.5 °C` |
| 4 | `wall_cap` | Wall controller capability byte (always `0x0A`) |
| 5–6 | `0x00 0x00` | Reserved |
| 7 | `0x21` | Fixed marker |
| 8 | `seq2` | Secondary sequence |
| 9 | `cs` | Checksum |

### TYPE 2 — Temperature Report

A variable-length frame carrying the mode/fan byte and set temperature.

```
7E  01  F1  [01|02]  0D  [mf]  00  00  [set×10]  00  ...
```

| Byte | Field | Description |
|------|-------|-------------|
| 0 | `0x7E` | Frame delimiter |
| 1–2 | `0x01 0xF1` | Frame type markers |
| 3 | Sub-type | `0x01` or `0x02` |
| 4 | `0x0D` | Fixed marker |
| 5 | `mf` | Mode + fan combined byte (see table below) |
| 8 | `set×10` | Set temperature × 10 (e.g., `0xF0` = 24.0 °C) — **stale in fan-only mode** |

**Mode + Fan byte (`mf`) encoding:**

| Upper nibble | Mode |
|--------------|------|
| `0x9_` | Cool |
| `0xA_` | Dry |
| `0xC_` | Heat |
| `0xE_` | Fan-Only |
| `0x1_` | Off |

| Lower nibble | Fan Speed |
|--------------|-----------|
| `0x_2` | High |
| `0x_4` | Medium |
| `0x_6` | Low |
| `0x_A` | Auto |

> Fan speed is only meaningful in Cool and Heat modes. Dry, Fan-Only, and Off modes report `N/A`.

### TYPE 3 — Sensor Frame

An 11-byte frame carrying the indoor unit's NTC temperature and compressor status.

```
7E  F1  F1  56  0B  [pipe_raw]  01  00  [comp]  [cs]  (7E)
```

| Byte | Field | Description |
|------|-------|-------------|
| 5 | `pipe_raw` | Indoor unit NTC raw → `(raw − 0x28) × 0.5 °C` |
| 8 | `comp` | `0x40` = compressor ON, `0x00` = compressor OFF |

> **Note:** The indoor unit NTC and wall controller NTC are physically separate sensors. They can differ by several degrees because they are in different locations.

### TYPE 4 — Mode Frame

A 21-byte frame that carries a separate mode indicator.

```
7E  F1  00  A1  15  00  [mode_raw]  ...
```

| `mode_raw` | Mode |
|------------|------|
| `0x61` | Cool |
| `0x41` | Dry |
| `0x81` | Heat |
| `0x00` | Fan-Only / Off |

### Tail Pattern

A recurring byte pattern found embedded in TYPE 2 frames and other larger frames:

```
[mf]  00  00  [set×10]  00  [coil1×10]  00  [coil2×10]  00  13  [cshi]  [cslo]
```

| Field | Description |
|-------|-------------|
| `mf` | Mode + fan byte (same encoding as TYPE 2) |
| `set×10` | Set temperature × 10 (16-bit, big-endian) |
| `coil1×10` | Heat-exchanger coil sensor 1, × 10 |
| `coil2×10` | Heat-exchanger coil sensor 2, × 10 |
| `0x13` | Tail terminator |

> **Fan-only rule:** When `mf` indicates fan-only mode, the `set×10` field contains a stale value from the previous mode and is ignored by the decoder.

---

## Decoded AC State

The `status` command prints a formatted table with all tracked values:

```
  ┌── AC STATE ──────────────────────────────────────────────┐
  │  Mode (T4)       : COOL  (0x61)                         │
  │  Mode+Fan        : COOL + HIGH  (mf=0x92)               │
  │  Compressor      : ON                                    │
  ├──────────────────────────────────────────────────────────┤
  │  Set temp        : 24.0°C                                │
  │  Room NTC (wall) : 25.5°C                                │
  │  Room NTC (unit) : 26.0°C                                │
  ├──────────────────────────────────────────────────────────┤
  │  Coil 1          : 12.5°C                                │
  │  Coil 2          : 11.0°C                                │
  ├──────────────────────────────────────────────────────────┤
  │  Room age        : 1200 ms                               │
  │  Unit NTC age    : 800 ms                                │
  │  Set temp age    : 1500 ms                               │
  │  Mode age        : 3000 ms                               │
  └──────────────────────────────────────────────────────────┘
```

---

## Temperature Display Logic

The firmware mirrors what the physical LCD wall controller displays:

| Mode | LCD Shows | Source |
|------|-----------|--------|
| Cool / Heat / Dry | Set temperature | TYPE 2 byte 8 (÷ 10) |
| Fan-Only | Indoor unit NTC | TYPE 3 byte 5 (decoded) |
| Off | Last setpoint or room temp | Unit-dependent behavior |

---

## KinCony B16 I/O Control

The KinCony B16 board includes 16 relay outputs and 16 digital inputs accessible via **Modbus RTU** on the same RS-485 bus (address `0x01`).

The firmware includes a built-in Modbus master that can:

- **Read all 16 outputs** — `ro` command, uses Modbus function code `0x01` (Read Coils).
- **Read all 16 inputs** — `ri` command, uses Modbus function code `0x02` (Read Discrete Inputs).
- **Set individual outputs** — `on N` / `off N` commands, uses Modbus function code `0x05` (Write Single Coil).

> **Note:** Modbus commands are transmitted on the same RS-485 bus as the AC traffic. The firmware handles both protocols simultaneously.

---

## Project Structure

```
ac-packet-extractor/
├── platformio.ini          # PlatformIO configuration (ESP32-S3, Arduino framework)
├── src/
│   ├── main.cpp            # Main firmware — HDLC receiver, frame decoder, serial console
│   └── main_test.cpp.bak   # Backup of a basic serial test sketch
├── include/                # Project header files (currently empty)
├── lib/                    # Project-specific libraries (currently empty)
├── test/                   # PlatformIO unit tests (currently empty)
└── .vscode/
    └── extensions.json     # Recommends PlatformIO IDE extension
```

### Key Source Sections (`src/main.cpp`)

| Section | Description |
|---------|-------------|
| Hardware defines | GPIO pins, baud rate, Modbus address |
| HDLC frame collector | Collects bytes between `0x7E` delimiters with timeout flush |
| `AcState` struct | Stores all decoded AC parameters with timestamps |
| Decoders (`mode_str`, `fan_str`, etc.) | Convert raw bytes to human-readable strings |
| `classify()` | Determines frame type from header bytes |
| `scan_tail()` | Searches for the embedded tail pattern in any frame |
| `decode_frame()` | Main dispatcher — routes frames to type-specific handlers |
| `hdlc_rx()` | Interrupt-free polling receiver for the RS-485 UART |
| Modbus helpers | CRC calculation, relay control, input readback |
| `setup()` / `loop()` | Arduino entry points — initialization and main polling loop |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `[POLARITY]` warnings flooding output | RS-485 A/B wires are swapped | Swap A and B at the B16 terminal |
| No frames detected | Wrong baud rate or disconnected bus | Verify 9600 baud wiring and that the AC system is powered on |
| `[?]` unknown frames | Unrecognized packet format | These are normal — not all frames on the bus are documented |
| `[HDLC] overflow` | Frame exceeds 128-byte buffer | Rare; may indicate electrical noise on the bus |
| Serial monitor shows garbage | Wrong monitor baud rate | Set your terminal to **115200 baud** |
| No USB serial port appears | USB-CDC not enabled or driver missing | Ensure `ARDUINO_USB_CDC_ON_BOOT=1` build flag is set (it is by default in `platformio.ini`) |
| Set temp shows stale value in Fan-Only | Expected behavior | The protocol transmits a leftover byte; the decoder correctly ignores it |

---

## License

This project does not currently specify a license. Please contact the repository owner for usage terms.
