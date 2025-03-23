# Fingerprint Sensor Arduino Project

This project demonstrates how to interface the **GT-NUCL1633K1** fingerprint sensor module with an **Arduino** using serial communication and interrupts. The goal is to enroll, identify, and manage fingerprints using a simple microcontroller setup.

## Hardware

- **Fingerprint Module**: GT-NUCL1633K1 by ADH-Tech
- **Microcontroller**: Arduino-compatible board (Uno, Mega, etc.)
- **Transistor**: 2N7000 or compatible for interrupt handling
- **Level Shifting**: (Optional) For 3.3V logic compatibility
- **Connections**:
  - **TX (sensor)** → **RX (Arduino)**
  - **RX (sensor)** ← **TX (Arduino)**
  - **Touch Interrupt Pin (sensor)** → **Digital Pin (Arduino)** (with pull-up/down configuration)
  - **GND / VCC** as appropriate (Check sensor voltage levels)

## Features

- Enroll new fingerprints
- Identify registered users
- Delete individual or all users
- Query fingerprint count
- Touch interrupt detection for real-time scanning

## How It Works

The sketch listens for an interrupt signal from the fingerprint sensor's touch output, triggering a check via serial communication. The fingerprint module communicates using a proprietary packet-based protocol detailed in the [GT-NUCL1633K1 Programming Guide](./GT-NUCL1633K1_Programming_guide_V1.3.pdf).

### Key Functions Used

- `Open()` – Initializes the sensor
- `IsPressFinger()` – Checks if a finger is on the sensor
- `Enroll(ID)` – Registers a new fingerprint with a unique ID
- `Identify()` – Compares scanned fingerprint to the database
- `DeleteID(ID)` – Removes a fingerprint by ID
- `DeleteAll()` – Wipes all stored fingerprints
- `GetEnrollCount()` – Returns number of enrolled users
- `GetEntryID()` – Finds the next available ID

## Dependencies

No external libraries are required. Communication is handled via `Serial` commands.

## Getting Started

1. Wire the GT-NUCL1633K1 to the Arduino as described above.
2. Open the Arduino IDE and upload the `rev0_1_fingerprint_Interrupt_2N700.ino` sketch.
3. Open the Serial Monitor to interact with the module (ensure the correct baud rate is set).
4. Touch the fingerprint sensor to trigger recognition or enrollment processes.

## Protocol Overview

The module uses an 8-byte packet structure for command/response communication:
