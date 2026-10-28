# ESP32-C5 TWAI (CAN-FD) Manager

A robust, asynchronous CAN-FD implementation for the ESP32-C5 using ESP-IDF v6.0+. This project demonstrates how to initialize the TWAI (Two-Wire Automotive Interface) controller, manage asynchronous transmissions, handle FreeRTOS-based reception, and automatically pad arbitrary payload sizes to standard CAN-FD Data Length Codes (DLC).

## Features

- **Full CAN-FD Support**: Configured for 500 kbps arbitration and 2 Mbps data phase (Bit Rate Switch enabled).
- **Asynchronous TX**: Utilizes a custom `twai_manager` component with an internal slot-based buffer to prevent blocking.
- **Interrupt-Driven RX**: Receives frames in the hardware ISR and safely passes them to a FreeRTOS worker task via a queue.
- **Automatic DLC Padding**: Send custom payload lengths (e.g., 34 bytes) and the manager will automatically pad them with zeros to the nearest valid CAN-FD size (48 bytes / DLC 14).
- **Bus Health Monitoring**: A dedicated supervisor task continuously monitors the bus state and automatically recovers the node if it enters a Bus-Off state.
- **ESP-IDF v6 Compatible**: Uses the updated `twai_frame_t` structure (with the nested `header` sub-struct).
- **Single-Core Optimized**: Tasks are pinned to Core 0, ensuring stability on single-core RISC-V architectures like the ESP32-C5.

## Hardware Requirements

- **Development Board**: ESP32-C5 (or any other ESP32 series with TWAI-FD support).
- **CAN-FD Transceiver**: A transceiver that explicitly supports CAN-FD speeds up to at least 2 Mbps (e.g., TJA1044GT/3, TJA1051T/3, or MCP2562FD). *Note: Standard CAN 2.0 transceivers like the SN65HVD233 will not work reliably in FD mode.*

### Wiring

| ESP32-C5 Pin | Transceiver Pin |
|--------------|-----------------|
| GPIO 4       | TXD             |
| GPIO 5       | RXD             |
| 3.3V / 5V    | VCC             |
| GND          | GND             |

*(Adjust `CAN_TX_PIN` and `CAN_RX_PIN` in `main.c` if using different GPIOs).*

## Project Structure

Ensure your project is structured as follows so the CMake build system registers the custom component correctly:

```text
your_project/
├── CMakeLists.txt                <-- Main project CMakeLists
├── main/
│   ├── CMakeLists.txt
│   └── main.c                    <-- Application code
└── components/
    └── twai_manager/             <-- Custom TWAI Manager Component
        ├── CMakeLists.txt
        ├── twai_manager.c
        └── include/
            └── twai_manager.h