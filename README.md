# ESP32-C6 Zigbee Router/Gateway Project

This project implements a **Zigbee Router** using the ESP32-C6 microcontroller. It is designed to extend the range of your Zigbee network and can be used with boards like the **Seeed Studio XIAO ESP32C6**.

## Features

-   **Zigbee Router Role**: Acts as a signal repeater to extend network coverage.
-   **External Antenna Support**: Configured to use the external U.FL antenna for maximum range (specifically for the Seeed Studio XIAO ESP32C6).
-   **Max Transmission Power**: Set to **20dBm** to ensure the best possible signal strength.
-   **Signal Monitoring**: Periodically scans and logs neighboring Zigbee devices (Coordinator, Routers, End Devices) via the USB serial monitor, displaying:
    -   RSSI (Signal strength)
    -   LQI (Link Quality Indicator)
    -   Device Type

## Hardware Requirements

-   **ESP32-C6 Development Board** (Tested with Seeed Studio XIAO ESP32C6)
-   **External 2.4GHz Antenna** (connected via U.FL)
-   USB-C cable for programming and power

## Usage Guide

### 1. Environment Setup

Ensure you have the **Espressif ESP-IDF** framework installed and configured. If you are using VS Code, the **Espressif IDF** extension is highly recommended.

### 2. Compilation

Open a terminal in the project root directory and run:

```bash
idf.py build
```

*Note: The first build may take a few minutes.*

### 3. Flashing the Device

Connect your ESP32-C6 to your computer. Find the correct COM port and run:

```bash
idf.py -p COMx flash
```

*(Replace `COMx` with your actual serial port, e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux/macOS).*

### 4. Monitoring Output

To view the logs and the signal strength of neighboring devices, run:

```bash
idf.py -p COMx monitor
```

You will see logs resembling the following every 3 seconds:

```text
W (time) ESP_ZB_ROUTER: ========== Scan Voisins #1 (RSSI > -90 dBm) ==========
I (time) ESP_ZB_ROUTER: Addr: 0x0000 (Coord)  | RSSI: -65 dBm | LQI: 255
I (time) ESP_ZB_ROUTER: Addr: 0x1A2B (Router) | RSSI: -70 dBm | LQI: 180
W (time) ESP_ZB_ROUTER: =======================================================
```

### 5. Zigbee Network Joining

-   The device is configured to join an existing Zigbee network automatically.
-   Ensure your **Zigbee Coordinator** (e.g., zigbee2mqtt, ZHA) is in **Permit Join** mode when you first power on this device.
-   Once joined, it will automatically route traffic and report neighbors.

## Troubleshooting

-   **Weak Signal?**: Ensure the external antenna is securely connected to the U.FL connector. The code explicitly enables the RF switch for the external antenna on GPIO 3 and 14.
-   **Not Joining?**: Verify your Coordinator is allowing new devices to join. You may need to factory reset the device if it was previously paired to another network (erase flash via `idf.py erase_flash` and re-flash).
