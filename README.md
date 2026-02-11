# ESP32-C6 Zigbee Router/Gateway Project

This project implements a **Zigbee Router** using the **Seeed Studio XIAO ESP32C6**.
It acts as a signal repeater to extend your home automation network and features a **Web Monitor Interface** to view signal quality (RSSI/LQI) in real-time on your smartphone.

## Features

-   **Zigbee Router Role**: Acts as a signal repeater to extend network coverage.
-   **Web Monitor Interface**: Host a local web server to view RSSI and LQI stats on your phone (no PC required).
-   **WiFi & Zigbee Coexistence**: Optimized software coexistence to allow both radios to run simultaneously on the single-radio ESP32-C6.
-   **External Antenna Support**: Configured to use the external U.FL antenna for maximum range (GPIO 3/14).
-   **Max Transmission Power**: Set to **20dBm**.

## Hardware Requirements

-   **ESP32-C6 Development Board** (Tested with Seeed Studio XIAO ESP32C6)
-   **External 2.4GHz Antenna** (connected via U.FL)
-   **USB-C cable** (power/programming)

### Antenna Selection Guide
-   **Frequency**: 2.4 GHz
-   **Connector**: U.FL / IPEX1
-   **Recommended**: 3dBi to 6dBi gain.

## Web Interface Monitoring

The device hosts a web page to help you place it in the optimal location without needing a laptop connected via USB.

1.  **Status**: Displays if the device is `Scanning`, `Connecting`, or `Connected` to the Zigbee network.
2.  **Neighbors Table**: Lists all visible Zigbee neighbors.
    -   **RSSI (Received Signal Strength)**: Signal volume (e.g., -60dBm). Higher is better (closer to 0).
    -   **LQI (Link Quality Indicator)**: Signal clarity (0-255). Higher is better.
        -   **255**: Perfect
        -   **>100**: Good/Stable
        -   **<50**: Poor/Unstable

> **Tip**: Prioritize **LQI** over RSSI. A strong but "noisy" signal (High RSSI, Low LQI) is worse than a weaker but "clear" signal.

## Usage Guide

### 1. Configuration

To keep your WiFi credentials private, this project uses a separate configuration file.

1.  Navigate to the `main/` directory.
2.  Duplicate the example file `secrets.example.h` and name it `secrets.h`.
3.  Open `main/secrets.h` and enter your WiFi credentials:
    ```c
    #define ESP_WIFI_SSID      "YOUR_WIFI_SSID"
    #define ESP_WIFI_PASS      "YOUR_WIFI_PASSWORD"
    ```
    *(Note: `secrets.h` is already in `.gitignore`, so your passwords won't be committed).*

### 2. Build & Flash

```bash
idf.py build
idf.py -p COMx flash monitor
```

*(Where `COMx` is your serial port).*

### 3. Usage

1.  Power on the device.
2.  Open your Serial Monitor initially to see the IP address.
    -   *Log Example*: `ESP_ZB_ROUTER: got ip:192.168.1.50`
3.  Open that IP in your phone's browser: `http://192.168.1.50`
4.  Walk around your house with the device (powered by a power bank) and refresh the page to find the dead spots or the best repeater location.

## Troubleshooting

-   **"Scanned 0 Access Points"**: This indicates a Coexistence configuration issue or antenna failure. The current code uses a specific initialization sequence (WiFi Init -> Coex Enable -> Zigbee Start) to prevent this.
-   **Web Interface not loading**: Ensure the device is powered. If the Zigbee stack crashes, the WiFi might also stop. Check the USB logs.
-   **Low LQI**: Move the device away from metal objects, microwaves, or dense concrete walls.

## Resetting (Re-pairing)

To pair with a new Zigbee network:
1.  **Erase Flash**: `idf.py -p COMx erase-flash`
2.  **Re-flash**: `idf.py -p COMx flash`
3.  Enable **Permit Join** on your Zigbee Coordinator.
