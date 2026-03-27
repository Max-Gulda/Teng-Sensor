# ECG Belt

This repository contains two parts:

- `nrf54/`: Zephyr firmware for the Nordic **nRF54L15 DK target** (`nrf54l15dk/nrf54l15/cpuapp`) used by the course board setup
- `gui/`: a super simple Python GUI that connects over BLE and shows streamed sensor + computed values

Minimal Zephyr RTOS firmware intended for the course **proprietary ECG belt board** (distributed alongside this repo). The Zephyr build target used for this course setup is:

- `nrf54l15dk/nrf54l15/cpuapp`

The firmware samples:

- **ADS131M02** 24-bit ADC (ECG + auxiliary ADC channel in the default BLE payload)
- **LSM6DSO** 6-axis IMU (accel + gyro)

It runs **Pan‑Tompkins** heart rate detection in real time and transmits data over **Bluetooth LE** (custom GATT characteristics + Nordic UART Service (NUS) for logs).

## Attribution

Created by:

- **Jesper Sjöberg** — GitHub: [@repsejs](https://github.com/repsejs) — KTH: **@jessjobe**
- **Max Gulda** — GitHub: [@Max-Gulda](https://github.com/Max-Gulda) — KTH: **@gulda**

## Architecture

### Threads and priorities

| Thread | Priority | What it does | Key code |
|---|---:|---|---|
| Sampling | 5 | Reads ADC + IMU at the configured rate and queues data to BLE + ECG processing | `nrf54/src/sampling/sampling.c` |
| ECG | 6 | Runs Pan‑Tompkins on the ECG stream; emits HR (and RR interval) over BLE | `nrf54/src/ecg/ecg.c`, `nrf54/src/pan_tompkins/` |
| Bluetooth | 7 | Batches and transmits ECG/IMU notifications; manages connect/disconnect | `nrf54/src/bluetooth/bluetooth.c`, `nrf54/src/bluetooth/bt_*.c` |
| Main | default | Initialization + periodic CPU load logging | `nrf54/src/main.c` |

### Data flow

```text
ADS131M02 ──► sampling_thread ──► ecg_queue ──► ecg_thread ──► Pan‑Tompkins ──► BLE HR notifications
                     └──────────► BLE ECG channel (batched notifications)
LSM6DSO   ──► sampling_thread ───────────────► BLE IMU channel (batched notifications)
LOG_* / printk ─────────────────────────────► NUS (BLE log backend)
```

### Source layout (firmware)

- `nrf54/src/ads131m02_spi/` — ADS131M02 SPI driver + configuration
- `nrf54/src/lsm6dso_spi/` — LSM6DSO SPI driver + configuration
- `nrf54/src/sampling/` — timed sampling thread, queues to BLE + ECG
- `nrf54/src/ecg/` — ECG thread + Pan‑Tompkins integration
- `nrf54/src/pan_tompkins/` — R‑peak detection and BPM/RR logic
- `nrf54/src/filters/` — signal processing utilities used by the ECG pipeline
- `nrf54/src/bluetooth/` — BLE GATT services, batching, queues, and HR notifications
- `nrf54/src/ble_log_backend/` — routes Zephyr `LOG_*` output over BLE (NUS)
- `nrf54/src/circ_buffer/` — static circular buffer utilities used by the ECG/Pan‑Tompkins path

## Build and run (VS Code + nRF Connect extension)

### Prerequisites

- VS Code
- **nRF Connect for VS Code** extension installed (Nordic Semiconductor)
- A working nRF Connect SDK toolchain (the extension can install/manage this)
- The **course proprietary board** connected over USB (uses the nRF54L15 DK target)

### Open the project

1. Open this folder in VS Code.
2. In the nRF Connect sidebar:
   - Select the **application**: `nrf54/`
   - Select the board: `nrf54l15dk/nrf54l15/cpuapp`.

### Build

Use the nRF Connect extension’s **Build** action. This project uses Zephyr sysbuild (MCUboot enabled), matching:

```bash
cd nrf54
west build -b nrf54l15dk/nrf54l15/cpuapp --sysbuild
```

### Flash

Use the nRF Connect extension’s **Flash** action, or from a terminal:

```bash
west flash
```

### View logs / verify it runs

You have two common options:

- **UART/serial logs**: use the extension’s serial monitor, or:

```bash
west espresso
```

- **BLE logs over NUS**: connect with a NUS-capable client and view logs streamed by the BLE log backend (enabled in `nrf54/src/main.c`).

## IntelliSense note (students)

This repo is configured to work with VS Code’s **C/C++ IntelliSense**. For full, correct include paths/macros, build once so Zephyr generates:

- `nrf54/build/compile_commands.json`

After the first build, IntelliSense should work.

## Bluetooth DFU (nRF Device Manager)

After building, the build directory will contain a DFU package:

- `nrf54/build/dfu_application.zip`

To flash **over Bluetooth** using the **nRF Device Manager** mobile app:

1. Flash once over USB initially (so the device has the bootloader + BLE DFU enabled).
2. Power the device and connect to it in **nRF Device Manager**.
3. Choose **DFU / Update firmware** (wording varies by app version).
4. Select `dfu_application.zip` and start the update.
5. Wait for the device to reboot and reconnect.

## GUI (Python)

To run the super simple BLE GUI:

```bash
cd gui
uv sync
uv run ecg-belt-gui
```

