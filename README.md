# TENG Sensor Scope

This repository contains two parts:

- `nrf54/`: Zephyr firmware for the Nordic **nRF54L15 DK target** (`nrf54l15dk/nrf54l15/cpuapp`) used by the course board setup
- `gui/`: a Python BLE scope GUI that displays calibrated ADS131M04 voltage streams

Minimal Zephyr RTOS firmware intended for the course proprietary board setup. The Zephyr build target used for this course setup is:

- `nrf54l15dk/nrf54l15/cpuapp`

The firmware samples **ADS131M04** 24-bit ADC data. All 4 ADC channels are enabled and streamed over BLE as signed raw counts. CH0 and CH1 are intended to be displayed as source voltage through the 100k/15k input dividers; CH2 and CH3 are direct ADC input voltage.

It transmits data over **Bluetooth LE** using custom GATT characteristics plus Nordic UART Service (NUS) for logs and scope commands.

## Attribution

Created by:

- **Jesper Sjöberg** — GitHub: [@repsejs](https://github.com/repsejs) — KTH: **@jessjobe**
- **Max Gulda** — GitHub: [@Max-Gulda](https://github.com/Max-Gulda) — KTH: **@gulda**

## Architecture

### Threads and priorities

| Thread | Priority | What it does | Key code |
|---|---:|---|---|
| Sampling | 5 | Reads ADC on ADS131M04 DRDY falling edges and queues data to BLE | `nrf54/src/sampling/sampling.c` |
| Bluetooth | 7 | Batches and transmits ADC notifications; manages connect/disconnect | `nrf54/src/bluetooth/bluetooth.c`, `nrf54/src/bluetooth/bt_*.c` |
| Main | default | Initialization + periodic CPU load logging | `nrf54/src/main.c` |

### Data flow

```text
ADS131M04 CH0-CH3 ─► sampling_thread ───────────────────────────────────────────────► BLE ADC channel (batched notifications)
LOG_* / printk ─────────────────────────────► NUS (BLE log backend)
```

### Source layout (firmware)

- `nrf54/src/ads131m04_spi/` — ADS131M04 SPI driver + configuration
- `nrf54/src/sampling/` — DRDY-driven sampling thread, queues ADC frames to BLE
- `nrf54/src/bluetooth/` — BLE GATT services, batching, queues, and notifications
- `nrf54/src/ble_log_backend/` — routes Zephyr `LOG_*` output over BLE (NUS)

### ADS131M04 notes

- The old `ads131m02_spi` module was renamed to `ads131m04_spi`.
- The driver parses 4 ADC channels and exposes CH0, CH1, CH2, and CH3 in `ads131m04_data_t`.
- The default setup in `ads131m04_full_setup()` enables all four channels.
- CH0 and CH1 have 100k/15k dividers and are converted to source voltage in the GUI.
- CH2 and CH3 have no divider and are converted to direct ADC input voltage in the GUI.
- The main custom BLE ADC payload carries timestamp + CH0 + CH1 + CH2 + CH3.
- Sampling is driven by the ADS131M04 DRDY pin. The default ADC OSR is 8192 (500 SPS), and the GUI can switch among the standard ADS131M04 output data rates.

## Build and run (VS Code + nRF Connect extension)

### Important (VS Code workspace)

When using the **nRF Connect for VS Code** extension, make sure the **opened workspace folder is `nrf54/`** (not the repo root). Otherwise the extension may not detect the application correctly.

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

To run the BLE scope GUI:

```bash
cd gui
uv sync
uv run teng-sensor-scope
```

Use **Settings** in the GUI to choose visible channels and plot window length.
