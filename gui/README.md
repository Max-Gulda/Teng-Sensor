# ECG Belt GUI (Python)

Super simple desktop GUI that connects over BLE and displays:

- Heart rate (BPM) + RR interval (from standard BLE Heart Rate Measurement `0x2A37`)
- Raw ECG samples from CH0 (custom characteristic)
- ADS131M04 channel samples CH0, CH1, CH2, and CH3 (custom characteristic)
- IMU samples (custom characteristic)

## Run (uv)

From the repo root:

```bash
cd gui
uv sync
uv run ecg-belt-gui
```

## Notes

- The firmware device name is expected to be `ECG_Belt`.
- BLE UUIDs are defined in `nrf54/src/bluetooth/bluetooth.c`.
- The firmware now uses an `ADS131M04` driver.
- All four ADS131M04 channels are enabled and streamed over BLE.
- The raw ECG plot and the HR pipeline use CH0 on this PCB.
- On the current PCB, CH1, CH2, and CH3 are floating and may look noisy or rail.
- GUI toolkit: Qt via `PySide6`.
