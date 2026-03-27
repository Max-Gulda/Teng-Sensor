# ECG Belt GUI (Python)

Super simple desktop GUI that connects over BLE and displays:

- Heart rate (BPM) + RR interval (from standard BLE Heart Rate Measurement `0x2A37`)
- Raw ECG samples (custom characteristic)
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

