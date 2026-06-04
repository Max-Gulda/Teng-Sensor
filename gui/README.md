# TENG Sensor Scope GUI (Python)

Desktop GUI that connects over BLE and displays:

- Calibrated ADS131M04 channel voltages CH0, CH1, CH2, and CH3
- Per-channel visibility, plot window length, and display filter controls
- CSV recordings saved under `Recordings/` in the repository root
- Firmware logs over NUS in the launching terminal

## Run (uv)

From the repo root:

```bash
cd gui
uv sync
uv run teng-sensor-scope
```

The old `uv run ecg-belt-gui` command remains as a compatibility alias.

## Voltage Scaling

- The firmware streams raw signed ADS131M04 counts.
- CH0 and CH1 use the 100k/15k voltage dividers and are displayed as source voltage.
- CH2 and CH3 have no divider and are displayed as direct ADC input voltage.
- Per-channel trim constants live in `src/ecg_belt_gui/app.py` as `CHANNEL_TRIM_GAIN` and `CHANNEL_TRIM_OFFSET_V`.

## Scope Settings

Use the **Settings** button to:

- Hide or show individual channels.
- Set the visible plot window in seconds.
- Set the ADS131M04 output data rate from the standard OSR-derived rates.
- Enable and tune the display-only Butterworth low-pass filter cutoff.
- Enable or disable the display-only 50 Hz notch filter.
- Firmware samples on ADS131M04 DRDY events at the selected output data rate.

## Recording

Use the **Record** button while connected to write incoming ADC samples to CSV. Each recording is saved as `Recordings/teng_recording_YYYYMMDD_HHMMSS.csv` from the repository root and includes sample index, raw channel counts, and calibrated channel voltages.

## Notes

- The firmware device name is expected to be `TENG_Scope`.
- BLE UUIDs are defined in `nrf54/src/bluetooth/bluetooth.c`.
- GUI toolkit: Qt via `PySide6`.
