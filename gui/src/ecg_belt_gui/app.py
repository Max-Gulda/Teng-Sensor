import asyncio
import struct
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg


# -----------------------------
# BLE scanning in a Qt thread
# -----------------------------

class ScanThread(QtCore.QThread):
    resultsReady = QtCore.Signal(object)   # list[(address:str, name:str|None, rssi:int|None)]
    errorMessage = QtCore.Signal(str)
    logMessage = QtCore.Signal(str)

    def __init__(self, *, timeout_s: float = 5.0, parent=None):
        super().__init__(parent=parent)
        self._timeout_s = float(timeout_s)

    def run(self):
        try:
            devices = asyncio.run(self._scan())
            self.resultsReady.emit(devices)
        except Exception as exc:
            self.errorMessage.emit(str(exc))

    async def _scan(self) -> list[tuple[str, str | None, int | None]]:
        # First attempt: discover() (simple)
        self.logMessage.emit("Using BleakScanner.discover()...")
        try:
            found = await asyncio.wait_for(
                BleakScanner.discover(timeout=self._timeout_s),
                timeout=self._timeout_s + 5.0,
            )
            devices = self._normalize(found)
            if devices:
                return devices
            self.logMessage.emit("discover() returned 0 devices, trying callback scan...")
        except asyncio.TimeoutError:
            self.logMessage.emit("discover() timed out, trying callback scan...")
        except Exception as exc:
            self.logMessage.emit(f"discover() error: {exc}; trying callback scan...")

        # Fallback: callback scan (start/stop) with hard timeouts
        seen: dict[str, tuple[str, str | None, int | None]] = {}

        def cb(device: BLEDevice, adv_data):
            addr = getattr(device, "address", "") or ""
            if not addr:
                return
            name = getattr(adv_data, "local_name", None)
            rssi = getattr(adv_data, "rssi", None)
            prev = seen.get(addr)
            if prev is None:
                seen[addr] = (addr, (name or getattr(device, "name", None)), rssi if isinstance(rssi, int) else None)
                return
            _addr, pname, prssi = prev
            if isinstance(rssi, int) and (prssi is None or rssi > prssi):
                seen[addr] = (_addr, name or pname, rssi)
            elif name and not pname:
                seen[addr] = (_addr, name, prssi)

        scanner = BleakScanner(detection_callback=cb)
        await asyncio.wait_for(scanner.start(), timeout=5.0)
        await asyncio.sleep(self._timeout_s)
        await asyncio.wait_for(scanner.stop(), timeout=5.0)

        devices = list(seen.values())
        devices.sort(key=self._sort_key)
        return devices

    @staticmethod
    def _normalize(found: list[BLEDevice]) -> list[tuple[str, str | None, int | None]]:
        out: list[tuple[str, str | None, int | None]] = []
        for d in found:
            addr = getattr(d, "address", "") or ""
            if not addr:
                continue
            name = (getattr(d, "name", "") or "").strip() or None
            rssi = getattr(d, "rssi", None)
            out.append((addr, name, rssi if isinstance(rssi, int) else None))
        out.sort(key=ScanThread._sort_key)
        return out

    @staticmethod
    def _sort_key(item: tuple[str, str | None, int | None]):
        _addr, name, rssi = item
        nm = (name or "").strip()
        return (0 if nm else 1, -(rssi if isinstance(rssi, int) else -999))


class BleStreamThread(QtCore.QThread):
    statusMessage = QtCore.Signal(str)
    connectionChanged = QtCore.Signal(bool)
    hrmReady = QtCore.Signal(object)      # bytes
    ecgRawReady = QtCore.Signal(object)   # bytes
    ecgAuxReady = QtCore.Signal(object)   # bytes
    imuReady = QtCore.Signal(object)      # bytes
    deviceNameReady = QtCore.Signal(str)
    nusLogReady = QtCore.Signal(str)

    def __init__(self, *, address: str, name: str | None = None, parent=None):
        super().__init__(parent=parent)
        self._address = str(address)
        self._name = (name or "").strip() or None
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        try:
            asyncio.run(self._run_async())
        except Exception as exc:
            self.statusMessage.emit(f"BLE error: {exc}")
        finally:
            self.connectionChanged.emit(False)

    async def _run_async(self):
        addr = self._address
        name = self._name or "Unknown"
        self.statusMessage.emit(f"Connecting to {name} ({addr})...")
        async with BleakClient(addr) as client:
            self.connectionChanged.emit(True)
            try:
                raw = await client.read_gatt_char(UUID_GAP_DEVICE_NAME)
                resolved = bytes(raw).decode("utf-8", errors="replace").strip()
                if resolved:
                    self.deviceNameReady.emit(resolved)
                    self.statusMessage.emit(f"Resolved device name: {resolved}")
            except Exception:
                pass

            self.statusMessage.emit("Connected. Subscribing...")

            def on_hrm(_s, data: bytearray):
                if self._running:
                    self.hrmReady.emit(bytes(data))

            def on_ecg(_s, data: bytearray):
                if self._running:
                    self.ecgRawReady.emit(bytes(data))

            def on_ecg_aux(_s, data: bytearray):
                if self._running:
                    self.ecgAuxReady.emit(bytes(data))

            def on_imu(_s, data: bytearray):
                if self._running:
                    self.imuReady.emit(bytes(data))

            def on_nus(_s, data: bytearray):
                if not self._running:
                    return
                try:
                    self.nusLogReady.emit(bytes(data).decode("utf-8", errors="replace"))
                except Exception:
                    pass

            await client.start_notify(UUID_HR_MEASUREMENT, on_hrm)
            await client.start_notify(UUID_ECG_RAW_DATA, on_ecg)
            await client.start_notify(UUID_ECG_DATA, on_ecg_aux)
            await client.start_notify(UUID_IMU_DATA, on_imu)
            try:
                await client.start_notify(UUID_NUS_TX, on_nus)
                self.statusMessage.emit("NUS log stream enabled.")
            except Exception:
                pass
            self.statusMessage.emit("Streaming started.")

            while self._running and client.is_connected:
                await asyncio.sleep(0.1)

            self.statusMessage.emit("Stopping notifications...")
            for uuid in (UUID_HR_MEASUREMENT, UUID_ECG_RAW_DATA, UUID_ECG_DATA, UUID_IMU_DATA, UUID_NUS_TX):
                try:
                    await client.stop_notify(uuid)
                except Exception:
                    pass


# Custom service UUIDs (see nrf54/src/bluetooth/bluetooth.c)
UUID_ECG_SERVICE = "12345678-1234-5678-1234-56789abcdef0"
UUID_ECG_DATA = "12345678-1234-5678-1234-56789abcdef1"      # ecg_data_t batches (timestamp + ecg_aux)
UUID_IMU_DATA = "12345678-1234-5678-1234-56789abcdef2"      # imu_data_t batches
UUID_ECG_RAW_DATA = "12345678-1234-5678-1234-56789abcdef3"  # ecg_raw_data_t batches (timestamp + heart)

# Standard Heart Rate Measurement characteristic UUID (0x2A37)
UUID_HR_MEASUREMENT = "00002a37-0000-1000-8000-00805f9b34fb"

# Nordic UART Service (NUS) TX characteristic UUID (logs from firmware)
UUID_NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# Standard GAP Device Name characteristic UUID (0x2A00)
UUID_GAP_DEVICE_NAME = "00002a00-0000-1000-8000-00805f9b34fb"

# LSM6DSO accel sensitivity used by the firmware configuration:
# accel full-scale = +/-4g  -> 0.122 mg/LSB
ACCEL_G_PER_LSB = 0.000122


@dataclass
class UiState:
    connected: bool = False
    device: str = ""
    bpm: int | None = None
    rr_ms: int | None = None
    ecg_last: int | None = None
    imu_last: tuple[int, int, int, int, int, int] | None = None


def parse_hrm(payload: bytes) -> tuple[int | None, int | None]:
    """
    Parse BLE Heart Rate Measurement (0x2A37).
    Returns (bpm, rr_ms) where rr_ms is the *last* RR interval in the packet if present.
    """
    if not payload:
        return None, None

    flags = payload[0]
    hr_16bit = bool(flags & 0x01)
    rr_present = bool(flags & 0x10)

    idx = 1
    if hr_16bit:
        if len(payload) < idx + 2:
            return None, None
        bpm = int.from_bytes(payload[idx : idx + 2], "little")
        idx += 2
    else:
        if len(payload) < idx + 1:
            return None, None
        bpm = payload[idx]
        idx += 1

    # Skip "Energy Expended" field if present (flags bit 3)
    if flags & 0x08:
        idx += 2

    rr_ms = None
    if rr_present:
        # RR-interval values are uint16 in units of 1/1024 s (per spec).
        # There may be multiple RR values; use the last complete one.
        while idx + 2 <= len(payload):
            rr_1024 = int.from_bytes(payload[idx : idx + 2], "little")
            idx += 2
            rr_ms = int(rr_1024 * 1000 / 1024)

    return bpm, rr_ms


def iter_structs(fmt: str, payload: bytes):
    size = struct.calcsize(fmt)
    for off in range(0, len(payload) - (len(payload) % size), size):
        yield struct.unpack_from(fmt, payload, off)


class App(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ECG Belt GUI (BLE)")
        self.resize(900, 700)

        self.state = UiState()
        self._streaming = False
        self._scan_results: list[tuple[str, str | None]] = []  # (address, name)
        self._scan_thread: "ScanThread | None" = None
        self._reader: "BleStreamThread | None" = None

        self._ecg_buf: list[float] = []
        self._ecg_aux_buf: list[float] = []
        self._imu_ax_g: list[float] = []
        self._imu_ay_g: list[float] = []
        self._imu_az_g: list[float] = []
        self._max_plot_samples = 2000

        # Display conditioning (raw stream is ADC counts).
        self._ecg_lp: float = 0.0
        self._ecg_aux_lp: float = 0.0
        self._ecg_detrend_alpha: float = 0.01

        self._status = QtWidgets.QLabel("Disconnected")
        self._device = QtWidgets.QLabel("-")
        self._bpm = QtWidgets.QLabel("-")
        self._rr = QtWidgets.QLabel("-")
        self._ecg = QtWidgets.QLabel("-")
        self._imu = QtWidgets.QLabel("-")
        self._log = QtWidgets.QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)

        header = QtWidgets.QGridLayout()
        header.setColumnStretch(1, 1)

        header.addWidget(QtWidgets.QLabel("Status:"), 0, 0)
        header.addWidget(self._status, 0, 1)
        header.addWidget(QtWidgets.QLabel("Device:"), 1, 0)
        header.addWidget(self._device, 1, 1)

        header.addWidget(QtWidgets.QLabel("Found BLE devices:"), 2, 0)
        self._device_combo = QtWidgets.QComboBox()
        self._device_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        header.addWidget(self._device_combo, 2, 1)

        header.addWidget(QtWidgets.QLabel("Heart rate (BPM):"), 3, 0)
        header.addWidget(self._bpm, 3, 1)
        header.addWidget(QtWidgets.QLabel("RR interval (ms):"), 4, 0)
        header.addWidget(self._rr, 4, 1)

        header.addWidget(QtWidgets.QLabel("Log:"), 5, 0, QtCore.Qt.AlignmentFlag.AlignTop)
        header.addWidget(self._log, 5, 1)

        buttons = QtWidgets.QHBoxLayout()
        self._btn_scan = QtWidgets.QPushButton("Scan")
        self._btn_stream = QtWidgets.QPushButton("Start streaming")
        self._btn_stop = QtWidgets.QPushButton("Stop")
        self._chk_ecg = QtWidgets.QCheckBox("Show ECG")
        self._chk_ecg.setChecked(True)
        self._chk_ecg_aux = QtWidgets.QCheckBox("Show ECG aux")
        self._chk_ecg_aux.setChecked(True)
        self._chk_ecg_detrend = QtWidgets.QCheckBox("Remove DC (ECG)")
        self._chk_ecg_detrend.setChecked(True)
        self._chk_ecg_invert = QtWidgets.QCheckBox("Invert (ECG)")
        self._chk_ecg_invert.setChecked(False)
        self._chk_imu = QtWidgets.QCheckBox("Show IMU")
        self._chk_imu.setChecked(True)

        self._btn_scan.clicked.connect(self._on_scan)
        self._btn_stream.clicked.connect(self._on_start_streaming)
        self._btn_stop.clicked.connect(self._on_stop_streaming)
        self._chk_ecg.stateChanged.connect(self._update_plot_visibility)
        self._chk_ecg_aux.stateChanged.connect(self._update_plot_visibility)
        self._chk_imu.stateChanged.connect(self._update_plot_visibility)

        buttons.addWidget(self._btn_scan)
        buttons.addWidget(self._btn_stream)
        buttons.addWidget(self._btn_stop)
        buttons.addSpacing(16)
        buttons.addWidget(self._chk_ecg)
        buttons.addWidget(self._chk_ecg_aux)
        buttons.addWidget(self._chk_ecg_detrend)
        buttons.addWidget(self._chk_ecg_invert)
        buttons.addWidget(self._chk_imu)
        buttons.addStretch(1)
        self._btn_stop.setEnabled(False)

        plots = QtWidgets.QVBoxLayout()
        self._ecg_plot = pg.PlotWidget(title="ECG raw")
        self._ecg_plot.showGrid(x=True, y=True, alpha=0.3)
        self._ecg_curve = self._ecg_plot.plot(pen=pg.mkPen("#00e5ff", width=1))

        self._ecg_aux_plot = pg.PlotWidget(title="ECG aux (ADC ch0)")
        self._ecg_aux_plot.showGrid(x=True, y=True, alpha=0.3)
        self._ecg_aux_curve = self._ecg_aux_plot.plot(pen=pg.mkPen("#ffb300", width=1))

        self._imu_plot = pg.PlotWidget(title="IMU accel (g)")
        self._imu_plot.showGrid(x=True, y=True, alpha=0.3)
        self._imu_curve_x = self._imu_plot.plot(pen=pg.mkPen("#ff5252", width=1), name="ax")
        self._imu_curve_y = self._imu_plot.plot(pen=pg.mkPen("#4caf50", width=1), name="ay")
        self._imu_curve_z = self._imu_plot.plot(pen=pg.mkPen("#448aff", width=1), name="az")

        plots.addWidget(self._ecg_plot, 2)
        plots.addWidget(self._ecg_aux_plot, 2)
        plots.addWidget(self._imu_plot, 2)

        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.setInterval(50)  # ~20 FPS
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start()

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(header)
        root.addLayout(buttons)
        root.addLayout(plots)

        self.destroyed.connect(lambda *_: self._on_close())

    def _log_line(self, msg: str):
        # Print logs to the launching terminal (stdout) instead of the GUI widget.
        # Keep the widget around (layout simplicity), but don't write to it.
        print(str(msg), flush=True)

    def _set_ui(self, *, status=None, device=None, bpm=None, rr=None, ecg=None, imu=None):
        if status is not None:
            self._status.setText(status)
        if device is not None:
            self._device.setText(device)
        if bpm is not None:
            self._bpm.setText(bpm)
        if rr is not None:
            self._rr.setText(rr)
        if ecg is not None:
            self._ecg.setText(ecg)
        if imu is not None:
            self._imu.setText(imu)

    def _append_plot_sample(self, buf: list[float], value: float):
        buf.append(float(value))
        if len(buf) > self._max_plot_samples:
            del buf[: len(buf) - self._max_plot_samples]

    def _refresh_plots(self):
        if self._ecg_buf:
            y = self._ecg_buf
            x = list(range(len(y)))
            self._ecg_curve.setData(x, y)
        if self._ecg_aux_buf:
            y2 = self._ecg_aux_buf
            x2 = list(range(len(y2)))
            self._ecg_aux_curve.setData(x2, y2)
        if self._imu_ax_g:
            n = len(self._imu_ax_g)
            x = list(range(n))
            self._imu_curve_x.setData(x, self._imu_ax_g)
            self._imu_curve_y.setData(x, self._imu_ay_g)
            self._imu_curve_z.setData(x, self._imu_az_g)

    def _update_plot_visibility(self):
        self._ecg_plot.setVisible(self._chk_ecg.isChecked())
        self._ecg_aux_plot.setVisible(self._chk_ecg_aux.isChecked())
        self._imu_plot.setVisible(self._chk_imu.isChecked())

    def _on_scan(self):
        self._set_ui(status="Scanning...")
        self._log_line("Scan started (5s)...")
        if self._scan_thread and self._scan_thread.isRunning():
            self._log_line("Scan already running...")
            return

        self._scan_thread = ScanThread(timeout_s=5.0)
        self._scan_thread.logMessage.connect(self._log_line)
        self._scan_thread.resultsReady.connect(self._on_scan_results)
        self._scan_thread.errorMessage.connect(self._on_scan_error)
        self._scan_thread.start()

    @QtCore.Slot(object)
    def _on_scan_results(self, devices):
        self._set_device_list(devices)
        self._set_ui(status=f"Found {len(devices)} device(s)")
        self._log_line(f"Scan finished: {len(devices)} device(s)")
        if not devices:
            self._log_line(
                "No devices found. On macOS: System Settings → Privacy & Security → Bluetooth.\n"
                "Allow Bluetooth access for the app launching Python (Terminal/iTerm), then retry."
            )

    @QtCore.Slot(str)
    def _on_scan_error(self, message: str):
        self._set_ui(status=f"Scan failed: {message}")
        self._log_line(f"Scan failed: {message}")

    def _on_start_streaming(self):
        if self._streaming:
            self._log_line("Already streaming.")
            return
        self._set_ui(status="Starting stream...")
        self._log_line("Starting stream...")
        self._btn_stream.setEnabled(False)
        self._btn_scan.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._start_streaming_thread()

    def _on_stop_streaming(self):
        self._log_line("Stopping stream...")
        self._btn_stop.setEnabled(False)
        self._stop_streaming_thread()

    def _on_close(self):
        self._stop_streaming_thread()
        self.destroy()

    def _get_selected_device(self) -> tuple[str, str] | None:
        idx = self._device_combo.currentIndex()
        if idx < 0 or idx >= len(self._scan_results):
            return None
        return self._scan_results[idx]

    def _set_device_list(self, devices: list[tuple[str, str | None, int | None]]):
        # devices: (address, name, rssi)
        self._scan_results = [(addr, name) for (addr, name, _rssi) in devices]
        self._device_combo.clear()
        for addr, name, rssi in devices:
            label_name = (name or "").strip() or "Unknown"
            rssi_txt = f", RSSI {rssi} dBm" if isinstance(rssi, int) else ""
            self._device_combo.addItem(f"{label_name} ({addr}{rssi_txt})")

    # scanning is handled by ScanThread

    def _start_streaming_thread(self):
        self._stop_streaming_thread()
        selected = self._get_selected_device()
        if selected is None:
            self._set_ui(status="Select a device first")
            self._log_line("Select a device first.")
            self._btn_stream.setEnabled(True)
            self._btn_scan.setEnabled(True)
            self._btn_stop.setEnabled(False)
            return

        # Reset plots on new stream
        self._ecg_buf.clear()
        self._ecg_aux_buf.clear()
        self._imu_ax_g.clear()
        self._imu_ay_g.clear()
        self._imu_az_g.clear()

        address, name = selected
        self._reader = BleStreamThread(address=address, name=name, parent=self)
        self._reader.statusMessage.connect(self._log_line)
        self._reader.connectionChanged.connect(self._on_stream_connection_changed)
        self._reader.deviceNameReady.connect(self._on_stream_device_name)
        self._reader.nusLogReady.connect(self._on_stream_nus_log)
        self._reader.hrmReady.connect(self._on_stream_hrm)
        self._reader.ecgRawReady.connect(self._on_stream_ecg_raw)
        self._reader.ecgAuxReady.connect(self._on_stream_ecg_aux)
        self._reader.imuReady.connect(self._on_stream_imu)
        self._reader.start()

    def _stop_streaming_thread(self):
        reader = self._reader
        self._reader = None
        if reader is not None:
            try:
                reader.stop()
                reader.wait(3000)
            except Exception:
                pass
        self._streaming = False
        self._set_ui(status="Disconnected", device="-")
        self._btn_stream.setEnabled(True)
        self._btn_scan.setEnabled(True)
        self._btn_stop.setEnabled(False)

    @QtCore.Slot(bool)
    def _on_stream_connection_changed(self, connected: bool):
        self._streaming = bool(connected)
        if connected:
            self._set_ui(status="Streaming")
        else:
            self._set_ui(status="Disconnected")
            self._btn_stream.setEnabled(True)
            self._btn_scan.setEnabled(True)
            self._btn_stop.setEnabled(False)

    @QtCore.Slot(str)
    def _on_stream_device_name(self, name: str):
        # Update the displayed device name based on the device's GAP value (0x2A00).
        current = self._device.text()
        if "(" in current and ")" in current:
            addr = current[current.find("(") + 1 : current.rfind(")")]
            self._set_ui(device=f"{name} ({addr})")
        else:
            self._set_ui(device=name)

    @QtCore.Slot(str)
    def _on_stream_nus_log(self, text: str):
        # Forward firmware logs into the GUI log panel.
        for line in text.splitlines():
            if line.strip():
                self._log_line(line)

    @QtCore.Slot(object)
    def _on_stream_hrm(self, payload: bytes):
        bpm, rr = parse_hrm(payload)
        if bpm is not None:
            self._set_ui(bpm=str(bpm))
        if rr is not None:
            self._set_ui(rr=str(rr))

    @QtCore.Slot(object)
    def _on_stream_ecg_raw(self, payload: bytes):
        last = None
        for (_ts, heart) in iter_structs("<Ii", payload):
            x = float(int(heart))
            if self._chk_ecg_detrend.isChecked():
                # One-pole LP + subtraction => simple high-pass (baseline wander removal).
                self._ecg_lp += self._ecg_detrend_alpha * (x - self._ecg_lp)
                x = x - self._ecg_lp
            if self._chk_ecg_invert.isChecked():
                x = -x
            last = int(heart)
            self._append_plot_sample(self._ecg_buf, x)
        if last is not None:
            self._set_ui(ecg=str(last))

    @QtCore.Slot(object)
    def _on_stream_ecg_aux(self, payload: bytes):
        # ecg_data_t is packed: uint32 sample_count + int32 ecg_aux
        for (_ts, ecg_aux) in iter_structs("<Ii", payload):
            x = float(int(ecg_aux))
            if self._chk_ecg_detrend.isChecked():
                self._ecg_aux_lp += self._ecg_detrend_alpha * (x - self._ecg_aux_lp)
                x = x - self._ecg_aux_lp
            self._append_plot_sample(self._ecg_aux_buf, x)

    @QtCore.Slot(object)
    def _on_stream_imu(self, payload: bytes):
        last = None
        for (_ts, ax, ay, az, gx, gy, gz) in iter_structs("<Ihhhhhh", payload):
            last = (int(ax), int(ay), int(az), int(gx), int(gy), int(gz))
            self._append_plot_sample(self._imu_ax_g, float(ax) * ACCEL_G_PER_LSB)
            self._append_plot_sample(self._imu_ay_g, float(ay) * ACCEL_G_PER_LSB)
            self._append_plot_sample(self._imu_az_g, float(az) * ACCEL_G_PER_LSB)
        if last is not None:
            ax_g = last[0] * ACCEL_G_PER_LSB
            ay_g = last[1] * ACCEL_G_PER_LSB
            az_g = last[2] * ACCEL_G_PER_LSB
            s = f"{ax_g:.3f} g, {ay_g:.3f} g, {az_g:.3f} g | gyro raw: {last[3]}, {last[4]}, {last[5]}"
            self._set_ui(imu=s)

    # BLE notification handlers are managed inside BleStreamThread


def main():
    qapp = QtWidgets.QApplication([])
    w = App()
    w.show()
    qapp.exec()

