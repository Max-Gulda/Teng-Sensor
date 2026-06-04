import asyncio
import csv
from datetime import datetime
import math
import queue
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from PySide6 import QtCore, QtGui, QtWidgets
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
    adcFrameReady = QtCore.Signal(object) # bytes
    deviceNameReady = QtCore.Signal(str)
    nusLogReady = QtCore.Signal(str)

    def __init__(self, *, address: str, name: str | None = None, parent=None):
        super().__init__(parent=parent)
        self._address = str(address)
        self._name = (name or "").strip() or None
        self._running = True
        self._commands: queue.Queue[str] = queue.Queue()

    def stop(self):
        self._running = False

    def send_command(self, command: str):
        self._commands.put(str(command))

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

            def on_adc_frame(_s, data: bytearray):
                if self._running:
                    self.adcFrameReady.emit(bytes(data))

            def on_nus(_s, data: bytearray):
                if not self._running:
                    return
                try:
                    self.nusLogReady.emit(bytes(data).decode("utf-8", errors="replace"))
                except Exception:
                    pass

            await client.start_notify(UUID_ADC_DATA, on_adc_frame)
            try:
                await client.start_notify(UUID_NUS_TX, on_nus)
                self.statusMessage.emit("NUS log stream enabled.")
            except Exception:
                pass
            self.statusMessage.emit("Streaming started. GUI shows calibrated ADS131M04 CH0-CH3 voltages.")

            while self._running and client.is_connected:
                await self._drain_commands(client)
                await asyncio.sleep(0.1)

            self.statusMessage.emit("Stopping notifications...")
            for uuid in (UUID_ADC_DATA, UUID_NUS_TX):
                try:
                    await client.stop_notify(uuid)
                except Exception:
                    pass

    async def _drain_commands(self, client: BleakClient):
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return

            payload = (command.rstrip() + "\n").encode("utf-8")
            try:
                await client.write_gatt_char(UUID_NUS_RX, payload, response=False)
                self.statusMessage.emit(f"Sent command: {command}")
            except Exception as exc:
                self.statusMessage.emit(f"Failed to send command '{command}': {exc}")

# Custom service UUIDs (see nrf54/src/bluetooth/bluetooth.c)
UUID_ADC_SERVICE = "12345678-1234-5678-1234-56789abcdef0"
UUID_ADC_DATA = "12345678-1234-5678-1234-56789abcdef1"      # adc_data_t batches (timestamp + ch0-ch3)
# Nordic UART Service (NUS) TX characteristic UUID (logs from firmware)
UUID_NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
# Nordic UART Service (NUS) RX characteristic UUID (commands to firmware)
UUID_NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

# Standard GAP Device Name characteristic UUID (0x2A00)
UUID_GAP_DEVICE_NAME = "00002a00-0000-1000-8000-00805f9b34fb"

# ADS131M04 conversion. Firmware config uses gain=1 on all channels.
ADS131M04_VREF_V = 1.2
ADS131M04_FULL_SCALE_COUNTS = 1 << 23
ADS131M04_CHANNEL_GAINS = (1.0, 1.0, 1.0, 1.0)

# CH0/CH1 have 100k high-side and 15k low-side dividers before the ADC.
DIVIDER_HIGH_OHM = 100_000.0
DIVIDER_LOW_OHM = 15_000.0
DIVIDER_SOURCE_MULTIPLIER = (DIVIDER_HIGH_OHM + DIVIDER_LOW_OHM) / DIVIDER_LOW_OHM
ADC_INPUT_MULTIPLIERS = (
    DIVIDER_SOURCE_MULTIPLIER,
    DIVIDER_SOURCE_MULTIPLIER,
    1.0,
    1.0,
)

# Per-channel trim hooks for measured calibration. Keep offset in source volts.
CHANNEL_TRIM_GAIN = (1.0, 1.0, 1.0, 1.0)
CHANNEL_TRIM_OFFSET_V = (0.0, 0.0, 0.0, 0.0)

ADS131M04_SAMPLE_RATES = (
    ("500 SPS", 500),
    ("250 SPS", 250),
)
ADS131M04_DEFAULT_SAMPLE_RATE_HZ = 500
DEFAULT_WINDOW_SECONDS = 10.0
LOWPASS_MIN_CUTOFF_HZ = 1
LOWPASS_MAX_CUTOFF_HZ = 240
LOWPASS_DEFAULT_CUTOFF_HZ = 50
NOTCH_FREQUENCY_HZ = 50
NOTCH_Q = 30.0
ADC_CHANNEL_LABELS = (
    "CH0 source V (divider)",
    "CH1 source V (divider)",
    "CH2 ADC V",
    "CH3 ADC V",
)
ADC_CHANNEL_COLORS = ("#00e5ff", "#ffb300", "#4caf50", "#ff5252")
SETTINGS_ORG = "TENG Sensor"
SETTINGS_APP = "Data Collector"
SETTINGS_CHANNEL_VISIBLE = "scope/channel_visible"
SETTINGS_WINDOW_SECONDS = "scope/window_seconds"
SETTINGS_AC_COUPLING = "scope/ac_coupling"
SETTINGS_LOWPASS_ENABLED = "scope/lowpass_enabled"
SETTINGS_LOWPASS_CUTOFF_HZ = "scope/lowpass_cutoff_hz"
SETTINGS_NOTCH_ENABLED = "scope/notch_50hz_enabled"
SETTINGS_ADS131_SAMPLE_RATE_HZ = "scope/ads131_sample_rate_hz"
PROJECT_ROOT = Path(__file__).resolve().parents[3]
RECORDINGS_DIR = PROJECT_ROOT / "Recordings"


@dataclass
class UiState:
    connected: bool = False
    device: str = ""
    adc_last: tuple[float, float, float, float] | None = None


def iter_structs(fmt: str, payload: bytes):
    size = struct.calcsize(fmt)
    for off in range(0, len(payload) - (len(payload) % size), size):
        yield struct.unpack_from(fmt, payload, off)


def adc_counts_to_volts(channel: int, counts: int) -> float:
    adc_pin_v = (
        float(counts)
        * (ADS131M04_VREF_V / ADS131M04_CHANNEL_GAINS[channel])
        / ADS131M04_FULL_SCALE_COUNTS
    )
    source_v = adc_pin_v * ADC_INPUT_MULTIPLIERS[channel]
    return source_v * CHANNEL_TRIM_GAIN[channel] + CHANNEL_TRIM_OFFSET_V[channel]


class ButterworthLowpass:
    def __init__(self, sample_rate_hz: float, cutoff_hz: float):
        cutoff_hz = min(max(float(cutoff_hz), LOWPASS_MIN_CUTOFF_HZ), LOWPASS_MAX_CUTOFF_HZ)
        k = math.tan(math.pi * cutoff_hz / float(sample_rate_hz))
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self._b0 = k * k * norm
        self._b1 = 2.0 * self._b0
        self._b2 = self._b0
        self._a1 = 2.0 * (k * k - 1.0) * norm
        self._a2 = (1.0 - math.sqrt(2.0) * k + k * k) * norm
        self._x1 = 0.0
        self._x2 = 0.0
        self._y1 = 0.0
        self._y2 = 0.0
        self._initialized = False

    def filter(self, sample: float) -> float:
        sample = float(sample)
        if not self._initialized:
            self._x1 = sample
            self._x2 = sample
            self._y1 = sample
            self._y2 = sample
            self._initialized = True

        out = (
            self._b0 * sample
            + self._b1 * self._x1
            + self._b2 * self._x2
            - self._a1 * self._y1
            - self._a2 * self._y2
        )
        self._x2 = self._x1
        self._x1 = sample
        self._y2 = self._y1
        self._y1 = out
        return out


class NotchFilter:
    def __init__(self, sample_rate_hz: float, notch_hz: float, q: float):
        w0 = 2.0 * math.pi * float(notch_hz) / float(sample_rate_hz)
        cos_w0 = math.cos(w0)
        alpha = math.sin(w0) / (2.0 * float(q))
        a0 = 1.0 + alpha
        self._b0 = 1.0 / a0
        self._b1 = -2.0 * cos_w0 / a0
        self._b2 = 1.0 / a0
        self._a1 = -2.0 * cos_w0 / a0
        self._a2 = (1.0 - alpha) / a0
        self._x1 = 0.0
        self._x2 = 0.0
        self._y1 = 0.0
        self._y2 = 0.0
        self._initialized = False

    def filter(self, sample: float) -> float:
        sample = float(sample)
        if not self._initialized:
            self._x1 = sample
            self._x2 = sample
            self._y1 = sample
            self._y2 = sample
            self._initialized = True

        out = (
            self._b0 * sample
            + self._b1 * self._x1
            + self._b2 * self._x2
            - self._a1 * self._y1
            - self._a2 * self._y2
        )
        self._x2 = self._x1
        self._x1 = sample
        self._y2 = self._y1
        self._y1 = out
        return out


class App(QtWidgets.QWidget):
    @staticmethod
    def _make_icon(draw_fn) -> QtGui.QIcon:
        pixmap = QtGui.QPixmap(32, 32)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        draw_fn(painter)
        painter.end()
        return QtGui.QIcon(pixmap)

    @classmethod
    def _make_record_dot_icon(cls) -> QtGui.QIcon:
        def draw(painter: QtGui.QPainter):
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor("#ff3b30"))
            painter.drawEllipse(QtCore.QPointF(16, 16), 7.5, 7.5)

        return cls._make_icon(draw)

    @classmethod
    def _make_record_stop_icon(cls) -> QtGui.QIcon:
        def draw(painter: QtGui.QPainter):
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor("#ff3b30"))
            painter.drawRoundedRect(QtCore.QRectF(9, 9, 14, 14), 2, 2)

        return cls._make_icon(draw)

    @classmethod
    def _disconnect_icon(cls) -> QtGui.QIcon:
        def draw(painter: QtGui.QPainter):
            pen = QtGui.QPen(QtGui.QColor("#d7d7d7"), 3.0, QtCore.Qt.PenStyle.SolidLine)
            pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QtCore.QPointF(10, 10), QtCore.QPointF(22, 22))
            painter.drawLine(QtCore.QPointF(22, 10), QtCore.QPointF(10, 22))

        return cls._make_icon(draw)

    @classmethod
    def _settings_sliders_icon(cls) -> QtGui.QIcon:
        def draw(painter: QtGui.QPainter):
            pen = QtGui.QPen(QtGui.QColor("#d7d7d7"), 2.4, QtCore.Qt.PenStyle.SolidLine)
            pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            for y, knob_x in ((10, 20), (16, 12), (22, 17)):
                painter.drawLine(QtCore.QPointF(7, y), QtCore.QPointF(25, y))
                painter.setBrush(QtGui.QColor("#d7d7d7"))
                painter.drawEllipse(QtCore.QPointF(knob_x, y), 2.8, 2.8)

        return cls._make_icon(draw)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TENG Sensor Scope (BLE, ADS131M04)")
        self.resize(900, 700)

        self.state = UiState()
        self._streaming = False
        self._scan_results: list[tuple[str, str | None]] = []  # (address, name)
        self._scan_thread: "ScanThread | None" = None
        self._reader: "BleStreamThread | None" = None
        self._settings = QtCore.QSettings(SETTINGS_ORG, SETTINGS_APP)
        self._recording_file: TextIO | None = None
        self._recording_writer: Any | None = None
        self._recording_path: Path | None = None

        self._adc_ch0_buf: list[float] = []
        self._adc_ch1_buf: list[float] = []
        self._adc_ch2_buf: list[float] = []
        self._adc_ch3_buf: list[float] = []
        self._adc_buffers = [
            self._adc_ch0_buf,
            self._adc_ch1_buf,
            self._adc_ch2_buf,
            self._adc_ch3_buf,
        ]
        self._adc_sample_idx_buf: list[int] = []
        self._plot_sample_idx = 0
        self._channel_visible = self._load_channel_visible()
        self._sample_rate_hz = self._load_ads131_sample_rate_hz()
        self._window_seconds = self._load_window_seconds()
        self._max_plot_samples = self._window_sample_count()

        # Optional display conditioning only; ADC data remains raw on BLE.
        self._adc_lp: list[float] = [0.0, 0.0, 0.0, 0.0]
        self._ac_coupling_alpha: float = 0.01
        self._lowpass_enabled = self._load_bool(SETTINGS_LOWPASS_ENABLED, True)
        self._lowpass_cutoff_hz = self._load_lowpass_cutoff_hz()
        self._lowpass_filters = self._create_lowpass_filters()
        self._notch_enabled = self._load_bool(SETTINGS_NOTCH_ENABLED, True)
        self._notch_filters = self._create_notch_filters()

        self._status = QtWidgets.QLabel("Disconnected")
        self._device = QtWidgets.QLabel("-")
        self._adc_last = QtWidgets.QLabel("-")
        self._adc_note = QtWidgets.QLabel(
            "CH0 and CH1 are shown as source voltage through the 100k/15k dividers. "
            "CH2 and CH3 are shown as direct ADC input voltage."
        )
        self._adc_note.setWordWrap(True)

        header = QtWidgets.QGridLayout()
        header.setColumnStretch(1, 1)

        self._lbl_status = QtWidgets.QLabel("Status:")
        self._lbl_device = QtWidgets.QLabel("Device:")
        self._lbl_device_combo = QtWidgets.QLabel("Found BLE devices:")
        self._lbl_adc_note = QtWidgets.QLabel("ADC stream note:")
        self._lbl_adc_last = QtWidgets.QLabel("Last ADC voltages:")

        header.addWidget(self._lbl_status, 0, 0)
        header.addWidget(self._status, 0, 1)
        header.addWidget(self._lbl_device, 1, 0)
        header.addWidget(self._device, 1, 1)

        header.addWidget(self._lbl_device_combo, 2, 0)
        self._device_combo = QtWidgets.QComboBox()
        self._device_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        header.addWidget(self._device_combo, 2, 1)

        header.addWidget(self._lbl_adc_note, 3, 0)
        header.addWidget(self._adc_note, 3, 1)
        header.addWidget(self._lbl_adc_last, 4, 0)
        header.addWidget(self._adc_last, 4, 1)

        buttons = QtWidgets.QHBoxLayout()
        button_icon_size = QtCore.QSize(16, 16)
        self._btn_scan = QtWidgets.QPushButton("Scan")
        self._btn_stream = QtWidgets.QPushButton("Connect")
        self._btn_stop = QtWidgets.QPushButton("Disconnect")
        self._btn_record = QtWidgets.QPushButton("Record")
        self._btn_record.setCheckable(True)
        self._btn_settings = QtWidgets.QPushButton("Settings")
        self._record_icon = self._make_record_dot_icon()
        self._record_stop_icon = self._make_record_stop_icon()
        self._btn_stop.setIcon(self._disconnect_icon())
        self._btn_record.setIcon(self._record_icon)
        self._btn_settings.setIcon(self._settings_sliders_icon())
        for button in (
            self._btn_scan,
            self._btn_stream,
            self._btn_stop,
            self._btn_record,
            self._btn_settings,
        ):
            button.setIconSize(button_icon_size)
            button.setMinimumWidth(104)
            button.setMinimumHeight(28)
            button.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
            button.setStyleSheet(
                """
                QPushButton {
                    background: #5a5a5a;
                    border: 1px solid #666666;
                    border-radius: 6px;
                    color: #f2f2f2;
                    font-weight: 600;
                    padding: 5px 12px;
                }
                QPushButton:hover {
                    background: #666666;
                    border-color: #777777;
                }
                QPushButton:pressed {
                    background: #4c4c4c;
                }
                QPushButton:disabled {
                    background: #4a4a4a;
                    border-color: #515151;
                    color: #8c8c8c;
                }
                """
            )
        self._btn_record.setMinimumWidth(136)
        self._chk_ac_coupling = QtWidgets.QCheckBox("AC couple display")
        self._chk_ac_coupling.setChecked(self._load_bool(SETTINGS_AC_COUPLING, False))
        self._chk_ac_coupling.setToolTip(
            "Removes the slow DC offset from the plotted signal. CSV recordings keep the original calibrated values."
        )
        self._btn_stop.setToolTip("Disconnect from the current BLE device.")
        self._btn_record.setToolTip("Save incoming ADC samples to a CSV file in Recordings.")
        self._btn_settings.setToolTip("Open plot, sample rate, and display filter settings.")

        self._btn_scan.clicked.connect(self._on_scan)
        self._btn_stream.clicked.connect(self._on_start_streaming)
        self._btn_stop.clicked.connect(self._on_stop_streaming)
        self._btn_record.toggled.connect(self._on_record_toggled)
        self._btn_settings.clicked.connect(self._open_settings)
        self._chk_ac_coupling.stateChanged.connect(lambda _state: self._save_settings())

        buttons.addWidget(self._btn_scan)
        buttons.addWidget(self._btn_stream)
        buttons.addWidget(self._btn_stop)
        buttons.addWidget(self._btn_record)
        buttons.addWidget(self._btn_settings)
        buttons.addSpacing(16)
        buttons.addWidget(self._chk_ac_coupling)
        buttons.addStretch(1)
        self._btn_stop.setEnabled(False)
        self._btn_record.setEnabled(False)

        plots = QtWidgets.QVBoxLayout()
        self._adc_plots: list[pg.PlotWidget] = []
        self._adc_curves = []
        for label, color in zip(ADC_CHANNEL_LABELS, ADC_CHANNEL_COLORS):
            plot = pg.PlotWidget(title=label)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel("left", "Voltage", units="V")
            plot.setLabel("bottom", "Time", units="s")
            if self._adc_plots:
                plot.setXLink(self._adc_plots[0])
            curve = plot.plot(pen=pg.mkPen(color, width=1), name=label)
            self._adc_plots.append(plot)
            self._adc_curves.append(curve)
            plots.addWidget(plot, 1)
        self._update_plot_visibility()

        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.setInterval(50)  # ~20 FPS
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start()

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(header)
        root.addLayout(buttons)
        root.addLayout(plots)
        QtCore.QTimer.singleShot(0, self._on_scan)

    def _log_line(self, msg: str):
        # Print logs to the launching terminal (stdout) instead of the GUI widget.
        # Keep the widget around (layout simplicity), but don't write to it.
        print(str(msg), flush=True)

    def closeEvent(self, event):
        self._save_settings()
        self._stop_recording()
        self._stop_streaming_thread()
        super().closeEvent(event)

    def _load_bool(self, key: str, default: bool) -> bool:
        value = self._settings.value(key, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _load_window_seconds(self) -> float:
        value = self._settings.value(SETTINGS_WINDOW_SECONDS, DEFAULT_WINDOW_SECONDS)
        try:
            window_seconds = float(value)
        except (TypeError, ValueError):
            return DEFAULT_WINDOW_SECONDS
        return min(120.0, max(0.1, window_seconds))

    def _load_ads131_sample_rate_hz(self) -> int:
        value = self._settings.value(SETTINGS_ADS131_SAMPLE_RATE_HZ, ADS131M04_DEFAULT_SAMPLE_RATE_HZ)
        try:
            sample_rate_hz = int(round(float(value)))
        except (TypeError, ValueError):
            return ADS131M04_DEFAULT_SAMPLE_RATE_HZ
        valid_rates = {rate for _label, rate in ADS131M04_SAMPLE_RATES}
        if sample_rate_hz not in valid_rates:
            return ADS131M04_DEFAULT_SAMPLE_RATE_HZ
        return sample_rate_hz

    def _max_lowpass_cutoff_hz(self, sample_rate_hz: int | None = None) -> int:
        rate_hz = self._sample_rate_hz if sample_rate_hz is None else int(sample_rate_hz)
        return min(LOWPASS_MAX_CUTOFF_HZ, max(LOWPASS_MIN_CUTOFF_HZ, (rate_hz // 2) - 1))

    def _load_lowpass_cutoff_hz(self) -> int:
        value = self._settings.value(SETTINGS_LOWPASS_CUTOFF_HZ, LOWPASS_DEFAULT_CUTOFF_HZ)
        try:
            cutoff_hz = int(round(float(value)))
        except (TypeError, ValueError):
            return LOWPASS_DEFAULT_CUTOFF_HZ
        return min(self._max_lowpass_cutoff_hz(), max(LOWPASS_MIN_CUTOFF_HZ, cutoff_hz))

    def _load_channel_visible(self) -> list[bool]:
        value = self._settings.value(SETTINGS_CHANNEL_VISIBLE, "")
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(",")]
            if len(parts) == len(ADC_CHANNEL_LABELS):
                return [part.lower() in ("1", "true", "yes", "on") for part in parts]
        return [True, True, True, True]

    def _save_settings(self):
        visible = ",".join("1" if visible else "0" for visible in self._channel_visible)
        self._settings.setValue(SETTINGS_CHANNEL_VISIBLE, visible)
        self._settings.setValue(SETTINGS_WINDOW_SECONDS, self._window_seconds)
        self._settings.setValue(SETTINGS_ADS131_SAMPLE_RATE_HZ, self._sample_rate_hz)
        self._settings.setValue(SETTINGS_AC_COUPLING, self._chk_ac_coupling.isChecked())
        self._settings.setValue(SETTINGS_LOWPASS_ENABLED, self._lowpass_enabled)
        self._settings.setValue(SETTINGS_LOWPASS_CUTOFF_HZ, self._lowpass_cutoff_hz)
        self._settings.setValue(SETTINGS_NOTCH_ENABLED, self._notch_enabled)
        self._settings.sync()

    def _create_lowpass_filters(self) -> list[ButterworthLowpass]:
        return [
            ButterworthLowpass(self._sample_rate_hz, self._lowpass_cutoff_hz)
            for _ in ADC_CHANNEL_LABELS
        ]

    def _reset_lowpass_filters(self):
        self._lowpass_filters = self._create_lowpass_filters()

    def _create_notch_filters(self) -> list[NotchFilter]:
        return [
            NotchFilter(self._sample_rate_hz, NOTCH_FREQUENCY_HZ, NOTCH_Q)
            for _ in ADC_CHANNEL_LABELS
        ]

    def _reset_notch_filters(self):
        self._notch_filters = self._create_notch_filters()

    def _apply_lowpass(self, values: list[float]) -> list[float]:
        if not self._lowpass_enabled:
            return values
        return [
            filt.filter(value)
            for filt, value in zip(self._lowpass_filters, values)
        ]

    def _apply_notch(self, values: list[float]) -> list[float]:
        if not self._notch_enabled:
            return values
        return [
            filt.filter(value)
            for filt, value in zip(self._notch_filters, values)
        ]

    def _clear_plot_buffers(self):
        for buf in self._adc_buffers:
            buf.clear()
        self._adc_sample_idx_buf.clear()
        self._plot_sample_idx = 0
        self._adc_lp = [0.0, 0.0, 0.0, 0.0]

    def _send_ads131_sample_rate(self):
        if self._reader is None:
            return
        self._reader.send_command(f"ads131_rate {self._sample_rate_hz}")

    def _set_ui(self, *, status=None, device=None, adc=None):
        if status is not None:
            self._status.setText(status)
        if device is not None:
            self._device.setText(device)
        if adc is not None:
            self._adc_last.setText(adc)

    def _set_connection_info_collapsed(self, collapsed: bool):
        for widget in (
            self._lbl_status,
            self._status,
            self._lbl_device,
            self._device,
            self._lbl_device_combo,
            self._device_combo,
            self._lbl_adc_note,
            self._adc_note,
            self._lbl_adc_last,
            self._adc_last,
            self._btn_scan,
            self._btn_stream,
        ):
            widget.setVisible(not collapsed)

    def _on_record_toggled(self, checked: bool):
        if checked:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._recording_writer is not None:
            return
        try:
            RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = RECORDINGS_DIR / f"teng_recording_{timestamp}.csv"
            handle = path.open("w", newline="", encoding="utf-8")
            writer = csv.writer(handle)
            writer.writerow([
                "sample_index",
                "ch0_counts",
                "ch1_counts",
                "ch2_counts",
                "ch3_counts",
                "ch0_volts",
                "ch1_volts",
                "ch2_volts",
                "ch3_volts",
            ])
        except OSError as exc:
            self._recording_file = None
            self._recording_writer = None
            self._recording_path = None
            self._btn_record.blockSignals(True)
            self._btn_record.setChecked(False)
            self._btn_record.blockSignals(False)
            self._set_ui(status=f"Recording failed: {exc}")
            self._log_line(f"Recording failed: {exc}")
            return

        self._recording_file = handle
        self._recording_writer = writer
        self._recording_path = path
        self._btn_record.setText("Stop recording")
        self._btn_record.setIcon(self._record_stop_icon)
        self._log_line(f"Recording to {path}")

    def _stop_recording(self):
        handle = self._recording_file
        path = self._recording_path
        self._recording_file = None
        self._recording_writer = None
        self._recording_path = None
        if handle is not None:
            try:
                handle.close()
            except OSError as exc:
                self._log_line(f"Failed to close recording: {exc}")
            else:
                if path is not None:
                    self._log_line(f"Recording saved: {path}")
        self._btn_record.blockSignals(True)
        self._btn_record.setChecked(False)
        self._btn_record.setText("Record")
        self._btn_record.setIcon(self._record_icon)
        self._btn_record.blockSignals(False)

    def _record_adc_sample(self, sample_idx: int, counts: tuple[int, int, int, int], values: list[float]):
        writer = self._recording_writer
        if writer is None:
            return
        writer.writerow([
            int(sample_idx),
            *[int(count) for count in counts],
            *[f"{value:.9g}" for value in values],
        ])

    def _append_plot_sample(self, buf: list[float], value: float):
        buf.append(float(value))
        if len(buf) > self._max_plot_samples:
            del buf[: len(buf) - self._max_plot_samples]

    def _window_sample_count(self) -> int:
        return max(1, int(round(self._window_seconds * self._sample_rate_hz)))

    def _trim_plot_buffers(self):
        for buf in (
            self._adc_sample_idx_buf,
            *self._adc_buffers,
        ):
            if len(buf) > self._max_plot_samples:
                del buf[: len(buf) - self._max_plot_samples]

    def _refresh_plots(self):
        if self._adc_ch0_buf and self._adc_sample_idx_buf:
            newest = self._adc_sample_idx_buf[-1]
            x_adc = [
                (idx - newest) / self._sample_rate_hz
                for idx in self._adc_sample_idx_buf
            ]
            for curve, buf in zip(self._adc_curves, self._adc_buffers):
                curve.setData(x_adc, buf)

    def _update_plot_visibility(self):
        if hasattr(self, "_adc_plots"):
            for idx, plot in enumerate(self._adc_plots):
                plot.setVisible(self._channel_visible[idx])

    def _open_settings(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Scope settings")
        layout = QtWidgets.QVBoxLayout(dialog)

        channels_group = QtWidgets.QGroupBox("Visible channels")
        channels_layout = QtWidgets.QGridLayout(channels_group)
        channel_checks: list[QtWidgets.QCheckBox] = []
        for idx, label in enumerate(ADC_CHANNEL_LABELS):
            check = QtWidgets.QCheckBox(label)
            check.setChecked(self._channel_visible[idx])
            channel_checks.append(check)
            channels_layout.addWidget(check, idx // 2, idx % 2)
        layout.addWidget(channels_group)

        timing_group = QtWidgets.QGroupBox("Acquisition")
        timing_layout = QtWidgets.QFormLayout(timing_group)

        sample_rate = QtWidgets.QComboBox()
        for label, rate_hz in ADS131M04_SAMPLE_RATES:
            sample_rate.addItem(label, rate_hz)
        sample_rate_idx = sample_rate.findData(self._sample_rate_hz)
        sample_rate.setCurrentIndex(max(0, sample_rate_idx))
        timing_layout.addRow("ADS131M04 rate", sample_rate)

        window_seconds = QtWidgets.QDoubleSpinBox()
        window_seconds.setRange(0.1, 120.0)
        window_seconds.setDecimals(1)
        window_seconds.setSingleStep(0.5)
        window_seconds.setSuffix(" s")
        window_seconds.setValue(self._window_seconds)
        timing_layout.addRow("Plot window", window_seconds)

        layout.addWidget(timing_group)

        filter_group = QtWidgets.QGroupBox("Display filter")
        filter_layout = QtWidgets.QFormLayout(filter_group)

        lowpass_enabled = QtWidgets.QCheckBox("Butterworth low-pass")
        lowpass_enabled.setChecked(self._lowpass_enabled)
        filter_layout.addRow(lowpass_enabled)

        notch_enabled = QtWidgets.QCheckBox("50 Hz notch")
        notch_enabled.setChecked(self._notch_enabled)
        filter_layout.addRow(notch_enabled)

        cutoff_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        cutoff_slider.setRange(LOWPASS_MIN_CUTOFF_HZ, self._max_lowpass_cutoff_hz())
        cutoff_slider.setValue(self._lowpass_cutoff_hz)
        cutoff_spin = QtWidgets.QSpinBox()
        cutoff_spin.setRange(LOWPASS_MIN_CUTOFF_HZ, self._max_lowpass_cutoff_hz())
        cutoff_spin.setSuffix(" Hz")
        cutoff_spin.setValue(self._lowpass_cutoff_hz)
        cutoff_slider.valueChanged.connect(cutoff_spin.setValue)
        cutoff_spin.valueChanged.connect(cutoff_slider.setValue)

        def update_cutoff_range():
            selected_rate_hz = int(sample_rate.currentData())
            max_cutoff_hz = self._max_lowpass_cutoff_hz(selected_rate_hz)
            cutoff_slider.setMaximum(max_cutoff_hz)
            cutoff_spin.setMaximum(max_cutoff_hz)
            if cutoff_spin.value() > max_cutoff_hz:
                cutoff_spin.setValue(max_cutoff_hz)

        sample_rate.currentIndexChanged.connect(lambda _idx: update_cutoff_range())
        update_cutoff_range()

        cutoff_controls = QtWidgets.QHBoxLayout()
        cutoff_controls.addWidget(cutoff_slider, 1)
        cutoff_controls.addWidget(cutoff_spin)
        filter_layout.addRow("Cutoff", cutoff_controls)

        layout.addWidget(filter_group)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok
            | QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return

        self._channel_visible = [check.isChecked() for check in channel_checks]
        self._window_seconds = float(window_seconds.value())
        new_sample_rate_hz = int(sample_rate.currentData())
        sample_rate_changed = self._sample_rate_hz != new_sample_rate_hz
        self._sample_rate_hz = new_sample_rate_hz
        lowpass_enabled_changed = self._lowpass_enabled != lowpass_enabled.isChecked()
        self._lowpass_enabled = lowpass_enabled.isChecked()
        notch_enabled_changed = self._notch_enabled != notch_enabled.isChecked()
        self._notch_enabled = notch_enabled.isChecked()
        new_cutoff_hz = int(cutoff_spin.value())
        if sample_rate_changed or lowpass_enabled_changed or new_cutoff_hz != self._lowpass_cutoff_hz:
            self._lowpass_cutoff_hz = new_cutoff_hz
            self._reset_lowpass_filters()
        if sample_rate_changed or notch_enabled_changed:
            self._reset_notch_filters()
        self._max_plot_samples = self._window_sample_count()
        self._trim_plot_buffers()
        self._update_plot_visibility()
        self._save_settings()
        if sample_rate_changed:
            self._clear_plot_buffers()
            self._send_ads131_sample_rate()

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

    def _get_selected_device(self) -> tuple[str, str] | None:
        idx = self._device_combo.currentIndex()
        if idx < 0 or idx >= len(self._scan_results):
            return None
        return self._scan_results[idx]

    def _set_device_list(self, devices: list[tuple[str, str | None, int | None]]):
        # devices: (address, name, rssi)
        self._scan_results = [(addr, name) for (addr, name, _rssi) in devices]
        self._device_combo.clear()
        teng_scope_idx: int | None = None
        for addr, name, rssi in devices:
            label_name = (name or "").strip() or "Unknown"
            rssi_txt = f", RSSI {rssi} dBm" if isinstance(rssi, int) else ""
            self._device_combo.addItem(f"{label_name} ({addr}{rssi_txt})")
            if label_name == "TENG_Scope":
                teng_scope_idx = self._device_combo.count() - 1
            elif teng_scope_idx is None and label_name.lower() == "teng_scope":
                teng_scope_idx = self._device_combo.count() - 1
        if teng_scope_idx is not None:
            self._device_combo.setCurrentIndex(teng_scope_idx)

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
            self._btn_record.setEnabled(False)
            return

        self._btn_stream.setEnabled(False)
        self._btn_scan.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_record.setEnabled(False)

        # Reset plots on new stream
        self._clear_plot_buffers()
        self._reset_lowpass_filters()
        self._reset_notch_filters()

        address, name = selected
        self._reader = BleStreamThread(address=address, name=name, parent=self)
        self._reader.statusMessage.connect(self._log_line)
        self._reader.connectionChanged.connect(self._on_stream_connection_changed)
        self._reader.deviceNameReady.connect(self._on_stream_device_name)
        self._reader.nusLogReady.connect(self._on_stream_nus_log)
        self._reader.adcFrameReady.connect(self._on_stream_adc_frame)
        self._reader.start()
        self._send_ads131_sample_rate()

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
        self._btn_record.setEnabled(False)
        self._set_connection_info_collapsed(False)
        self._stop_recording()

    @QtCore.Slot(bool)
    def _on_stream_connection_changed(self, connected: bool):
        self._streaming = bool(connected)
        if connected:
            self._set_ui(status="Streaming")
            self._btn_stream.setEnabled(False)
            self._btn_scan.setEnabled(False)
            self._btn_stop.setEnabled(True)
            self._btn_record.setEnabled(True)
            self._set_connection_info_collapsed(True)
        else:
            self._set_ui(status="Disconnected")
            self._btn_stream.setEnabled(True)
            self._btn_scan.setEnabled(True)
            self._btn_stop.setEnabled(False)
            self._btn_record.setEnabled(False)
            self._set_connection_info_collapsed(False)
            self._stop_recording()

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
    def _on_stream_adc_frame(self, payload: bytes):
        # adc_data_t is packed: uint32 sample_count + int32 ch0 + ch1 + ch2 + ch3
        last_values = None
        for (sample_idx, ch0, ch1, ch2, ch3) in iter_structs("<Iiiii", payload):
            counts = (ch0, ch1, ch2, ch3)
            values = [
                adc_counts_to_volts(0, ch0),
                adc_counts_to_volts(1, ch1),
                adc_counts_to_volts(2, ch2),
                adc_counts_to_volts(3, ch3),
            ]
            self._record_adc_sample(sample_idx, counts, values)
            last_values = values
            values = self._apply_lowpass(values)
            values = self._apply_notch(values)
            if self._chk_ac_coupling.isChecked():
                for idx, sample in enumerate(values):
                    self._adc_lp[idx] += self._ac_coupling_alpha * (sample - self._adc_lp[idx])
                    values[idx] = sample - self._adc_lp[idx]
            self._adc_sample_idx_buf.append(self._plot_sample_idx)
            self._plot_sample_idx += 1
            if len(self._adc_sample_idx_buf) > self._max_plot_samples:
                del self._adc_sample_idx_buf[: len(self._adc_sample_idx_buf) - self._max_plot_samples]
            for buf, value in zip(self._adc_buffers, values):
                self._append_plot_sample(buf, value)
        if self._recording_file is not None:
            self._recording_file.flush()
        if last_values is not None:
            self._set_ui(adc="  ".join(f"CH{idx}: {value:.4f} V" for idx, value in enumerate(last_values)))

    # BLE notification handlers are managed inside BleStreamThread


def main():
    qapp = QtWidgets.QApplication([])
    w = App()
    w.show()
    qapp.exec()
