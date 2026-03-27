import asyncio
import struct
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk

from bleak import BleakClient, BleakScanner


DEVICE_NAME = "ECG_Belt"

# Custom service UUIDs (see nrf54/src/bluetooth/bluetooth.c)
UUID_ECG_SERVICE = "12345678-1234-5678-1234-56789abcdef0"
UUID_ECG_DATA = "12345678-1234-5678-1234-56789abcdef1"      # ecg_data_t batches (timestamp + ecg_aux)
UUID_IMU_DATA = "12345678-1234-5678-1234-56789abcdef2"      # imu_data_t batches
UUID_ECG_RAW_DATA = "12345678-1234-5678-1234-56789abcdef3"  # ecg_raw_data_t batches (timestamp + heart)

# Standard Heart Rate Measurement characteristic UUID (0x2A37)
UUID_HR_MEASUREMENT = "00002a37-0000-1000-8000-00805f9b34fb"


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


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ECG Belt GUI (BLE)")
        self.geometry("520x300")

        self.state = UiState()
        self._loop = asyncio.new_event_loop()
        self._client: BleakClient | None = None

        self._ui_queue: "queue.Queue[callable]" | None = None

        # UI
        self._status = tk.StringVar(value="Disconnected")
        self._device = tk.StringVar(value="-")
        self._bpm = tk.StringVar(value="-")
        self._rr = tk.StringVar(value="-")
        self._ecg = tk.StringVar(value="-")
        self._imu = tk.StringVar(value="-")

        frm = ttk.Frame(self, padding=12)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._status).grid(row=0, column=1, sticky="w")

        ttk.Label(frm, text="Device:").grid(row=1, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._device).grid(row=1, column=1, sticky="w")

        ttk.Separator(frm).grid(row=2, column=0, columnspan=2, sticky="ew", pady=10)

        ttk.Label(frm, text="Heart rate (BPM):").grid(row=3, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._bpm).grid(row=3, column=1, sticky="w")

        ttk.Label(frm, text="RR interval (ms):").grid(row=4, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._rr).grid(row=4, column=1, sticky="w")

        ttk.Label(frm, text="ECG last sample:").grid(row=5, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._ecg).grid(row=5, column=1, sticky="w")

        ttk.Label(frm, text="IMU last (ax,ay,az,gx,gy,gz):").grid(row=6, column=0, sticky="w")
        ttk.Label(frm, textvariable=self._imu).grid(row=6, column=1, sticky="w")

        ttk.Separator(frm).grid(row=7, column=0, columnspan=2, sticky="ew", pady=10)

        btns = ttk.Frame(frm)
        btns.grid(row=8, column=0, columnspan=2, sticky="w")
        ttk.Button(btns, text="Scan & connect", command=self._on_connect).pack(side="left")
        ttk.Button(btns, text="Disconnect", command=self._on_disconnect).pack(side="left", padx=8)

        for i in range(2):
            frm.columnconfigure(i, weight=1)

        # Start asyncio loop in a background thread
        t = threading.Thread(target=self._run_loop, daemon=True)
        t.start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _set_ui(self, *, status=None, device=None, bpm=None, rr=None, ecg=None, imu=None):
        if status is not None:
            self._status.set(status)
        if device is not None:
            self._device.set(device)
        if bpm is not None:
            self._bpm.set(bpm)
        if rr is not None:
            self._rr.set(rr)
        if ecg is not None:
            self._ecg.set(ecg)
        if imu is not None:
            self._imu.set(imu)

    def _on_connect(self):
        self._set_ui(status="Scanning...")
        asyncio.run_coroutine_threadsafe(self._connect_task(), self._loop)

    def _on_disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect_task(), self._loop)

    def _on_close(self):
        try:
            asyncio.run_coroutine_threadsafe(self._disconnect_task(), self._loop).result(timeout=2)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self.destroy()

    async def _connect_task(self):
        try:
            devices = await BleakScanner.discover(timeout=5.0)
            target = None
            for d in devices:
                if (d.name or "").strip() == DEVICE_NAME:
                    target = d
                    break
            if target is None:
                self.after(0, lambda: self._set_ui(status=f"Not found: {DEVICE_NAME}", device="-"))
                return

            client = BleakClient(target)
            await client.connect()
            self._client = client

            self.after(0, lambda: self._set_ui(status="Connected", device=f"{target.name} ({target.address})"))

            # Notifications
            await client.start_notify(UUID_HR_MEASUREMENT, self._on_hrm)
            await client.start_notify(UUID_ECG_RAW_DATA, self._on_ecg_raw)
            await client.start_notify(UUID_IMU_DATA, self._on_imu)
        except Exception as e:
            self.after(0, lambda: self._set_ui(status=f"Connect failed: {e}", device="-"))

    async def _disconnect_task(self):
        if self._client is None:
            self.after(0, lambda: self._set_ui(status="Disconnected", device="-"))
            return
        try:
            c = self._client
            self._client = None
            try:
                await c.disconnect()
            finally:
                self.after(0, lambda: self._set_ui(status="Disconnected", device="-"))
        except Exception:
            self.after(0, lambda: self._set_ui(status="Disconnected", device="-"))

    def _on_hrm(self, _char, data: bytearray):
        bpm, rr = parse_hrm(bytes(data))
        if bpm is not None:
            self.after(0, lambda: self._set_ui(bpm=str(bpm)))
        if rr is not None:
            self.after(0, lambda: self._set_ui(rr=str(rr)))

    def _on_ecg_raw(self, _char, data: bytearray):
        payload = bytes(data)
        # ecg_raw_data_t is packed: uint32 sample_count + int32 heart
        # Notifications are batches of these structs.
        last = None
        for (_ts, heart) in iter_structs("<Ii", payload):
            last = int(heart)
        if last is not None:
            self.after(0, lambda: self._set_ui(ecg=str(last)))

    def _on_imu(self, _char, data: bytearray):
        payload = bytes(data)
        # imu_data_t is packed: uint32 sample_count + 6x int16
        last = None
        for (_ts, ax, ay, az, gx, gy, gz) in iter_structs("<Ihhhhhh", payload):
            last = (int(ax), int(ay), int(az), int(gx), int(gy), int(gz))
        if last is not None:
            s = f"{last[0]}, {last[1]}, {last[2]}, {last[3]}, {last[4]}, {last[5]}"
            self.after(0, lambda: self._set_ui(imu=s))


def main():
    App().mainloop()

