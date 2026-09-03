"""
PS2 手柄 GPIO 模拟 SPI 底层库。

提供 PS2Controller（硬件读写）与 PS2Receiver（后台采样、无效帧保护）。
业务按键映射见 ps2_control.py。

作者 王笑
日期 20260701
"""

from machine import Pin
import time

try:
    import _thread
except ImportError:
    _thread = None


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def sleep_us(us):
    if hasattr(time, "sleep_us"):
        time.sleep_us(us)
    else:
        time.sleep(us / 1000000.0)


class PS2Controller:
    def __init__(self, di, do, cs, clk):
        self.di = Pin(di, Pin.IN)
        self.do = Pin(do, Pin.OUT)
        self.cs = Pin(cs, Pin.OUT)
        self.clk = Pin(clk, Pin.OUT)

        self.cs.value(1)
        self.clk.value(1)

        self.rumble_small = 0x00
        self.rumble_large = 0x00

        self.PS2_BTN_SELECT = 0x0001
        self.PS2_BTN_L3 = 0x0002
        self.PS2_BTN_R3 = 0x0004
        self.PS2_BTN_START = 0x0008
        self.PS2_BTN_UP = 0x0010
        self.PS2_BTN_RIGHT = 0x0020
        self.PS2_BTN_DOWN = 0x0040
        self.PS2_BTN_LEFT = 0x0080
        self.PS2_BTN_L2 = 0x0100
        self.PS2_BTN_R2 = 0x0200
        self.PS2_BTN_L1 = 0x0400
        self.PS2_BTN_R1 = 0x0800
        self.PS2_BTN_TRIANGLE = 0x1000
        self.PS2_BTN_CIRCLE = 0x2000
        self.PS2_BTN_CROSS = 0x4000
        self.PS2_BTN_SQUARE = 0x8000

        self.scan = [0x00] * 9
        self.data = 0

    def _transfer(self, byte):
        res = 0
        for i in range(8):
            self.do.value(1 if byte & (1 << i) else 0)
            self.clk.value(0)
            sleep_us(10)
            if self.di.value():
                res |= (1 << i)
            self.clk.value(1)
            sleep_us(10)
        return res

    def _send_command(self, command_bytes):
        self.cs.value(0)
        sleep_us(20)
        for byte in command_bytes:
            self._transfer(byte)
            sleep_us(10)
        self.cs.value(1)
        sleep_ms(10)

    def init_vibration(self):
        print("正在配置震动模式...")
        self._send_command([0x01, 0x43, 0x00, 0x01, 0x00])
        self._send_command([0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00])
        self._send_command([0x01, 0x4D, 0x00, 0x00, 0x01])
        self._send_command([0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A])
        print("震动配置完成")

    def set_rumble(self, small, large):
        self.rumble_small = 0xFF if small else 0x00
        self.rumble_large = int(large)
        if self.rumble_large > 255:
            self.rumble_large = 255

    def update(self):
        self.cs.value(0)
        sleep_us(20)

        self._transfer(0x01)
        self.scan[1] = self._transfer(0x42)
        self.scan[2] = self._transfer(0x00)
        self.scan[3] = self._transfer(self.rumble_small)
        self.scan[4] = self._transfer(self.rumble_large)
        self.scan[5] = self._transfer(0x00)
        self.scan[6] = self._transfer(0x00)
        self.scan[7] = self._transfer(0x00)
        self.scan[8] = self._transfer(0x00)

        self.cs.value(1)
        self.data = (~((self.scan[4] << 8) | self.scan[3])) & 0xFFFF

    def button(self, btn):
        return (self.data & btn) == btn

    def analog_x(self, stick="left"):
        return self.scan[7] if stick == "left" else self.scan[5]

    def analog_y(self, stick="left"):
        return self.scan[8] if stick == "left" else self.scan[6]


class PS2Receiver:
    """安全读取 PS2，主循环只使用最近一次按键/摇杆快照。"""

    def __init__(self, controller, interval_ms=20, use_thread=False):
        self.controller = controller
        self.interval_ms = int(interval_ms)
        self.use_thread = bool(use_thread)
        self._lock = _thread.allocate_lock() if _thread is not None else None
        self._running = False
        self._thread_started = False
        self._data = 0
        self._left_x = 128
        self._left_y = 128
        self._right_x = 128
        self._right_y = 128
        self._last_update_ms = 0
        self._valid = False

    def __getattr__(self, name):
        return getattr(self.controller, name)

    def start(self):
        self.update()
        if not self.use_thread:
            print("PS2 使用主循环读取，并启用无效帧停车保护。")
            return False
        if _thread is None:
            print("PS2 后台线程不可用，使用主循环读取。")
            return False
        self._running = True
        try:
            _thread.start_new_thread(self._thread_loop, ())
            self._thread_started = True
            print("PS2 后台接收线程已启动。")
            return True
        except Exception:
            self._running = False
            self._thread_started = False
            print("PS2 后台线程启动失败，使用主循环读取。")
            return False

    def stop(self):
        self._running = False
        self._thread_started = False

    def update(self):
        if self._thread_started:
            return self.is_fresh()
        try:
            self.controller.update()
            return self._store_snapshot()
        except Exception:
            return False

    def button(self, btn):
        return (self._read_data() & btn) == btn

    def analog_x(self, stick="left"):
        if self._lock is not None:
            self._lock.acquire()
        try:
            return self._left_x if stick == "left" else self._right_x
        finally:
            if self._lock is not None:
                self._lock.release()

    def analog_y(self, stick="left"):
        if self._lock is not None:
            self._lock.acquire()
        try:
            return self._left_y if stick == "left" else self._right_y
        finally:
            if self._lock is not None:
                self._lock.release()

    def snapshot(self, max_age_ms=250):
        if self._lock is not None:
            self._lock.acquire()
        try:
            age_ms = ticks_diff(ticks_ms(), self._last_update_ms)
            fresh = self._valid and age_ms <= int(max_age_ms)
            return (
                fresh,
                self._data,
                self._left_x,
                self._left_y,
                self._right_x,
                self._right_y,
                age_ms,
            )
        finally:
            if self._lock is not None:
                self._lock.release()

    def _thread_loop(self):
        while self._running:
            try:
                self.controller.update()
                self._store_snapshot()
            except Exception:
                pass
            sleep_ms(self.interval_ms)

    def _store_snapshot(self):
        mode = self.controller.scan[1]
        marker = self.controller.scan[2]
        if not self._is_valid_frame(mode, marker):
            return False

        data = self.controller.data
        left_x = self.controller.analog_x("left")
        left_y = self.controller.analog_y("left")
        right_x = self.controller.analog_x("right")
        right_y = self.controller.analog_y("right")
        now = ticks_ms()

        if self._lock is not None:
            self._lock.acquire()
        try:
            self._data = data
            self._left_x = left_x
            self._left_y = left_y
            self._right_x = right_x
            self._right_y = right_y
            self._last_update_ms = now
            self._valid = True
        finally:
            if self._lock is not None:
                self._lock.release()
        return True

    def is_fresh(self, max_age_ms=250):
        if self._lock is not None:
            self._lock.acquire()
        try:
            if not self._valid:
                return False
            age_ms = ticks_diff(ticks_ms(), self._last_update_ms)
            return age_ms <= int(max_age_ms)
        finally:
            if self._lock is not None:
                self._lock.release()

    def _read_data(self):
        if self._lock is not None:
            self._lock.acquire()
        try:
            return self._data
        finally:
            if self._lock is not None:
                self._lock.release()

    @staticmethod
    def _is_valid_frame(mode, marker):
        return marker == 0x5A and mode in (0x41, 0x73, 0x79)
