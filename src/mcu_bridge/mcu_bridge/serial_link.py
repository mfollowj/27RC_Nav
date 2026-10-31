"""串口链路封装：打开 / 重连 / 写，失败不把异常抛进节点主循环。

为什么不用 pyserial 的 write_timeout 长阻塞：串口写超时会卡住定时器回调，
导致其它 ROS 回调（比如 /goal_pose）跟着延迟。这里用很短的 write_timeout，
写失败就丢弃这一帧并重连，下一帧继续。
"""

import time


class SerialLink:
    """一个很薄的 pyserial 包装，dry_run 时不真正打开串口。"""

    def __init__(self, port, baudrate=115200, reconnect_interval=1.0,
                 write_timeout=0.05, dry_run=False, logger=None):
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.reconnect_interval = max(0.0, float(reconnect_interval))
        self.write_timeout = float(write_timeout)
        self.dry_run = bool(dry_run)
        self.logger = logger

        self._serial = None
        self._last_attempt = 0.0
        self.frames_sent = 0
        self.write_errors = 0
        self.open_errors = 0
        self.dry_run_frames = 0

        if not self.dry_run:
            self.open()

    # ------------------------------------------------------------------ 内部
    # rclpy 的 logger 不允许"同一个调用点换 severity"（会抛
    # ValueError: Logger severity cannot be changed between calls），
    # 所以这里按级别分开写，不要用 getattr(logger, level) 动态分发。
    def _log_info(self, msg):
        if self.logger is not None:
            self.logger.info(msg)

    def _log_error(self, msg):
        if self.logger is not None:
            self.logger.error(msg)

    @property
    def is_open(self):
        return self.dry_run or (self._serial is not None and self._serial.is_open)

    def open(self):
        if self.dry_run:
            return True
        if self._serial is not None and self._serial.is_open:
            return True
        now = time.monotonic()
        if now - self._last_attempt < self.reconnect_interval:
            return False
        self._last_attempt = now
        try:
            import serial  # 延迟 import：dry_run / 单测不需要 pyserial
        except ImportError:
            self.open_errors += 1
            self._log_error(
                      "pyserial 未安装：sudo apt install python3-serial")
            return False
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=self.write_timeout,
            )
            self._log_info("串口已打开：%s @ %d 8N1"
                       % (self.port, self.baudrate))
            return True
        except Exception as exc:            # noqa: BLE001 - 串口错误类型很多
            self._serial = None
            self.open_errors += 1
            self._log_error("打不开串口 %s：%s" % (self.port, exc))
            return False

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:               # noqa: BLE001 - best effort
                pass
            self._serial = None

    # ------------------------------------------------------------------ 写
    def write(self, frame):
        """写一帧；返回是否成功。失败会自动进入重连流程。"""
        if self.dry_run:
            self.dry_run_frames += 1
            self.frames_sent += 1
            if self.dry_run_frames == 1:
                self._log_info("dry_run：第一帧 %s" % frame.hex(" ").upper())
            return True

        if self._serial is None or not self._serial.is_open:
            if not self.open():
                return False
        try:
            self._serial.write(frame)
            self.frames_sent += 1
            return True
        except Exception as exc:            # noqa: BLE001
            self.write_errors += 1
            self._log_error("串口写失败（%s），重连" % exc)
            self.close()
            return False

    def stats(self):
        return ("frames=%d dry_run=%d write_errors=%d open_errors=%d"
                % (self.frames_sent, self.dry_run_frames,
                   self.write_errors, self.open_errors))
