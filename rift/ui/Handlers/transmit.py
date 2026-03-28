import numpy as np
import serial
from numpy import ndarray
from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot
from PySide6.QtWidgets import QApplication

from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.transmit import commander, commands


class TransmitHandler(QObject):
    message = Signal(str)
    update_sliders = Signal(np.ndarray)
    stop = Signal()
    set_zero = Signal()
    to_zero = Signal()
    catch_up = Signal()
    bot_live = False

    def __init__(self, parent: QObject | None = None, *, objectName: str | None = None) -> None:
        super().__init__(parent, objectName=objectName)
        self.worker = TransmitWorker()
        self.worker.message.connect(self.message.emit)
        self.worker.update_sliders.connect(self.update_sliders.emit)
        self.stop.connect(self.worker.stop)
        self.set_zero.connect(self.worker.set_zero)
        self.to_zero.connect(self.worker.to_zero)
        self.catch_up.connect(self.worker.catch_up)

    def start_transmission(self, *, port: str) -> None:
        self.work_thread = QThread()
        self.worker.moveToThread(self.work_thread)
        self.work_thread.finished.connect(self.worker.pull_to_main)
        self.worker.start(port)
        self.work_thread.start()
        self.bot_live = True

    def kill_transmission(self) -> None:
        self.worker.close()
        self.work_thread.exit()
        self.bot_live = False


class TransmitWorker(QObject):
    message = Signal(str)
    update_sliders = Signal(np.ndarray)
    ready = Signal()

    def __init__(
        self,
        *,
        timeout: float = 0.01,
        parent: QObject | None = None,
        feedback: bool = False,
    ) -> None:
        super().__init__(parent)
        self.ser = serial.Serial(baudrate=115200, timeout=timeout)
        self.q_des = np.zeros(12, dtype=np.intp)
        self.commander = commander.Commander(self.ser, self.message.emit)
        self.feedback = feedback
        self.done_timer = QTimer(self)
        self.update_error_timer = QTimer(self, interval=3000, singleShot=True)
        self.update_error_timer.timeout.connect(self.update_error)

    @Slot()
    def stop(self) -> None:
        self.done_timer.stop()
        if not self.ser.is_open:
            return
        self.commander.send_stop()
        self.update_error_timer.start()

    @Slot()
    def update_error(self) -> None:
        q_cur = self.commander.get_q()
        if q_cur is not None:
            self.update_sliders.emit(q_cur - self.q_des)

    @Slot()
    def set_zero(self) -> None:
        self.q_des[:] = 0
        if not self.ser.is_open:
            return
        self.commander.set_zero()
        self.update_error_timer.start()

    @Slot()
    def to_zero(self) -> None:
        self.q_des[:] = 0
        if not self.ser.is_open:
            return
        self.commander.to_zero()
        self.update_error_timer.start()

    @Slot()
    def catch_up(self) -> None:
        if not self.ser.is_open:
            return
        self.commander.send_stop()
        self.commander.send_q(self.q_des)
        self.update_error_timer.start()

    @Slot(ndarray, ndarray)
    def transmit(self, x: Matrix, dq: Vector) -> None:
        if not self.ser.is_open:
            self.ready.emit()
            return
        q_cur = self.commander.get_q()
        if q_cur is not None:
            self.update_sliders.emit(q_cur - self.q_des)
        ticks = (rover.TICKS_PER_SIDE * dq).astype(np.intp)
        self.q_des += ticks
        if self.feedback and q_cur is not None:
            ticks = self.q_des - q_cur
        dt = commands.get_smallest_dt(ticks, max_speed=1000)
        self.commander.send_dq(ticks, dt)
        self.done_timer.singleShot(int(dt * 1000), self.ready.emit)

    def start(self, port: str) -> None:
        self.ser.port = port
        try:
            self.ser.open()
        except serial.SerialException as e:
            self.message.emit(e.args[0])

    def close(self) -> None:
        self.ser.close()

    @Slot()
    def pull_to_main(self) -> None:
        app = QApplication.instance()
        if app is not None:
            self.moveToThread(app.thread())
