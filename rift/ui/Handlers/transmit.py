import time

import numpy as np
import serial
from numpy import ndarray
from PySide6.QtCore import QObject, QThread, Signal, Slot

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

    def start_transmission(self, *, port: str) -> None:
        self.work_thread = QThread()
        self.worker = TransmitWorker()
        self.worker.moveToThread(self.work_thread)

        self.worker.message.connect(self.message.emit)
        self.worker.update_sliders.connect(self.update_sliders.emit)
        self.stop.connect(self.worker.stop)
        self.set_zero.connect(self.worker.set_zero)
        self.to_zero.connect(self.worker.to_zero)
        self.catch_up.connect(self.worker.catch_up)
        self.work_thread.finished.connect(self.worker.deleteLater)
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

    def __init__(self, *, timeout: float = 0.01, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.ser = serial.Serial(baudrate=115200, timeout=timeout)
        self.commander = commander.Commander(
            ser=self.ser,
            q_des=np.zeros(12, dtype=np.intp),
            log=self.message.emit,
        )

    @Slot()
    def stop(self) -> None:
        if not self.ser.is_open:
            return
        self.commander.stop()

    @Slot()
    def set_zero(self) -> None:
        if not self.ser.is_open:
            return
        self.commander.set_zero()

    @Slot()
    def to_zero(self) -> None:
        if not self.ser.is_open:
            return
        self.commander.to_zero()
        time.sleep(3.0)
        error = self.commander.get_error()
        if error is not None:
            self.update_sliders.emit(error)

    @Slot()
    def catch_up(self) -> None:
        self.commander.catch_up()
        time.sleep(3.0)
        error = self.commander.get_error()
        if error is not None:
            self.update_sliders.emit(error)

    @Slot(ndarray, ndarray)
    def transmit(self, x: Matrix, dq: Vector) -> None:
        if not self.ser.is_open:
            return
        error = self.commander.get_error()
        if error is not None:
            self.update_sliders.emit(error)
        ticks = (rover.TICKS_PER_SIDE * dq).astype(np.intp)
        self.commander.update(ticks)
        if error is not None:
            ticks += error
        dt = commands.get_smallest_dt(ticks, max_speed=1000)
        self.commander.send_dq(ticks, dt)

    def start(self, port: str) -> None:
        self.ser.port = port
        try:
            self.ser.open()
        except serial.SerialException as e:
            self.message.emit(e.args[0])

    def close(self) -> None:
        self.ser.close()
