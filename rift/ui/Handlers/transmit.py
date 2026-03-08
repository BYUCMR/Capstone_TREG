import time

import serial
from numpy import ndarray
from PySide6.QtCore import QObject, QThread, Signal, Slot

from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.transmit import commands


class TransmitHandler(QObject):
    message = Signal(str)
    bot_live = False

    def start_transmission(self, *, dt: float = 1.5) -> None:
        self.work_thread = QThread()
        self.worker = TransmitWorker(dt=dt)
        self.worker.moveToThread(self.work_thread)

        self.worker.message.connect(self.message.emit)
        self.work_thread.finished.connect(self.worker.deleteLater)
        self.worker.start()
        self.work_thread.start()

        self.bot_live = True

    def kill_transmission(self) -> None:
        self.worker.stop()
        self.work_thread.exit()
        self.bot_live = False


class TransmitWorker(QObject):
    message = Signal(str)

    def __init__(self, *, dt: float = 1.5, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.dt = dt
        self.ser = serial.Serial(baudrate=115200, timeout=dt+1)

    @Slot(ndarray, ndarray)
    def transmit(self, x: Matrix, dq: Vector) -> None:
        if not self.ser.is_open:
            return
        ticks_per_sec = rover.TICKS_PER_SIDE * dq / self.dt
        cmd = commands.VEL(map(int, ticks_per_sec))
        self.ser.writelines((commands.STOP, cmd))
        self.ser.flush()
        if responses := self.ser.read_all():
            for response in responses.splitlines():
                if not response.startswith(b'['):
                    self.message.emit(response.decode())
        time.sleep(self.dt)

    def start(self, port: str = '/dev/ttyUSB0') -> None:
        self.ser.port = port
        try:
            self.ser.open()
        except serial.SerialException as e:
            self.message.emit(e.args[0])

    def stop(self) -> None:
        self.ser.close()
