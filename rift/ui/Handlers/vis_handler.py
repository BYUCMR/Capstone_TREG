import time
from collections.abc import Generator

import numpy as np
from numpy import ndarray
from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot
# from PySide6.QtWidgets import QWidget

from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.robot import InverseKinematicsError
from rift.steps import Command
# from .ui_vis import Ui_vis_window

class SimWindow(QObject): #referenced as sim_widget by mainwindow class
    send_cmd = Signal(Command)
    query_next = Signal()
    send_startup = Signal()
    message = Signal(str)

    def __init__(
        self,
        cmd_state: Command,
        ui,
        parent: QObject | None = None
    ) -> None:
        super().__init__()

        self.ui = ui

        self.cmd_state = cmd_state

        self.view_live = False

        view, self.animate = rover.set_up_animation(trace_len=10)
        self.ui.ctr_layout.insertWidget(0,view)
        view.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    @Slot(ndarray, ndarray, ndarray)
    def update_anim(self, x: Matrix, dx: Matrix, dq: Vector) -> None:
        self.animate(x)
        self.query_next.emit()

    @Slot()
    def send_new(self):
        self.send_cmd.emit(self.cmd_state)

    def start_sim(self) -> None:
        # self.show()
        # print('yuh')

        self.work_thread = QThread()
        self.worker = VizWorker()
        self.worker.period = 10
        self.worker.moveToThread(self.work_thread)

        self.send_startup.connect(self.worker.setup)
        self.send_startup.emit()
        self.send_cmd.connect(self.worker.run_cmd)
        self.query_next.connect(self.worker.run_next)
        self.worker.ready.connect(self.send_new)
        self.worker.anim_update.connect(self.update_anim)
        self.worker.message.connect(self.message.emit)
        self.work_thread.finished.connect(self.worker.deleteLater)
        self.send_new()
        self.work_thread.start()

        self.view_live = True

    def kill_sim(self) -> None:
        # print('nuh')
        # self.hide()
        self.work_thread.requestInterruption()
        self.view_live = False


class VizWorker(QObject):
    ready = Signal()
    anim_update = Signal(ndarray, ndarray, ndarray)
    message = Signal(str)

    period: int = 1
    resolution: int = 100
    gen: Generator[tuple[Matrix, Vector]] | None = None

    @Slot()
    def setup(self) -> None:
        self.robot = rover.make_robot()

    @Slot(Command)
    def run_cmd(self, cmd: Command) -> None:
        cur_thread = QThread.currentThread()
        if cur_thread.isInterruptionRequested():
            cur_thread.exit()
            return
        self.gen = rover.take_command(self.robot, cmd, resolution=self.resolution)
        self.run_next()

    @Slot()
    def run_next(self) -> None:
        if self.gen is None:
            self.ready.emit()
            return
        delta_x = np.zeros_like(self.robot.pos)
        delta_q = np.zeros(len(self.robot.control.inverse))
        for _ in range(self.period):
            try:
                dx, dq = next(self.gen)
            except StopIteration:
                self.ready.emit()
                break
            except InverseKinematicsError as e:
                self.message.emit(e.args[0])
                self.ready.emit()
                break
            delta_x += dx
            delta_q += dq
        else:
            time.sleep(0.001)
            self.anim_update.emit(self.robot.pos.copy(), delta_x, delta_q)
