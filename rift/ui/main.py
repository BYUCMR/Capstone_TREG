from datetime import datetime

import numpy as np
import serial.tools.list_ports
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QCloseEvent, QKeyEvent
from PySide6.QtWidgets import QMainWindow, QWidget

from rift import rover
from rift.arraytypes import Vector
from .ui_main import Ui_Control
from .Handlers.controls import Command, Mode
from .Handlers.joystick_handler import JoystickHandler
from .Handlers.transmit import TransmitHandler
from .Handlers.vis_handler import SimWindow


class MainWindow(QMainWindow): #referenced as widget by sim window class
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.ui = Ui_Control()
        self.ui.setupUi(self)
        self.term_log("Welcome to R.I.F.T. Control!")

        self.cmd_state = Command(Mode.offline, 0, 0, 0, 0)

        self.joystick_handler = JoystickHandler(self.ui)
        self.vis_handler = SimWindow(self.cmd_state, self.ui)
        self.bot_handler = TransmitHandler()
        self.joystick_handler.message.connect(self.term_log)
        self.joystick_handler.command.connect(self.cmd_set)
        self.vis_handler.message.connect(self.term_log)
        self.bot_handler.message.connect(self.term_log)
        self.bot_handler.update_sliders.connect(self.update_motor_sliders)

        self.ui.selector_label.setVisible(False)
        self.ui.selector.setVisible(False)

        self.setup_serial_ports()
        self.setup_sliders()

        self.ui.sim_toggle.clicked.connect(self.toggle_sim)
        # self.ui.sim_label.clicked.connect(self.open_sim)
        self.ui.bot_toggle.clicked.connect(self.toggle_bot)
        self.ui.selector.valueChanged.connect(self.update_item)
        # self.ui.serial_select.currentIndexChanged.connect(self.serial_port_change)
        self.ui.zero_pos.clicked.connect(self.zero_pos)
        self.ui.reset_button.clicked.connect(self.reset_pos)
        self.ui.eq_all.clicked.connect(self.bot_handler.catch_up)

        self.ui.forward.pressed.connect(lambda: self.cmd_update(1, 0, 0))
        # self.ui.forward.pressed.connect(self.cleanup)
        self.ui.backward.pressed.connect(lambda: self.cmd_update(-1, 0, 0))
        self.ui.left.pressed.connect(lambda: self.cmd_update(0, -1, 0))
        self.ui.right.pressed.connect(lambda: self.cmd_update(0, 1, 0))
        self.ui.del_right.pressed.connect(lambda: self.cmd_update(0, 0, 1))
        self.ui.del_left.pressed.connect(lambda: self.cmd_update(0, 0, -1))

        self.ui.forward.released.connect(lambda: self.cmd_update(-1, 0, 0))
        self.ui.backward.released.connect(lambda: self.cmd_update(1, 0, 0))
        self.ui.left.released.connect(lambda: self.cmd_update(0, 1, 0))
        self.ui.right.released.connect(lambda: self.cmd_update(0, -1, 0))
        self.ui.del_right.released.connect(lambda: self.cmd_update(0, 0, -1))
        self.ui.del_left.released.connect(lambda: self.cmd_update(0, 0, 1))

        self.ui.crawling.clicked.connect(lambda: self.mode_select(Mode.crawling))
        self.ui.node_control.clicked.connect(lambda: self.mode_select(Mode.node_control))
        self.ui.calibration.clicked.connect(lambda: self.mode_select(Mode.calibration))
        self.ui.sit_stand.clicked.connect(lambda: self.mode_select(Mode.stand))

        self.ui.sim_toggle.clicked.emit()

    @Slot()
    def toggle_sim(self) -> None:
        if not self.vis_handler.sim_live:
            self.greenify(self.ui.sim_label)
            self.ui.sim_label.setText("Click to Open")
            self.ui.sim_toggle.setText("Kill Simulation")
            self.term_log("Simulation Initialized")
            self.vis_handler.start_sim()
            if self.bot_handler.bot_live:
                self.vis_handler.worker.results.connect(
                    self.bot_handler.worker.transmit,
                    Qt.ConnectionType.BlockingQueuedConnection,
                )
        else:
            self.redify(self.ui.sim_label)
            self.ui.sim_label.setText("Simulation Offline")
            self.ui.sim_toggle.setText("Begin Simulation")
            self.term_log("Simulation Closed")
            self.vis_handler.kill_sim()

    @Slot()
    def toggle_bot(self) -> None:
        if not self.bot_handler.bot_live:
            self.greenify(self.ui.bot_label)
            self.ui.bot_label.setText("Robot Online")
            self.ui.bot_toggle.setText("Disconnect Robot")
            self.term_log("Robot Connected")
            self.bot_handler.start_transmission(
                port=self.ui.serial_select.currentText(),
            )
            if self.vis_handler.sim_live:
                self.vis_handler.worker.results.connect(
                    self.bot_handler.worker.transmit,
                    Qt.ConnectionType.BlockingQueuedConnection,
                )
        else:
            self.redify(self.ui.bot_label)
            self.ui.bot_label.setText("Robot Offline")
            self.ui.bot_toggle.setText("Connect Robot")
            self.term_log("Robot Disconnected")
            self.bot_handler.kill_transmission()

    @Slot()
    def update_item(self) -> None:
        self.cmd_state.item = self.ui.selector.value()

    @Slot()
    def zero_pos(self) -> None:
        self.bot_handler.set_zero.emit()
        self.vis_handler.reset.emit(rover.ROLLING_POS)

    @Slot()
    def reset_pos(self) -> None:
        self.bot_handler.to_zero.emit()
        self.vis_handler.reset.emit(rover.ROLLING_POS)

    def setup_sliders(self) -> None:
        self.ui.mb_1.clicked.connect(lambda: self.correct_motor_error(1))
        self.ui.mb_2.clicked.connect(lambda: self.correct_motor_error(2))
        self.ui.mb_3.clicked.connect(lambda: self.correct_motor_error(3))
        self.ui.mb_4.clicked.connect(lambda: self.correct_motor_error(4))
        self.ui.mb_5.clicked.connect(lambda: self.correct_motor_error(5))
        self.ui.mb_6.clicked.connect(lambda: self.correct_motor_error(6))
        self.ui.mb_7.clicked.connect(lambda: self.correct_motor_error(7))
        self.ui.mb_8.clicked.connect(lambda: self.correct_motor_error(8))
        self.ui.mb_9.clicked.connect(lambda: self.correct_motor_error(9))
        self.ui.mb_10.clicked.connect(lambda: self.correct_motor_error(10))
        self.ui.mb_11.clicked.connect(lambda: self.correct_motor_error(11))
        self.ui.mb_12.clicked.connect(lambda: self.correct_motor_error(12))
        self.ui.eq_all.clicked.connect(lambda: self.correct_motor_error(0))

    @Slot()
    def correct_motor_error(self, motor) -> None:
        print("Correcting Motor Number: ",motor)
        #Code to correct a motor's position, or all of the motors if EQ all (motor=0) is clicked

    @Slot(np.ndarray)
    def update_motor_sliders(self, values: Vector[np.intp]) -> None:
        sliders = [
            self.ui.ms_1,
            self.ui.ms_2,
            self.ui.ms_3,
            self.ui.ms_4,
            self.ui.ms_5,
            self.ui.ms_6,
            self.ui.ms_7,
            self.ui.ms_8,
            self.ui.ms_9,
            self.ui.ms_10,
            self.ui.ms_11,
            self.ui.ms_12,
        ]
        for slider, value in zip(sliders, values, strict=True):
            slider.setValue(int(value))

    def setup_serial_ports(self) -> None:
        for port in serial.tools.list_ports.comports():
            self.ui.serial_select.addItem(port.device)

    # def serial_port_change(self, index) -> None:
    #     print("Serial Ports Changing, yaaaaa")

    def mode_select(self, mode: Mode) -> None:
        self.ui.left.setEnabled(True)
        self.ui.del_left.setEnabled(True)
        self.ui.right.setEnabled(True)
        self.ui.del_right.setEnabled(True)

        if mode is Mode.crawling:
            self.plainify_modes()
            self.greenify(self.ui.crawling)
            self.cmd_state.mode = Mode.crawling
            self.cmd_state.item = 0
            self.ui.selector_label.setVisible(False)
            self.ui.selector.setVisible(False)
        elif mode is Mode.node_control:
            self.plainify_modes()
            self.greenify(self.ui.node_control)
            self.cmd_state.mode = Mode.node_control
            self.cmd_state.item = self.ui.selector.value()
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Node")
            self.ui.selector.setVisible(True)
        elif mode is Mode.calibration:
            self.plainify_modes()
            self.greenify(self.ui.calibration)
            self.cmd_state.mode = Mode.calibration
            self.cmd_state.item = self.ui.selector.value()
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Roller")
            self.ui.selector.setVisible(True)
            self.ui.left.setEnabled(False)
            self.ui.del_left.setEnabled(False)
            self.ui.right.setEnabled(False)
            self.ui.del_right.setEnabled(False)
        elif mode is Mode.stand:
            self.plainify_modes()
            self.greenify(self.ui.sit_stand)
            self.cmd_state.mode = Mode.stand
            self.ui.selector_label.setVisible(False)
            self.ui.selector.setVisible(False)
            self.ui.forward.setEnabled(False)
            self.ui.backward.setEnabled(False)
            self.ui.left.setEnabled(False)
            self.ui.right.setEnabled(False)
        self.term_log(f"Control Mode switched to {mode.value.replace('_', ' ')}")

    def cmd_update(self, x: float, y: float, z: float) -> None:
        self.cmd_state.x += x
        self.cmd_state.y += y
        self.cmd_state.z += z
        # print(f"X: {self.cmd_state.x}, Y: {self.cmd_state.y}, Z: {self.cmd_state.z}")

    def cmd_set(self, x: float, y: float, z: float) -> None:
        self.cmd_state.x = x
        self.cmd_state.y = y
        self.cmd_state.z = z
        print(f"X: {self.cmd_state.x}, Y: {self.cmd_state.y}, Z: {self.cmd_state.z}")

    def cleanup(self) -> None:
        print("Attempting Cleanup")
        try:
            self.joystick_handler.js_thread.quit()
            self.joystick_handler.js_worker.deleteLater()
            print("Joystick hath been murked")
        except:
            print("No Joystick to kill")
        try:
            self.vis_handler.work_thread.requestInterruption()
            print("Sim killed")
        except:
            print("No Sim to kill")
        try:
            self.bot_handler.work_thread.exit()
            print("Robot disconnected")
        except:
            print("No Robot to disconnect")

    #overwriting key input handlers
    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            self.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_S:
            self.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_D:
            self.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_W:
            self.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_E:
            self.cmd_update(0, 0, 1)
        elif key == Qt.Key.Key_Q:
            self.cmd_update(0, 0, -1)
        event.accept()

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            self.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_S:
            self.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_D:
            self.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_W:
            self.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_E:
            self.cmd_update(0, 0, -1)
        elif key == Qt.Key.Key_Q:
            self.cmd_update(0, 0, 1)
        event.accept()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.cleanup()

    #Methods for quickly changing style sheets
    def greenify(self, item: QWidget) -> None:
        item.setStyleSheet("background-color: rgb(135, 255, 135); color: rgb(0, 0, 0);")

    def redify(self, item: QWidget) -> None:
        item.setStyleSheet("background-color: rgb(255, 155, 155); color: rgb(0, 0, 0);")

    def plainify(self, item: QWidget) -> None:
        item.setStyleSheet("")

    def plainify_modes(self) -> None:
        self.plainify(self.ui.node_control)
        self.plainify(self.ui.crawling)
        self.plainify(self.ui.calibration)
        self.plainify(self.ui.sit_stand)

    #Method for quickly logging to the faux terminal
    @Slot(str)
    def term_log(self, text: object) -> None:
        now = datetime.now()
        self.ui.term.appendPlainText(f"{now:%m/%d/%y %H:%M:%S} - {text}")
