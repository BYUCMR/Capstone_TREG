import time

import panda3d.core as p3d
from PySide6.QtCore import QObject, QThread, Signal, Slot


class JoystickHandler(QObject):
    message = Signal(str)
    command = Signal(float,float,float)

    def __init__(self, ui,parent: QObject | None = None):
        super().__init__(parent)
        self.ui = ui
        self.message.emit("test")
        self.ui.js_toggle.clicked.connect(self.start_joystick)

    @Slot()
    def start_joystick(self):
        self.js_worker = JoyWorker()
        self.js_thread = QThread()

        self.js_worker.moveToThread(self.js_thread)

        self.js_thread.started.connect(self.js_worker.heavy_task)
        self.js_worker.finished.connect(self.js_thread.quit)
        self.js_worker.finished.connect(self.js_worker.deleteLater)
        self.js_thread.finished.connect(self.js_thread.deleteLater)

        self.js_worker.message.connect(self.message.emit)
        self.js_worker.action.connect(self.command.emit)

        self.js_thread.start()

    def message_pr(self,msg):
        print(msg)


class JoyWorker(QObject):
    action = Signal(float, float, float)
    message = Signal(str)
    finished = Signal()
    device_mgr = p3d.InputDeviceManager.get_global_ptr()

    @Slot()
    def heavy_task(self):
        self.message.emit("Joystick Handler Running")
        self.device_mgr.update()
        joysticks = self.device_mgr.get_devices(p3d.InputDevice.DeviceClass.flight_stick)
        if len(joysticks) == 0:
            self.finished.emit()
            self.message.emit("No Joystick Found")
        else:
            joystick = joysticks[0]
            self.message.emit(f"Jostick Initialized: {joystick.name}")
            self.hosting = True
            while self.hosting:
                # if joystick.find_button('trigger').pressed:
                #     self.message.emit("Joystick Button Pressed")
                #     self.action.emit()
                x = -joystick.find_axis(p3d.InputDevice.Axis.pitch).value
                if x < 0.1 and x > -0.1: x = 0
                y = -joystick.find_axis(p3d.InputDevice.Axis.roll).value
                if y < 0.1 and y > -0.1: y = 0
                z = -joystick.find_axis(p3d.InputDevice.Axis.yaw).value
                if z < 0.1 and z > -0.1: z = 0
                self.action.emit(x,y,z)
                time.sleep(0.1)
            self.finished.emit()
                # time.sleep(0.5)
