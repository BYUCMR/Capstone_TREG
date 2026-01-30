from os import environ
environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import warnings
warnings.filterwarnings("ignore")
import pygame
from PySide6.QtCore import QThread, Signal

class JoystickHandler:
    def __init__(self, ui):
        self.ui = ui
        self.ui.js_toggle.clicked.connect(self.start_joystick)


    def start_joystick(self):
        self.js_thread = JoyWorker()
        self.js_thread.message.connect(self.ui.term_log)

        self.js_thread.start()




class JoyWorker(QThread):
    action = Signal()
    message = Signal(str)
    finished = Signal()

    def run(self):
        self.message.emit("Pygame Running")
        pygame.init()
        pygame.joystick.init()
        if(pygame.joystick.get_count() == 0):
            pygame.joystick.quit()
            pygame.quit()
            self.finished.emit()
            self.message.emit("No Joystick Found")
        else:
            joystick = pygame.joystick.Joystick(0)
            self.message.emit(f"Jostick Initialized: {joystick.get_name()}")
            self.hosting = True
            while self.hosting:
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        self.message.emit("Joystick Button Pressed")
                        self.action.emit()
                # time.sleep(0.5)













# class JoystickHandler:
#     def __init__(self, ui):
#         self.ui = ui
#         self.js_live = False

#         self.ui.js_toggle.clicked.connect(self.start_joystick)

#     def red(self):
#         self.ui.term_log("Button Clicked")

#     def start_joystick(self):
#         js_thread = QThreadPool()
#         js_worker = JWorker()
#         # js_worker.signals.action.connect(self.red)
#         # self.js_worker.moveToThread(self.js_thread)
#         # self.js_thread.started.connect(self.js_worker.run)
#         # self.js_worker.action.connect(self.red)
#         # self.js_worker.finished.connect(self.js_thread.quit)
#         # self.js_worker.finished.connect(self.js_worker.deleteLater)
#         # self.js_worker.finished.connect(self.js_thread.deleteLater)

#         js_thread.start(js_worker)
#         return


# # class WorkerSignals(QObject):
# #     action = Signal(str)
# #     finished = Signal(str)


# class JWorker(QRunnable):
#     def __init__(self):
#         super().__init__()
#         # self.signals = WorkerSignals()
#         self.hosting = False

#     def run(self):
#         print("Running")
#         pygame.init()
#         pygame.joystick.init()
#         if(pygame.joystick.get_count() == 0):
#             pygame.joystick.quit()
#             pygame.quit()
#             self.finished.emit()
#             print("no joystick")
#         else:
#             self.joystick = pygame.joystick.Joystick(0)
#             self.hosting = True
#             print("started")
#             while self.hosting:
#                 for event in pygame.event.get():
#                     if event.type == pygame.JOYBUTTONDOWN:
#                         print("button")
#                 time.sleep(0.5)


#     # def start_joystick(self):
#     #     if (self.js_live == False):
#     #         pygame.init()
#     #         pygame.joystick.init()
#     #         if(pygame.joystick.get_count() == 0):
#     #             self.ui.term_log("No joysticks found!")
#     #             pygame.joystick.quit()
#     #             pygame.quit()
#     #         else:
#     #             self.js_thread = QThread()
#     #             self.joystick = pygame.joystick.Joystick(0)
#     #             self.js_live = True
#     #             self.ui.term_log(f"Jostick Initialized: {self.joystick.get_name()}")
#     #             self.js_worker = JoystickWorker(self.joystick)
#     #             self.js_worker.moveToThread(self.js_thread)
#     #             self.js_thread.run = self.js_worker.host

#     #             self.js_worker.action.connect(self.red)

#     #             self.js_thread.start()
#     #     else:
#     #         self.js_worker.hosting = False
#     #         pygame.joystick.quit()
#     #         pygame.quit()
#     #         self.js_thread.quit()
#     #         self.js_live = False
#     #         self.ui.term_log("Joystick Killed")



# # class JoystickWorker(QObject):
# #     action = Signal(str)

# #     def __init__(self, joystick, parent=None):
# #         super().__init__(parent)
# #         self.joystick = joystick
# #         self.hosting = False

# #     def host(self):
# #         self.hosting = True
# #         while self.hosting:
# #             for event in pygame.event.get():
# #                 if event.type == pygame.JOYBUTTONDOWN:
# #                     self.action.emit("button_down")
# #             time.sleep(.05)

