from ui_vis import Ui_Form

class SimHandler:
    def __init__(self, ui):
        self.ui = ui
        self.ui.sim_toggle.clicked.connect(self.toggle_sim)
