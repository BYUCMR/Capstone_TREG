# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main.ui'
##
## Created by: Qt User Interface Compiler version 6.10.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QPlainTextEdit, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QSpinBox,
    QSplitter, QTabWidget, QVBoxLayout, QWidget)

class Ui_Control(object):
    def setupUi(self, Control):
        if not Control.objectName():
            Control.setObjectName(u"Control")
        Control.resize(800, 600)
        self.centralwidget = QWidget(Control)
        self.centralwidget.setObjectName(u"centralwidget")
        self.ctr_layout = QHBoxLayout(self.centralwidget)
        self.ctr_layout.setObjectName(u"ctr_layout")
        self.Full_Splitter = QSplitter(self.centralwidget)
        self.Full_Splitter.setObjectName(u"Full_Splitter")
        self.Full_Splitter.setOrientation(Qt.Orientation.Horizontal)
        self.verticalLayoutWidget_2 = QWidget(self.Full_Splitter)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.L1 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.L1.setObjectName(u"L1")
        self.L1.setContentsMargins(0, 0, 0, 0)
        self.L11 = QGridLayout()
        self.L11.setObjectName(u"L11")
        self.left = QPushButton(self.verticalLayoutWidget_2)
        self.left.setObjectName(u"left")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.left.sizePolicy().hasHeightForWidth())
        self.left.setSizePolicy(sizePolicy)
        self.left.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.left, 1, 0, 1, 1)

        self.del_right = QPushButton(self.verticalLayoutWidget_2)
        self.del_right.setObjectName(u"del_right")
        sizePolicy.setHeightForWidth(self.del_right.sizePolicy().hasHeightForWidth())
        self.del_right.setSizePolicy(sizePolicy)
        self.del_right.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.del_right, 0, 2, 1, 1)

        self.right = QPushButton(self.verticalLayoutWidget_2)
        self.right.setObjectName(u"right")
        sizePolicy.setHeightForWidth(self.right.sizePolicy().hasHeightForWidth())
        self.right.setSizePolicy(sizePolicy)
        self.right.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.right, 1, 2, 1, 1)

        self.del_left = QPushButton(self.verticalLayoutWidget_2)
        self.del_left.setObjectName(u"del_left")
        sizePolicy.setHeightForWidth(self.del_left.sizePolicy().hasHeightForWidth())
        self.del_left.setSizePolicy(sizePolicy)
        self.del_left.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.del_left, 0, 0, 1, 1)

        self.backward = QPushButton(self.verticalLayoutWidget_2)
        self.backward.setObjectName(u"backward")
        sizePolicy.setHeightForWidth(self.backward.sizePolicy().hasHeightForWidth())
        self.backward.setSizePolicy(sizePolicy)
        self.backward.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.backward, 1, 1, 1, 1)

        self.forward = QPushButton(self.verticalLayoutWidget_2)
        self.forward.setObjectName(u"forward")
        sizePolicy.setHeightForWidth(self.forward.sizePolicy().hasHeightForWidth())
        self.forward.setSizePolicy(sizePolicy)
        self.forward.setMinimumSize(QSize(0, 70))

        self.L11.addWidget(self.forward, 0, 1, 1, 1)


        self.L1.addLayout(self.L11)

        self.L12 = QHBoxLayout()
        self.L12.setObjectName(u"L12")
        self.L12.setContentsMargins(-1, 0, -1, -1)
        self.zero_pos = QPushButton(self.verticalLayoutWidget_2)
        self.zero_pos.setObjectName(u"zero_pos")
        self.zero_pos.setStyleSheet(u"background-color: qlineargradient(spread:repeat, x1:0, y1:0, x2:1, y2:0, stop:0.249 rgba(255, 255, 0, 255), stop:0.25 rgba(0, 0, 0, 255), stop:0.499 rgba(0, 0, 0, 255), stop:0.5 rgba(255, 255, 0, 255), stop:0.75 rgba(255, 255, 0, 255), stop:0.751 rgba(0, 0, 0, 255), stop:1 rgba(0, 0, 0, 255));\n"
"color: rgb(255, 0, 0);")

        self.L12.addWidget(self.zero_pos)

        self.L121 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.L12.addItem(self.L121)

        self.selector_label = QLabel(self.verticalLayoutWidget_2)
        self.selector_label.setObjectName(u"selector_label")

        self.L12.addWidget(self.selector_label)

        self.selector = QSpinBox(self.verticalLayoutWidget_2)
        self.selector.setObjectName(u"selector")
        self.selector.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.selector.setMinimum(0)
        self.selector.setMaximum(11)

        self.L12.addWidget(self.selector)


        self.L1.addLayout(self.L12)

        self.L14 = QGridLayout()
        self.L14.setObjectName(u"L14")
        self.node_control = QPushButton(self.verticalLayoutWidget_2)
        self.node_control.setObjectName(u"node_control")

        self.L14.addWidget(self.node_control, 0, 0, 1, 1)

        self.crawling = QPushButton(self.verticalLayoutWidget_2)
        self.crawling.setObjectName(u"crawling")
        self.crawling.setStyleSheet(u"")

        self.L14.addWidget(self.crawling, 0, 1, 1, 1)

        self.reset_button = QPushButton(self.verticalLayoutWidget_2)
        self.reset_button.setObjectName(u"reset_button")
        self.reset_button.setEnabled(True)

        self.L14.addWidget(self.reset_button, 1, 1, 1, 1)

        self.calibration = QPushButton(self.verticalLayoutWidget_2)
        self.calibration.setObjectName(u"calibration")

        self.L14.addWidget(self.calibration, 1, 2, 1, 1)

        self.rolling = QPushButton(self.verticalLayoutWidget_2)
        self.rolling.setObjectName(u"rolling")
        self.rolling.setEnabled(False)

        self.L14.addWidget(self.rolling, 0, 2, 1, 1)

        self.sit_stand = QPushButton(self.verticalLayoutWidget_2)
        self.sit_stand.setObjectName(u"sit_stand")
        self.sit_stand.setEnabled(True)

        self.L14.addWidget(self.sit_stand, 1, 0, 1, 1)


        self.L1.addLayout(self.L14)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.L1.addItem(self.verticalSpacer)

        self.Full_Splitter.addWidget(self.verticalLayoutWidget_2)
        self.verticalLayoutWidget = QWidget(self.Full_Splitter)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.L2 = QVBoxLayout(self.verticalLayoutWidget)
        self.L2.setObjectName(u"L2")
        self.L2.setContentsMargins(0, 0, 0, 0)
        self.L21 = QGridLayout()
        self.L21.setObjectName(u"L21")
        self.L21.setContentsMargins(-1, 0, -1, -1)
        self.bot_toggle = QPushButton(self.verticalLayoutWidget)
        self.bot_toggle.setObjectName(u"bot_toggle")

        self.L21.addWidget(self.bot_toggle, 1, 0, 1, 1)

        self.bot_label = QPushButton(self.verticalLayoutWidget)
        self.bot_label.setObjectName(u"bot_label")
        self.bot_label.setStyleSheet(u"background-color: rgb(255, 155, 155);\n"
"color: rgb(0, 0, 0);")

        self.L21.addWidget(self.bot_label, 1, 1, 1, 1)

        self.js_toggle = QPushButton(self.verticalLayoutWidget)
        self.js_toggle.setObjectName(u"js_toggle")

        self.L21.addWidget(self.js_toggle, 2, 0, 1, 1)

        self.js_label = QPushButton(self.verticalLayoutWidget)
        self.js_label.setObjectName(u"js_label")
        self.js_label.setStyleSheet(u"background-color: rgb(255, 155, 155);\n"
"color: rgb(0, 0, 0);")

        self.L21.addWidget(self.js_label, 2, 1, 1, 1)

        self.sim_toggle = QPushButton(self.verticalLayoutWidget)
        self.sim_toggle.setObjectName(u"sim_toggle")

        self.L21.addWidget(self.sim_toggle, 0, 0, 1, 1)

        self.sim_label = QPushButton(self.verticalLayoutWidget)
        self.sim_label.setObjectName(u"sim_label")
        self.sim_label.setStyleSheet(u"background-color: rgb(255, 155, 155);\n"
"color: rgb(0, 0, 0);")

        self.L21.addWidget(self.sim_label, 0, 1, 1, 1)


        self.L2.addLayout(self.L21)

        self.L211 = QHBoxLayout()
        self.L211.setObjectName(u"L211")
        self.L211.setContentsMargins(-1, 0, -1, -1)
        self.serial_label = QLabel(self.verticalLayoutWidget)
        self.serial_label.setObjectName(u"serial_label")
        self.serial_label.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.L211.addWidget(self.serial_label)

        self.serial_select = QComboBox(self.verticalLayoutWidget)
        self.serial_select.setObjectName(u"serial_select")

        self.L211.addWidget(self.serial_select)


        self.L2.addLayout(self.L211)

        self.L22 = QTabWidget(self.verticalLayoutWidget)
        self.L22.setObjectName(u"L22")
        self.term_tab = QWidget()
        self.term_tab.setObjectName(u"term_tab")
        self.verticalLayout = QVBoxLayout(self.term_tab)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.term = QPlainTextEdit(self.term_tab)
        self.term.setObjectName(u"term")
        self.term.setEnabled(True)
        self.term.setReadOnly(True)

        self.verticalLayout.addWidget(self.term)

        self.L22.addTab(self.term_tab, "")
        self.motor_tab = QWidget()
        self.motor_tab.setObjectName(u"motor_tab")
        self.verticalLayout_2 = QVBoxLayout(self.motor_tab)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.L221 = QGridLayout()
        self.L221.setObjectName(u"L221")
        self.ms_11 = QSlider(self.motor_tab)
        self.ms_11.setObjectName(u"ms_11")
        self.ms_11.setEnabled(False)
        self.ms_11.setMinimum(-1000)
        self.ms_11.setMaximum(1000)
        self.ms_11.setOrientation(Qt.Orientation.Horizontal)
        self.ms_11.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_11.setTickInterval(200)

        self.L221.addWidget(self.ms_11, 12, 1, 1, 1)

        self.ms_5 = QSlider(self.motor_tab)
        self.ms_5.setObjectName(u"ms_5")
        self.ms_5.setEnabled(False)
        self.ms_5.setMinimum(-1000)
        self.ms_5.setMaximum(1000)
        self.ms_5.setOrientation(Qt.Orientation.Horizontal)
        self.ms_5.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_5.setTickInterval(200)

        self.L221.addWidget(self.ms_5, 6, 1, 1, 1)

        self.mb_11 = QPushButton(self.motor_tab)
        self.mb_11.setObjectName(u"mb_11")

        self.L221.addWidget(self.mb_11, 12, 0, 1, 1)

        self.mb_1 = QPushButton(self.motor_tab)
        self.mb_1.setObjectName(u"mb_1")

        self.L221.addWidget(self.mb_1, 2, 0, 1, 1)

        self.ms_2 = QSlider(self.motor_tab)
        self.ms_2.setObjectName(u"ms_2")
        self.ms_2.setEnabled(False)
        self.ms_2.setMinimum(-1000)
        self.ms_2.setMaximum(1000)
        self.ms_2.setOrientation(Qt.Orientation.Horizontal)
        self.ms_2.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_2.setTickInterval(200)

        self.L221.addWidget(self.ms_2, 3, 1, 1, 1)

        self.ms_8 = QSlider(self.motor_tab)
        self.ms_8.setObjectName(u"ms_8")
        self.ms_8.setEnabled(False)
        self.ms_8.setMinimum(-1000)
        self.ms_8.setMaximum(1000)
        self.ms_8.setOrientation(Qt.Orientation.Horizontal)
        self.ms_8.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_8.setTickInterval(200)

        self.L221.addWidget(self.ms_8, 9, 1, 1, 1)

        self.ms_10 = QSlider(self.motor_tab)
        self.ms_10.setObjectName(u"ms_10")
        self.ms_10.setEnabled(False)
        self.ms_10.setMinimum(-1000)
        self.ms_10.setMaximum(1000)
        self.ms_10.setOrientation(Qt.Orientation.Horizontal)
        self.ms_10.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_10.setTickInterval(200)

        self.L221.addWidget(self.ms_10, 11, 1, 1, 1)

        self.ms_9 = QSlider(self.motor_tab)
        self.ms_9.setObjectName(u"ms_9")
        self.ms_9.setEnabled(False)
        self.ms_9.setMinimum(-1000)
        self.ms_9.setMaximum(1000)
        self.ms_9.setOrientation(Qt.Orientation.Horizontal)
        self.ms_9.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_9.setTickInterval(200)

        self.L221.addWidget(self.ms_9, 10, 1, 1, 1)

        self.ms_7 = QSlider(self.motor_tab)
        self.ms_7.setObjectName(u"ms_7")
        self.ms_7.setEnabled(False)
        self.ms_7.setMinimum(-1000)
        self.ms_7.setMaximum(1000)
        self.ms_7.setValue(0)
        self.ms_7.setOrientation(Qt.Orientation.Horizontal)
        self.ms_7.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_7.setTickInterval(200)

        self.L221.addWidget(self.ms_7, 8, 1, 1, 1)

        self.mb_3 = QPushButton(self.motor_tab)
        self.mb_3.setObjectName(u"mb_3")

        self.L221.addWidget(self.mb_3, 4, 0, 1, 1)

        self.motor_err_label = QLabel(self.motor_tab)
        self.motor_err_label.setObjectName(u"motor_err_label")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.motor_err_label.sizePolicy().hasHeightForWidth())
        self.motor_err_label.setSizePolicy(sizePolicy1)
        self.motor_err_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.L221.addWidget(self.motor_err_label, 0, 1, 1, 1)

        self.mb_7 = QPushButton(self.motor_tab)
        self.mb_7.setObjectName(u"mb_7")

        self.L221.addWidget(self.mb_7, 8, 0, 1, 1)

        self.mb_2 = QPushButton(self.motor_tab)
        self.mb_2.setObjectName(u"mb_2")

        self.L221.addWidget(self.mb_2, 3, 0, 1, 1)

        self.ms_6 = QSlider(self.motor_tab)
        self.ms_6.setObjectName(u"ms_6")
        self.ms_6.setEnabled(False)
        self.ms_6.setMinimum(-1000)
        self.ms_6.setMaximum(1000)
        self.ms_6.setValue(0)
        self.ms_6.setOrientation(Qt.Orientation.Horizontal)
        self.ms_6.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_6.setTickInterval(200)

        self.L221.addWidget(self.ms_6, 7, 1, 1, 1)

        self.ms_1 = QSlider(self.motor_tab)
        self.ms_1.setObjectName(u"ms_1")
        self.ms_1.setEnabled(False)
        self.ms_1.setMinimum(-1000)
        self.ms_1.setMaximum(1000)
        self.ms_1.setTracking(True)
        self.ms_1.setOrientation(Qt.Orientation.Horizontal)
        self.ms_1.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_1.setTickInterval(200)

        self.L221.addWidget(self.ms_1, 2, 1, 1, 1)

        self.ms_3 = QSlider(self.motor_tab)
        self.ms_3.setObjectName(u"ms_3")
        self.ms_3.setEnabled(False)
        self.ms_3.setMinimum(-1000)
        self.ms_3.setMaximum(1000)
        self.ms_3.setOrientation(Qt.Orientation.Horizontal)
        self.ms_3.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_3.setTickInterval(200)

        self.L221.addWidget(self.ms_3, 4, 1, 1, 1)

        self.mb_4 = QPushButton(self.motor_tab)
        self.mb_4.setObjectName(u"mb_4")

        self.L221.addWidget(self.mb_4, 5, 0, 1, 1)

        self.ms_12 = QSlider(self.motor_tab)
        self.ms_12.setObjectName(u"ms_12")
        self.ms_12.setEnabled(False)
        self.ms_12.setMinimum(-1000)
        self.ms_12.setMaximum(1000)
        self.ms_12.setOrientation(Qt.Orientation.Horizontal)
        self.ms_12.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_12.setTickInterval(200)

        self.L221.addWidget(self.ms_12, 13, 1, 1, 1)

        self.mb_6 = QPushButton(self.motor_tab)
        self.mb_6.setObjectName(u"mb_6")

        self.L221.addWidget(self.mb_6, 7, 0, 1, 1)

        self.mb_5 = QPushButton(self.motor_tab)
        self.mb_5.setObjectName(u"mb_5")

        self.L221.addWidget(self.mb_5, 6, 0, 1, 1)

        self.L2211 = QHBoxLayout()
        self.L2211.setObjectName(u"L2211")
        self.ms_label_1 = QLabel(self.motor_tab)
        self.ms_label_1.setObjectName(u"ms_label_1")

        self.L2211.addWidget(self.ms_label_1)

        self.S22111 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.L2211.addItem(self.S22111)

        self.ms_label_2 = QLabel(self.motor_tab)
        self.ms_label_2.setObjectName(u"ms_label_2")
        self.ms_label_2.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.L2211.addWidget(self.ms_label_2)


        self.L221.addLayout(self.L2211, 1, 1, 1, 1)

        self.mb_10 = QPushButton(self.motor_tab)
        self.mb_10.setObjectName(u"mb_10")

        self.L221.addWidget(self.mb_10, 11, 0, 1, 1)

        self.mb_9 = QPushButton(self.motor_tab)
        self.mb_9.setObjectName(u"mb_9")

        self.L221.addWidget(self.mb_9, 10, 0, 1, 1)

        self.eq_all = QPushButton(self.motor_tab)
        self.eq_all.setObjectName(u"eq_all")

        self.L221.addWidget(self.eq_all, 1, 0, 1, 1)

        self.mb_8 = QPushButton(self.motor_tab)
        self.mb_8.setObjectName(u"mb_8")

        self.L221.addWidget(self.mb_8, 9, 0, 1, 1)

        self.mb_12 = QPushButton(self.motor_tab)
        self.mb_12.setObjectName(u"mb_12")

        self.L221.addWidget(self.mb_12, 13, 0, 1, 1)

        self.ms_4 = QSlider(self.motor_tab)
        self.ms_4.setObjectName(u"ms_4")
        self.ms_4.setEnabled(False)
        self.ms_4.setMinimum(-1000)
        self.ms_4.setMaximum(1000)
        self.ms_4.setOrientation(Qt.Orientation.Horizontal)
        self.ms_4.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.ms_4.setTickInterval(200)

        self.L221.addWidget(self.ms_4, 5, 1, 1, 1)

        self.S221 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.L221.addItem(self.S221, 14, 1, 1, 1)


        self.verticalLayout_2.addLayout(self.L221)

        self.L22.addTab(self.motor_tab, "")

        self.L2.addWidget(self.L22)

        self.Full_Splitter.addWidget(self.verticalLayoutWidget)

        self.ctr_layout.addWidget(self.Full_Splitter)

        Control.setCentralWidget(self.centralwidget)

        self.retranslateUi(Control)

        self.L22.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(Control)
    # setupUi

    def retranslateUi(self, Control):
        Control.setWindowTitle(QCoreApplication.translate("Control", u"Control", None))
        self.left.setText(QCoreApplication.translate("Control", u"Left", None))
        self.del_right.setText(QCoreApplication.translate("Control", u"Right Delta", None))
        self.right.setText(QCoreApplication.translate("Control", u"Right", None))
        self.del_left.setText(QCoreApplication.translate("Control", u"Left Delta", None))
        self.backward.setText(QCoreApplication.translate("Control", u"Backward", None))
        self.forward.setText(QCoreApplication.translate("Control", u"Forward", None))
        self.zero_pos.setText(QCoreApplication.translate("Control", u"Zero", None))
        self.selector_label.setText(QCoreApplication.translate("Control", u"Node", None))
        self.node_control.setText(QCoreApplication.translate("Control", u"Node Control", None))
        self.crawling.setText(QCoreApplication.translate("Control", u"Crawling", None))
        self.reset_button.setText(QCoreApplication.translate("Control", u"Reset", None))
        self.calibration.setText(QCoreApplication.translate("Control", u"Calibration", None))
        self.rolling.setText(QCoreApplication.translate("Control", u"Rolling", None))
        self.sit_stand.setText(QCoreApplication.translate("Control", u"Sit/Stand", None))
        self.bot_toggle.setText(QCoreApplication.translate("Control", u"Connect Robot", None))
        self.bot_label.setText(QCoreApplication.translate("Control", u"Robot Offline", None))
        self.js_toggle.setText(QCoreApplication.translate("Control", u"Connect Joystick", None))
        self.js_label.setText(QCoreApplication.translate("Control", u"Joystick Offline", None))
        self.sim_toggle.setText(QCoreApplication.translate("Control", u"Begin Simulation", None))
        self.sim_label.setText(QCoreApplication.translate("Control", u"Simulation Offine", None))
        self.serial_label.setText(QCoreApplication.translate("Control", u"Serial Port", None))
        self.L22.setTabText(self.L22.indexOf(self.term_tab), QCoreApplication.translate("Control", u"Terminal", None))
        self.mb_11.setText(QCoreApplication.translate("Control", u"11", None))
        self.mb_1.setText(QCoreApplication.translate("Control", u"1", None))
        self.mb_3.setText(QCoreApplication.translate("Control", u"3", None))
        self.motor_err_label.setText(QCoreApplication.translate("Control", u"Error", None))
        self.mb_7.setText(QCoreApplication.translate("Control", u"7", None))
        self.mb_2.setText(QCoreApplication.translate("Control", u"2", None))
        self.mb_4.setText(QCoreApplication.translate("Control", u"4", None))
        self.mb_6.setText(QCoreApplication.translate("Control", u"6", None))
        self.mb_5.setText(QCoreApplication.translate("Control", u"5", None))
        self.ms_label_1.setText(QCoreApplication.translate("Control", u"-1000", None))
        self.ms_label_2.setText(QCoreApplication.translate("Control", u"1000", None))
        self.mb_10.setText(QCoreApplication.translate("Control", u"10", None))
        self.mb_9.setText(QCoreApplication.translate("Control", u"9", None))
        self.eq_all.setText(QCoreApplication.translate("Control", u"EQ All", None))
        self.mb_8.setText(QCoreApplication.translate("Control", u"8", None))
        self.mb_12.setText(QCoreApplication.translate("Control", u"12", None))
        self.L22.setTabText(self.L22.indexOf(self.motor_tab), QCoreApplication.translate("Control", u"Motor Error", None))
    # retranslateUi

