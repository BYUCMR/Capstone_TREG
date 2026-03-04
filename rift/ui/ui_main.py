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
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSizePolicy,
    QSpacerItem, QSpinBox, QSplitter, QVBoxLayout,
    QWidget)

class Ui_Control(object):
    def setupUi(self, Control):
        if not Control.objectName():
            Control.setObjectName(u"Control")
        Control.resize(600, 800)
        self.centralwidget = QWidget(Control)
        self.centralwidget.setObjectName(u"centralwidget")
        self.ctr_layout = QVBoxLayout(self.centralwidget)
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
        self.L121 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.L12.addItem(self.L121)

        self.selector_label = QLabel(self.verticalLayoutWidget_2)
        self.selector_label.setObjectName(u"selector_label")

        self.L12.addWidget(self.selector_label)

        self.selector = QSpinBox(self.verticalLayoutWidget_2)
        self.selector.setObjectName(u"selector")
        self.selector.setMinimum(0)
        self.selector.setMaximum(11)

        self.L12.addWidget(self.selector)


        self.L1.addLayout(self.L12)

        self.L13 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.L1.addItem(self.L13)

        self.L14 = QGridLayout()
        self.L14.setObjectName(u"L14")
        self.sit_stand = QPushButton(self.verticalLayoutWidget_2)
        self.sit_stand.setObjectName(u"sit_stand")
        self.sit_stand.setEnabled(False)

        self.L14.addWidget(self.sit_stand, 1, 0, 1, 1)

        self.crawling = QPushButton(self.verticalLayoutWidget_2)
        self.crawling.setObjectName(u"crawling")
        self.crawling.setStyleSheet(u"")

        self.L14.addWidget(self.crawling, 0, 1, 1, 1)

        self.rolling = QPushButton(self.verticalLayoutWidget_2)
        self.rolling.setObjectName(u"rolling")
        self.rolling.setEnabled(False)

        self.L14.addWidget(self.rolling, 0, 2, 1, 1)

        self.node_control = QPushButton(self.verticalLayoutWidget_2)
        self.node_control.setObjectName(u"node_control")

        self.L14.addWidget(self.node_control, 0, 0, 1, 1)

        self.chimney = QPushButton(self.verticalLayoutWidget_2)
        self.chimney.setObjectName(u"chimney")
        self.chimney.setEnabled(False)

        self.L14.addWidget(self.chimney, 1, 1, 1, 1)

        self.calibration = QPushButton(self.verticalLayoutWidget_2)
        self.calibration.setObjectName(u"calibration")

        self.L14.addWidget(self.calibration, 1, 2, 1, 1)


        self.L1.addLayout(self.L14)

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

        self.term = QPlainTextEdit(self.verticalLayoutWidget)
        self.term.setObjectName(u"term")
        self.term.setEnabled(True)
        self.term.setReadOnly(True)

        self.L2.addWidget(self.term)

        self.Full_Splitter.addWidget(self.verticalLayoutWidget)

        self.ctr_layout.addWidget(self.Full_Splitter)

        Control.setCentralWidget(self.centralwidget)

        self.retranslateUi(Control)

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
        self.selector_label.setText(QCoreApplication.translate("Control", u"Node", None))
        self.sit_stand.setText(QCoreApplication.translate("Control", u"Sit/Stand", None))
        self.crawling.setText(QCoreApplication.translate("Control", u"Crawling", None))
        self.rolling.setText(QCoreApplication.translate("Control", u"Rolling", None))
        self.node_control.setText(QCoreApplication.translate("Control", u"Node Control", None))
        self.chimney.setText(QCoreApplication.translate("Control", u"Chimney", None))
        self.calibration.setText(QCoreApplication.translate("Control", u"Calibration", None))
        self.bot_toggle.setText(QCoreApplication.translate("Control", u"Connect Robot", None))
        self.bot_label.setText(QCoreApplication.translate("Control", u"Robot Offline", None))
        self.js_toggle.setText(QCoreApplication.translate("Control", u"Connect Joystick", None))
        self.js_label.setText(QCoreApplication.translate("Control", u"Joystick Offline", None))
        self.sim_toggle.setText(QCoreApplication.translate("Control", u"Begin Simulation", None))
        self.sim_label.setText(QCoreApplication.translate("Control", u"Simulation Offine", None))
    # retranslateUi

