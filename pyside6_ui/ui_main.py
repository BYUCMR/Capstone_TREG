# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QLCDNumber, QLabel,
    QLineEdit, QMainWindow, QMenuBar, QPlainTextEdit,
    QPushButton, QSizePolicy, QSlider, QSpacerItem,
    QSpinBox, QSplitter, QStatusBar, QTabWidget,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1000, 1000)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_2 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.Full_Splitter = QSplitter(self.centralwidget)
        self.Full_Splitter.setObjectName(u"Full_Splitter")
        self.Full_Splitter.setOrientation(Qt.Orientation.Vertical)
        self.Bottom_Splitter = QSplitter(self.Full_Splitter)
        self.Bottom_Splitter.setObjectName(u"Bottom_Splitter")
        self.Bottom_Splitter.setOrientation(Qt.Orientation.Horizontal)
        self.control_tabs = QTabWidget(self.Bottom_Splitter)
        self.control_tabs.setObjectName(u"control_tabs")
        self.node_tab = QWidget()
        self.node_tab.setObjectName(u"node_tab")
        self.spinBox = QSpinBox(self.node_tab)
        self.spinBox.setObjectName(u"spinBox")
        self.spinBox.setGeometry(QRect(220, 20, 121, 25))
        self.label_2 = QLabel(self.node_tab)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(110, 20, 49, 16))
        self.pushButton_2 = QPushButton(self.node_tab)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(59, 190, 101, 31))
        icon = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.MediaSeekBackward))
        self.pushButton_2.setIcon(icon)
        self.lineEdit = QLineEdit(self.node_tab)
        self.lineEdit.setObjectName(u"lineEdit")
        self.lineEdit.setGeometry(QRect(270, 190, 113, 24))
        self.label_3 = QLabel(self.node_tab)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(280, 160, 49, 16))
        self.label_4 = QLabel(self.node_tab)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(90, 160, 49, 16))
        self.label_5 = QLabel(self.node_tab)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(270, 240, 49, 16))
        self.label_6 = QLabel(self.node_tab)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(80, 240, 49, 16))
        self.lineEdit_2 = QLineEdit(self.node_tab)
        self.lineEdit_2.setObjectName(u"lineEdit_2")
        self.lineEdit_2.setGeometry(QRect(260, 270, 113, 24))
        self.label_7 = QLabel(self.node_tab)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(270, 340, 49, 16))
        self.label_8 = QLabel(self.node_tab)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(80, 340, 49, 16))
        self.lineEdit_3 = QLineEdit(self.node_tab)
        self.lineEdit_3.setObjectName(u"lineEdit_3")
        self.lineEdit_3.setGeometry(QRect(260, 370, 113, 24))
        self.pushButton_14 = QPushButton(self.node_tab)
        self.pushButton_14.setObjectName(u"pushButton_14")
        self.pushButton_14.setGeometry(QRect(170, 190, 101, 31))
        icon1 = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.MediaSeekForward))
        self.pushButton_14.setIcon(icon1)
        self.pushButton_15 = QPushButton(self.node_tab)
        self.pushButton_15.setObjectName(u"pushButton_15")
        self.pushButton_15.setGeometry(QRect(151, 280, 101, 31))
        self.pushButton_15.setIcon(icon1)
        self.pushButton_3 = QPushButton(self.node_tab)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(40, 280, 101, 31))
        self.pushButton_3.setIcon(icon)
        self.pushButton_16 = QPushButton(self.node_tab)
        self.pushButton_16.setObjectName(u"pushButton_16")
        self.pushButton_16.setGeometry(QRect(151, 370, 101, 31))
        self.pushButton_16.setIcon(icon1)
        self.pushButton_4 = QPushButton(self.node_tab)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(40, 370, 101, 31))
        self.pushButton_4.setIcon(icon)
        self.control_tabs.addTab(self.node_tab, "")
        self.crawl_tab = QWidget()
        self.crawl_tab.setObjectName(u"crawl_tab")
        self.pushButton_7 = QPushButton(self.crawl_tab)
        self.pushButton_7.setObjectName(u"pushButton_7")
        self.pushButton_7.setGeometry(QRect(170, 120, 80, 24))
        icon2 = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.GoUp))
        self.pushButton_7.setIcon(icon2)
        self.pushButton_8 = QPushButton(self.crawl_tab)
        self.pushButton_8.setObjectName(u"pushButton_8")
        self.pushButton_8.setGeometry(QRect(160, 240, 80, 24))
        icon3 = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.GoDown))
        self.pushButton_8.setIcon(icon3)
        self.pushButton_9 = QPushButton(self.crawl_tab)
        self.pushButton_9.setObjectName(u"pushButton_9")
        self.pushButton_9.setGeometry(QRect(280, 170, 80, 24))
        self.pushButton_9.setIcon(icon1)
        self.pushButton_10 = QPushButton(self.crawl_tab)
        self.pushButton_10.setObjectName(u"pushButton_10")
        self.pushButton_10.setGeometry(QRect(40, 180, 80, 24))
        self.pushButton_10.setIcon(icon)
        self.pushButton_11 = QPushButton(self.crawl_tab)
        self.pushButton_11.setObjectName(u"pushButton_11")
        self.pushButton_11.setGeometry(QRect(170, 170, 80, 24))
        self.horizontalSlider = QSlider(self.crawl_tab)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setGeometry(QRect(90, 320, 160, 16))
        self.horizontalSlider.setOrientation(Qt.Orientation.Horizontal)
        self.label_9 = QLabel(self.crawl_tab)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(100, 290, 49, 16))
        self.control_tabs.addTab(self.crawl_tab, "")
        self.roll_tab = QWidget()
        self.roll_tab.setObjectName(u"roll_tab")
        self.pushButton_12 = QPushButton(self.roll_tab)
        self.pushButton_12.setObjectName(u"pushButton_12")
        self.pushButton_12.setGeometry(QRect(160, 90, 80, 24))
        self.pushButton_13 = QPushButton(self.roll_tab)
        self.pushButton_13.setObjectName(u"pushButton_13")
        self.pushButton_13.setGeometry(QRect(160, 340, 80, 24))
        self.control_tabs.addTab(self.roll_tab, "")
        self.chimney_tab = QWidget()
        self.chimney_tab.setObjectName(u"chimney_tab")
        self.control_tabs.addTab(self.chimney_tab, "")
        self.stickbug_tab = QWidget()
        self.stickbug_tab.setObjectName(u"stickbug_tab")
        self.verticalLayout = QVBoxLayout(self.stickbug_tab)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.sb_button = QPushButton(self.stickbug_tab)
        self.sb_button.setObjectName(u"sb_button")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sb_button.sizePolicy().hasHeightForWidth())
        self.sb_button.setSizePolicy(sizePolicy)
        self.sb_button.setMinimumSize(QSize(0, 100))
        font = QFont()
        font.setPointSize(36)
        self.sb_button.setFont(font)

        self.verticalLayout.addWidget(self.sb_button)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label = QLabel(self.stickbug_tab)
        self.label.setObjectName(u"label")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy1)

        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.sb_queue = QLCDNumber(self.stickbug_tab)
        self.sb_queue.setObjectName(u"sb_queue")

        self.gridLayout.addWidget(self.sb_queue, 1, 1, 1, 1)

        self.sb_viciousness = QSlider(self.stickbug_tab)
        self.sb_viciousness.setObjectName(u"sb_viciousness")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.sb_viciousness.sizePolicy().hasHeightForWidth())
        self.sb_viciousness.setSizePolicy(sizePolicy2)
        self.sb_viciousness.setMinimum(1)
        self.sb_viciousness.setMaximum(10)
        self.sb_viciousness.setOrientation(Qt.Orientation.Horizontal)
        self.sb_viciousness.setInvertedAppearance(False)
        self.sb_viciousness.setInvertedControls(False)
        self.sb_viciousness.setTickPosition(QSlider.TickPosition.TicksAbove)
        self.sb_viciousness.setTickInterval(1)

        self.gridLayout.addWidget(self.sb_viciousness, 1, 0, 1, 1)

        self.gridLayout.setRowStretch(1, 1)
        self.gridLayout.setColumnStretch(0, 1)

        self.verticalLayout.addLayout(self.gridLayout)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.control_tabs.addTab(self.stickbug_tab, "")
        self.Bottom_Splitter.addWidget(self.control_tabs)
        self.verticalLayoutWidget = QWidget(self.Bottom_Splitter)
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
        self.term.setEnabled(False)
        self.term.setReadOnly(True)

        self.L2.addWidget(self.term)

        self.Bottom_Splitter.addWidget(self.verticalLayoutWidget)
        self.Full_Splitter.addWidget(self.Bottom_Splitter)

        self.verticalLayout_2.addWidget(self.Full_Splitter)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1000, 21))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.control_tabs.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Node", None))
        self.pushButton_2.setText("")
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Position", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"X Axis", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Position", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Y Axis", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Position", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Z Axis", None))
        self.pushButton_14.setText("")
        self.pushButton_15.setText("")
        self.pushButton_3.setText("")
        self.pushButton_16.setText("")
        self.pushButton_4.setText("")
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.node_tab), QCoreApplication.translate("MainWindow", u"Node Control", None))
        self.pushButton_7.setText("")
        self.pushButton_8.setText("")
        self.pushButton_9.setText("")
        self.pushButton_10.setText("")
        self.pushButton_11.setText(QCoreApplication.translate("MainWindow", u"Stickbug", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Speed", None))
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.crawl_tab), QCoreApplication.translate("MainWindow", u"Crawling", None))
        self.pushButton_12.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pushButton_13.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.roll_tab), QCoreApplication.translate("MainWindow", u"Rolling", None))
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.chimney_tab), QCoreApplication.translate("MainWindow", u"Chimney Climbing", None))
        self.sb_button.setText(QCoreApplication.translate("MainWindow", u"Stickbug", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Stickbug Viciousness", None))
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.stickbug_tab), QCoreApplication.translate("MainWindow", u"Stickbug", None))
        self.bot_toggle.setText(QCoreApplication.translate("MainWindow", u"Connect Robot", None))
        self.bot_label.setText(QCoreApplication.translate("MainWindow", u"Robot Offline", None))
        self.js_toggle.setText(QCoreApplication.translate("MainWindow", u"Connect Joystick", None))
        self.js_label.setText(QCoreApplication.translate("MainWindow", u"Joystick Offline", None))
        self.sim_toggle.setText(QCoreApplication.translate("MainWindow", u"Begin Simulation", None))
        self.sim_label.setText(QCoreApplication.translate("MainWindow", u"Simulation Offine", None))
    # retranslateUi

