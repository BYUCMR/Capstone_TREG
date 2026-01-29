# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'vis.ui'
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
from PySide6.QtWidgets import (QApplication, QSizePolicy, QVBoxLayout, QWidget)

class Ui_vis_window(object):
    def setupUi(self, vis_window):
        if not vis_window.objectName():
            vis_window.setObjectName(u"vis_window")
        vis_window.resize(800, 600)
        self.layout = QVBoxLayout(vis_window)
        self.layout.setObjectName(u"layout")

        self.retranslateUi(vis_window)

        QMetaObject.connectSlotsByName(vis_window)
    # setupUi

    def retranslateUi(self, vis_window):
        vis_window.setWindowTitle(QCoreApplication.translate("vis_window", u"Visualization", None))
    # retranslateUi

