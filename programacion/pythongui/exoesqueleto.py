# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'exoesqueleto.ui'
##
## Created by: Qt User Interface Compiler version 6.5.1
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
from PySide6.QtWidgets import (QApplication, QMainWindow, QMenuBar, QPushButton,
    QSizePolicy, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(474, 447)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mvdwn_bt = QPushButton(self.centralwidget)
        self.mvdwn_bt.setObjectName(u"mvdwn_bt")
        self.mvdwn_bt.setGeometry(QRect(40, 20, 181, 111))
        icon = QIcon()
        icon.addFile(u"fdown.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvdwn_bt.setIcon(icon)
        self.mvdwn_bt.setIconSize(QSize(50, 50))
        self.mvup_bt = QPushButton(self.centralwidget)
        self.mvup_bt.setObjectName(u"mvup_bt")
        self.mvup_bt.setGeometry(QRect(40, 140, 181, 111))
        icon1 = QIcon()
        icon1.addFile(u"fup.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvup_bt.setIcon(icon1)
        self.mvup_bt.setIconSize(QSize(50, 50))
        self.rock_bt = QPushButton(self.centralwidget)
        self.rock_bt.setObjectName(u"rock_bt")
        self.rock_bt.setGeometry(QRect(250, 20, 181, 111))
        icon2 = QIcon()
        icon2.addFile(u"rocki.png", QSize(), QIcon.Normal, QIcon.Off)
        self.rock_bt.setIcon(icon2)
        self.rock_bt.setIconSize(QSize(50, 50))
        self.elgnt_bt = QPushButton(self.centralwidget)
        self.elgnt_bt.setObjectName(u"elgnt_bt")
        self.elgnt_bt.setGeometry(QRect(250, 140, 181, 111))
        icon3 = QIcon()
        icon3.addFile(u"elegant.png", QSize(), QIcon.Normal, QIcon.Off)
        self.elgnt_bt.setIcon(icon3)
        self.elgnt_bt.setIconSize(QSize(50, 50))
        self.home_bt = QPushButton(self.centralwidget)
        self.home_bt.setObjectName(u"home_bt")
        self.home_bt.setGeometry(QRect(250, 260, 181, 111))
        icon4 = QIcon()
        icon4.addFile(u"home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home_bt.setIcon(icon4)
        self.home_bt.setIconSize(QSize(50, 50))
        self.bluetooth_bt = QPushButton(self.centralwidget)
        self.bluetooth_bt.setObjectName(u"bluetooth_bt")
        self.bluetooth_bt.setGeometry(QRect(20, 280, 201, 111))
        icon5 = QIcon()
        icon5.addFile(u"bt.png", QSize(), QIcon.Normal, QIcon.Off)
        self.bluetooth_bt.setIcon(icon5)
        self.bluetooth_bt.setIconSize(QSize(50, 50))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 474, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Exo-esqueleto", None))
        self.mvdwn_bt.setText(QCoreApplication.translate("MainWindow", u"Mover Abajo", None))
        self.mvup_bt.setText(QCoreApplication.translate("MainWindow", u"Mover Arriba", None))
        self.rock_bt.setText(QCoreApplication.translate("MainWindow", u"Rock", None))
        self.elgnt_bt.setText(QCoreApplication.translate("MainWindow", u"Elegancia", None))
        self.home_bt.setText(QCoreApplication.translate("MainWindow", u"Referencia", None))
        self.bluetooth_bt.setText(QCoreApplication.translate("MainWindow", u"Conectar Exo-Esqueleto", None))
    # retranslateUi

