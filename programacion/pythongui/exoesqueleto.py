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
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(724, 592)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mvdwn_bt = QPushButton(self.centralwidget)
        self.mvdwn_bt.setObjectName(u"mvdwn_bt")
        self.mvdwn_bt.setGeometry(QRect(20, 20, 181, 111))
        icon = QIcon()
        icon.addFile(u"fdown.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvdwn_bt.setIcon(icon)
        self.mvdwn_bt.setIconSize(QSize(50, 50))
        self.mvup_bt = QPushButton(self.centralwidget)
        self.mvup_bt.setObjectName(u"mvup_bt")
        self.mvup_bt.setGeometry(QRect(20, 140, 181, 111))
        icon1 = QIcon()
        icon1.addFile(u"fup.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvup_bt.setIcon(icon1)
        self.mvup_bt.setIconSize(QSize(50, 50))
        self.thumb_bt = QPushButton(self.centralwidget)
        self.thumb_bt.setObjectName(u"thumb_bt")
        self.thumb_bt.setGeometry(QRect(220, 20, 81, 41))
        self.thumb_bt.setIconSize(QSize(50, 50))
        self.indx_bt = QPushButton(self.centralwidget)
        self.indx_bt.setObjectName(u"indx_bt")
        self.indx_bt.setGeometry(QRect(320, 20, 81, 41))
        self.indx_bt.setIconSize(QSize(50, 50))
        self.home_bt = QPushButton(self.centralwidget)
        self.home_bt.setObjectName(u"home_bt")
        self.home_bt.setGeometry(QRect(20, 270, 181, 111))
        icon2 = QIcon()
        icon2.addFile(u"home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home_bt.setIcon(icon2)
        self.home_bt.setIconSize(QSize(50, 50))
        self.bluetooth_bt = QPushButton(self.centralwidget)
        self.bluetooth_bt.setObjectName(u"bluetooth_bt")
        self.bluetooth_bt.setGeometry(QRect(10, 450, 201, 91))
        icon3 = QIcon()
        icon3.addFile(u"bt.png", QSize(), QIcon.Normal, QIcon.Off)
        self.bluetooth_bt.setIcon(icon3)
        self.bluetooth_bt.setIconSize(QSize(50, 50))
        self.middle_bt = QPushButton(self.centralwidget)
        self.middle_bt.setObjectName(u"middle_bt")
        self.middle_bt.setGeometry(QRect(420, 20, 81, 41))
        self.middle_bt.setIconSize(QSize(50, 50))
        self.ring_bt = QPushButton(self.centralwidget)
        self.ring_bt.setObjectName(u"ring_bt")
        self.ring_bt.setGeometry(QRect(520, 20, 81, 41))
        self.ring_bt.setIconSize(QSize(50, 50))
        self.little_bt = QPushButton(self.centralwidget)
        self.little_bt.setObjectName(u"little_bt")
        self.little_bt.setGeometry(QRect(620, 20, 81, 41))
        self.little_bt.setIconSize(QSize(50, 50))
        self.label_hand = QLabel(self.centralwidget)
        self.label_hand.setObjectName(u"label_hand")
        self.label_hand.setGeometry(QRect(300, 150, 301, 401))
        self.label_hand.setPixmap(QPixmap(u"left_hand.png"))
        self.label_hand.setScaledContents(True)
        self.label_thumb = QLabel(self.centralwidget)
        self.label_thumb.setObjectName(u"label_thumb")
        self.label_thumb.setGeometry(QRect(290, 180, 31, 51))
        self.label_thumb.setPixmap(QPixmap(u"fire.png"))
        self.label_thumb.setScaledContents(True)
        self.label_index = QLabel(self.centralwidget)
        self.label_index.setObjectName(u"label_index")
        self.label_index.setGeometry(QRect(440, 90, 31, 51))
        self.label_index.setPixmap(QPixmap(u"fire.png"))
        self.label_index.setScaledContents(True)
        self.label_middle = QLabel(self.centralwidget)
        self.label_middle.setObjectName(u"label_middle")
        self.label_middle.setGeometry(QRect(520, 110, 31, 51))
        self.label_middle.setPixmap(QPixmap(u"fire.png"))
        self.label_middle.setScaledContents(True)
        self.label_ring = QLabel(self.centralwidget)
        self.label_ring.setObjectName(u"label_ring")
        self.label_ring.setGeometry(QRect(560, 140, 31, 51))
        self.label_ring.setPixmap(QPixmap(u"fire.png"))
        self.label_ring.setScaledContents(True)
        self.label_little = QLabel(self.centralwidget)
        self.label_little.setObjectName(u"label_little")
        self.label_little.setGeometry(QRect(590, 200, 31, 51))
        self.label_little.setPixmap(QPixmap(u"fire.png"))
        self.label_little.setScaledContents(True)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 724, 22))
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
        self.thumb_bt.setText(QCoreApplication.translate("MainWindow", u"Pulgar", None))
        self.indx_bt.setText(QCoreApplication.translate("MainWindow", u"Indice", None))
        self.home_bt.setText(QCoreApplication.translate("MainWindow", u"Referencia", None))
        self.bluetooth_bt.setText(QCoreApplication.translate("MainWindow", u"Conectar Exo-Esqueleto", None))
        self.middle_bt.setText(QCoreApplication.translate("MainWindow", u"Medio", None))
        self.ring_bt.setText(QCoreApplication.translate("MainWindow", u"Anular", None))
        self.little_bt.setText(QCoreApplication.translate("MainWindow", u"Menique", None))
        self.label_hand.setText("")
        self.label_thumb.setText("")
        self.label_index.setText("")
        self.label_middle.setText("")
        self.label_ring.setText("")
        self.label_little.setText("")
    # retranslateUi

