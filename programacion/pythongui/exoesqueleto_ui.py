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
    QPushButton, QSizePolicy, QSlider, QSpinBox,
    QStatusBar, QTabWidget, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(743, 462)
        icon = QIcon()
        icon.addFile(u"exoesk.png", QSize(), QIcon.Normal, QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setTabShape(QTabWidget.Rounded)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mvdwn_bt = QPushButton(self.centralwidget)
        self.mvdwn_bt.setObjectName(u"mvdwn_bt")
        self.mvdwn_bt.setGeometry(QRect(690, 320, 41, 51))
        icon1 = QIcon()
        icon1.addFile(u"fdown.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvdwn_bt.setIcon(icon1)
        self.mvdwn_bt.setIconSize(QSize(50, 50))
        self.mvup_bt = QPushButton(self.centralwidget)
        self.mvup_bt.setObjectName(u"mvup_bt")
        self.mvup_bt.setGeometry(QRect(690, 270, 41, 51))
        icon2 = QIcon()
        icon2.addFile(u"fup.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvup_bt.setIcon(icon2)
        self.mvup_bt.setIconSize(QSize(50, 50))
        self.thumb_bt = QPushButton(self.centralwidget)
        self.thumb_bt.setObjectName(u"thumb_bt")
        self.thumb_bt.setGeometry(QRect(480, 60, 51, 31))
        self.thumb_bt.setIconSize(QSize(50, 50))
        self.indx_bt = QPushButton(self.centralwidget)
        self.indx_bt.setObjectName(u"indx_bt")
        self.indx_bt.setGeometry(QRect(480, 20, 51, 31))
        self.indx_bt.setIconSize(QSize(50, 50))
        self.home_bt = QPushButton(self.centralwidget)
        self.home_bt.setObjectName(u"home_bt")
        self.home_bt.setGeometry(QRect(690, 370, 41, 41))
        icon3 = QIcon()
        icon3.addFile(u"home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home_bt.setIcon(icon3)
        self.home_bt.setIconSize(QSize(20, 20))
        self.bluetooth_bt = QPushButton(self.centralwidget)
        self.bluetooth_bt.setObjectName(u"bluetooth_bt")
        self.bluetooth_bt.setGeometry(QRect(10, 380, 91, 31))
        icon4 = QIcon()
        icon4.addFile(u"bt.png", QSize(), QIcon.Normal, QIcon.Off)
        self.bluetooth_bt.setIcon(icon4)
        self.bluetooth_bt.setIconSize(QSize(20, 50))
        self.middle_bt = QPushButton(self.centralwidget)
        self.middle_bt.setObjectName(u"middle_bt")
        self.middle_bt.setGeometry(QRect(540, 20, 51, 31))
        self.middle_bt.setIconSize(QSize(50, 50))
        self.ring_bt = QPushButton(self.centralwidget)
        self.ring_bt.setObjectName(u"ring_bt")
        self.ring_bt.setGeometry(QRect(600, 20, 51, 31))
        self.ring_bt.setIconSize(QSize(50, 50))
        self.little_bt = QPushButton(self.centralwidget)
        self.little_bt.setObjectName(u"little_bt")
        self.little_bt.setGeometry(QRect(660, 20, 61, 31))
        self.little_bt.setIconSize(QSize(50, 50))
        self.label_hand = QLabel(self.centralwidget)
        self.label_hand.setObjectName(u"label_hand")
        self.label_hand.setGeometry(QRect(500, 130, 181, 281))
        self.label_hand.setPixmap(QPixmap(u"left_hand.png"))
        self.label_hand.setScaledContents(True)
        self.label_thumb = QLabel(self.centralwidget)
        self.label_thumb.setObjectName(u"label_thumb")
        self.label_thumb.setGeometry(QRect(490, 150, 21, 31))
        self.label_thumb.setPixmap(QPixmap(u"fire.png"))
        self.label_thumb.setScaledContents(True)
        self.label_index = QLabel(self.centralwidget)
        self.label_index.setObjectName(u"label_index")
        self.label_index.setGeometry(QRect(580, 90, 21, 31))
        self.label_index.setPixmap(QPixmap(u"fire.png"))
        self.label_index.setScaledContents(True)
        self.label_middle = QLabel(self.centralwidget)
        self.label_middle.setObjectName(u"label_middle")
        self.label_middle.setGeometry(QRect(630, 100, 21, 31))
        self.label_middle.setPixmap(QPixmap(u"fire.png"))
        self.label_middle.setScaledContents(True)
        self.label_ring = QLabel(self.centralwidget)
        self.label_ring.setObjectName(u"label_ring")
        self.label_ring.setGeometry(QRect(660, 120, 21, 31))
        self.label_ring.setPixmap(QPixmap(u"fire.png"))
        self.label_ring.setScaledContents(True)
        self.label_little = QLabel(self.centralwidget)
        self.label_little.setObjectName(u"label_little")
        self.label_little.setGeometry(QRect(680, 160, 21, 31))
        self.label_little.setPixmap(QPixmap(u"fire.png"))
        self.label_little.setScaledContents(True)
        self.speed_slider = QSlider(self.centralwidget)
        self.speed_slider.setObjectName(u"speed_slider")
        self.speed_slider.setGeometry(QRect(180, 210, 161, 21))
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(5)
        self.speed_slider.setPageStep(1)
        self.speed_slider.setOrientation(Qt.Horizontal)
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 200, 121, 41))
        font = QFont()
        font.setPointSize(18)
        self.label.setFont(font)
        self.speed_label = QLabel(self.centralwidget)
        self.speed_label.setObjectName(u"speed_label")
        self.speed_label.setGeometry(QRect(140, 240, 121, 31))
        font1 = QFont()
        font1.setPointSize(16)
        font1.setBold(True)
        self.speed_label.setFont(font1)
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 10, 161, 51))
        self.label_3.setFont(font)
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 140, 151, 41))
        self.label_2.setFont(font)
        self.stop_boton = QPushButton(self.centralwidget)
        self.stop_boton.setObjectName(u"stop_boton")
        self.stop_boton.setGeometry(QRect(280, 270, 161, 131))
        icon5 = QIcon()
        icon5.addFile(u"stop.png", QSize(), QIcon.Normal, QIcon.Off)
        self.stop_boton.setIcon(icon5)
        self.stop_boton.setIconSize(QSize(200, 100))
        self.stop_boton.setAutoRepeat(True)
        self.stop_boton.setFlat(False)
        self.start_boton = QPushButton(self.centralwidget)
        self.start_boton.setObjectName(u"start_boton")
        self.start_boton.setGeometry(QRect(290, 20, 111, 91))
        icon6 = QIcon()
        icon6.addFile(u"start-button.jpg", QSize(), QIcon.Normal, QIcon.Off)
        self.start_boton.setIcon(icon6)
        self.start_boton.setIconSize(QSize(100, 100))
        self.start_boton.setAutoRepeat(True)
        self.spinb_initpos = QSpinBox(self.centralwidget)
        self.spinb_initpos.setObjectName(u"spinb_initpos")
        self.spinb_initpos.setGeometry(QRect(190, 20, 71, 31))
        self.spinb_initpos.setMaximum(120)
        self.spinb_finalpos = QSpinBox(self.centralwidget)
        self.spinb_finalpos.setObjectName(u"spinb_finalpos")
        self.spinb_finalpos.setGeometry(QRect(190, 80, 71, 31))
        self.spinb_finalpos.setMaximum(120)
        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 70, 161, 51))
        self.label_4.setFont(font)
        self.spinb_reps = QSpinBox(self.centralwidget)
        self.spinb_reps.setObjectName(u"spinb_reps")
        self.spinb_reps.setGeometry(QRect(190, 140, 71, 31))
        self.spinb_reps.setMinimum(1)
        self.spinb_reps.setMaximum(10)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 743, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.stop_boton.setDefault(False)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Exo-esqueleto", None))
        self.mvdwn_bt.setText("")
        self.mvup_bt.setText("")
        self.thumb_bt.setText(QCoreApplication.translate("MainWindow", u"Pulgar", None))
        self.indx_bt.setText(QCoreApplication.translate("MainWindow", u"Indice", None))
        self.home_bt.setText("")
        self.bluetooth_bt.setText(QCoreApplication.translate("MainWindow", u"Conectar", None))
        self.middle_bt.setText(QCoreApplication.translate("MainWindow", u"Medio", None))
        self.ring_bt.setText(QCoreApplication.translate("MainWindow", u"Anular", None))
        self.little_bt.setText(QCoreApplication.translate("MainWindow", u"Menique", None))
        self.label_hand.setText("")
        self.label_thumb.setText("")
        self.label_index.setText("")
        self.label_middle.setText("")
        self.label_ring.setText("")
        self.label_little.setText("")
        self.label.setText(QCoreApplication.translate("MainWindow", u"Velocidad", None))
        self.speed_label.setText(QCoreApplication.translate("MainWindow", u"Muy Alta", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"1era Posicion \u00b0", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Repeticiones", None))
        self.stop_boton.setText("")
        self.start_boton.setText("")
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"2da Posicion \u00b0", None))
    # retranslateUi

