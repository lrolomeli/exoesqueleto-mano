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
    QPushButton, QSizePolicy, QSlider, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(775, 554)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mvdwn_bt = QPushButton(self.centralwidget)
        self.mvdwn_bt.setObjectName(u"mvdwn_bt")
        self.mvdwn_bt.setGeometry(QRect(330, 140, 61, 51))
        icon = QIcon()
        icon.addFile(u"fdown.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvdwn_bt.setIcon(icon)
        self.mvdwn_bt.setIconSize(QSize(50, 50))
        self.mvup_bt = QPushButton(self.centralwidget)
        self.mvup_bt.setObjectName(u"mvup_bt")
        self.mvup_bt.setGeometry(QRect(330, 80, 61, 51))
        icon1 = QIcon()
        icon1.addFile(u"fup.png", QSize(), QIcon.Normal, QIcon.Off)
        self.mvup_bt.setIcon(icon1)
        self.mvup_bt.setIconSize(QSize(50, 50))
        self.thumb_bt = QPushButton(self.centralwidget)
        self.thumb_bt.setObjectName(u"thumb_bt")
        self.thumb_bt.setGeometry(QRect(330, 20, 81, 41))
        self.thumb_bt.setIconSize(QSize(50, 50))
        self.indx_bt = QPushButton(self.centralwidget)
        self.indx_bt.setObjectName(u"indx_bt")
        self.indx_bt.setGeometry(QRect(420, 20, 81, 41))
        self.indx_bt.setIconSize(QSize(50, 50))
        self.home_bt = QPushButton(self.centralwidget)
        self.home_bt.setObjectName(u"home_bt")
        self.home_bt.setGeometry(QRect(20, 10, 81, 61))
        icon2 = QIcon()
        icon2.addFile(u"home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home_bt.setIcon(icon2)
        self.home_bt.setIconSize(QSize(50, 50))
        self.bluetooth_bt = QPushButton(self.centralwidget)
        self.bluetooth_bt.setObjectName(u"bluetooth_bt")
        self.bluetooth_bt.setGeometry(QRect(110, 10, 131, 61))
        icon3 = QIcon()
        icon3.addFile(u"bt.png", QSize(), QIcon.Normal, QIcon.Off)
        self.bluetooth_bt.setIcon(icon3)
        self.bluetooth_bt.setIconSize(QSize(50, 50))
        self.middle_bt = QPushButton(self.centralwidget)
        self.middle_bt.setObjectName(u"middle_bt")
        self.middle_bt.setGeometry(QRect(510, 20, 81, 41))
        self.middle_bt.setIconSize(QSize(50, 50))
        self.ring_bt = QPushButton(self.centralwidget)
        self.ring_bt.setObjectName(u"ring_bt")
        self.ring_bt.setGeometry(QRect(600, 20, 81, 41))
        self.ring_bt.setIconSize(QSize(50, 50))
        self.little_bt = QPushButton(self.centralwidget)
        self.little_bt.setObjectName(u"little_bt")
        self.little_bt.setGeometry(QRect(690, 20, 81, 41))
        self.little_bt.setIconSize(QSize(50, 50))
        self.label_hand = QLabel(self.centralwidget)
        self.label_hand.setObjectName(u"label_hand")
        self.label_hand.setGeometry(QRect(450, 140, 301, 401))
        self.label_hand.setPixmap(QPixmap(u"left_hand.png"))
        self.label_hand.setScaledContents(True)
        self.label_thumb = QLabel(self.centralwidget)
        self.label_thumb.setObjectName(u"label_thumb")
        self.label_thumb.setGeometry(QRect(440, 160, 31, 51))
        self.label_thumb.setPixmap(QPixmap(u"fire.png"))
        self.label_thumb.setScaledContents(True)
        self.label_index = QLabel(self.centralwidget)
        self.label_index.setObjectName(u"label_index")
        self.label_index.setGeometry(QRect(590, 80, 31, 51))
        self.label_index.setPixmap(QPixmap(u"fire.png"))
        self.label_index.setScaledContents(True)
        self.label_middle = QLabel(self.centralwidget)
        self.label_middle.setObjectName(u"label_middle")
        self.label_middle.setGeometry(QRect(660, 90, 31, 51))
        self.label_middle.setPixmap(QPixmap(u"fire.png"))
        self.label_middle.setScaledContents(True)
        self.label_ring = QLabel(self.centralwidget)
        self.label_ring.setObjectName(u"label_ring")
        self.label_ring.setGeometry(QRect(710, 120, 31, 51))
        self.label_ring.setPixmap(QPixmap(u"fire.png"))
        self.label_ring.setScaledContents(True)
        self.label_little = QLabel(self.centralwidget)
        self.label_little.setObjectName(u"label_little")
        self.label_little.setGeometry(QRect(740, 180, 31, 51))
        self.label_little.setPixmap(QPixmap(u"fire.png"))
        self.label_little.setScaledContents(True)
        self.speed_slider = QSlider(self.centralwidget)
        self.speed_slider.setObjectName(u"speed_slider")
        self.speed_slider.setGeometry(QRect(30, 220, 211, 51))
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(5)
        self.speed_slider.setPageStep(1)
        self.speed_slider.setOrientation(Qt.Horizontal)
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(30, 290, 121, 41))
        font = QFont()
        font.setPointSize(20)
        self.label.setFont(font)
        self.speed_label = QLabel(self.centralwidget)
        self.speed_label.setObjectName(u"speed_label")
        self.speed_label.setGeometry(QRect(170, 290, 201, 31))
        font1 = QFont()
        font1.setPointSize(16)
        font1.setBold(True)
        self.speed_label.setFont(font1)
        self.angle_slider = QSlider(self.centralwidget)
        self.angle_slider.setObjectName(u"angle_slider")
        self.angle_slider.setGeometry(QRect(10, 80, 211, 51))
        self.angle_slider.setMinimum(0)
        self.angle_slider.setMaximum(90)
        self.angle_slider.setPageStep(1)
        self.angle_slider.setValue(0)
        self.angle_slider.setSliderPosition(0)
        self.angle_slider.setOrientation(Qt.Horizontal)
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 160, 121, 41))
        self.label_3.setFont(font)
        self.angle_label = QLabel(self.centralwidget)
        self.angle_label.setObjectName(u"angle_label")
        self.angle_label.setGeometry(QRect(110, 170, 201, 31))
        self.angle_label.setFont(font1)
        self.Reps_Slider = QSlider(self.centralwidget)
        self.Reps_Slider.setObjectName(u"Reps_Slider")
        self.Reps_Slider.setGeometry(QRect(30, 350, 231, 51))
        self.Reps_Slider.setMinimum(1)
        self.Reps_Slider.setMaximum(10)
        self.Reps_Slider.setPageStep(1)
        self.Reps_Slider.setValue(1)
        self.Reps_Slider.setOrientation(Qt.Horizontal)
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 410, 151, 41))
        self.label_2.setFont(font)
        self.Reps_label_2 = QLabel(self.centralwidget)
        self.Reps_label_2.setObjectName(u"Reps_label_2")
        self.Reps_label_2.setGeometry(QRect(170, 420, 201, 31))
        self.Reps_label_2.setFont(font1)
        self.Neutral_Label = QLabel(self.centralwidget)
        self.Neutral_Label.setObjectName(u"Neutral_Label")
        self.Neutral_Label.setGeometry(QRect(130, 140, 181, 71))
        self.Neutral_Label.setPixmap(QPixmap(u"../../../../../Pictures/mano_interfaz_2.png"))
        self.PN_label = QLabel(self.centralwidget)
        self.PN_label.setObjectName(u"PN_label")
        self.PN_label.setGeometry(QRect(210, 160, 101, 21))
        font2 = QFont()
        font2.setBold(True)
        self.PN_label.setFont(font2)
        self.stop_boton = QPushButton(self.centralwidget)
        self.stop_boton.setObjectName(u"stop_boton")
        self.stop_boton.setGeometry(QRect(320, 230, 101, 91))
        icon4 = QIcon()
        icon4.addFile(u"../../../../../Pictures/captura_boton.png", QSize(), QIcon.Normal, QIcon.Off)
        self.stop_boton.setIcon(icon4)
        self.stop_boton.setIconSize(QSize(100, 100))
        self.stop_boton.setAutoRepeat(True)
        self.start_boton = QPushButton(self.centralwidget)
        self.start_boton.setObjectName(u"start_boton")
        self.start_boton.setGeometry(QRect(320, 330, 101, 101))
        icon5 = QIcon()
        icon5.addFile(u"../../imagenes/boton_start.jpg", QSize(), QIcon.Normal, QIcon.Off)
        self.start_boton.setIcon(icon5)
        self.start_boton.setIconSize(QSize(100, 100))
        self.start_boton.setAutoRepeat(True)
        MainWindow.setCentralWidget(self.centralwidget)
        self.Neutral_Label.raise_()
        self.mvdwn_bt.raise_()
        self.mvup_bt.raise_()
        self.thumb_bt.raise_()
        self.indx_bt.raise_()
        self.home_bt.raise_()
        self.bluetooth_bt.raise_()
        self.middle_bt.raise_()
        self.ring_bt.raise_()
        self.little_bt.raise_()
        self.label_hand.raise_()
        self.label_thumb.raise_()
        self.label_index.raise_()
        self.label_middle.raise_()
        self.label_ring.raise_()
        self.label_little.raise_()
        self.speed_slider.raise_()
        self.label.raise_()
        self.speed_label.raise_()
        self.angle_slider.raise_()
        self.label_3.raise_()
        self.angle_label.raise_()
        self.Reps_Slider.raise_()
        self.label_2.raise_()
        self.Reps_label_2.raise_()
        self.PN_label.raise_()
        self.stop_boton.raise_()
        self.start_boton.raise_()
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 775, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

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
        self.label.setText(QCoreApplication.translate("MainWindow", u"Velocidad:", None))
        self.speed_label.setText(QCoreApplication.translate("MainWindow", u"Muy Alta", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Angulo", None))
        self.angle_label.setText(QCoreApplication.translate("MainWindow", u"0\u00b0", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Repeticiones", None))
        self.Reps_label_2.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.Neutral_Label.setText("")
        self.PN_label.setText(QCoreApplication.translate("MainWindow", u"posicion Neutral", None))
        self.stop_boton.setText("")
        self.start_boton.setText("")
    # retranslateUi

