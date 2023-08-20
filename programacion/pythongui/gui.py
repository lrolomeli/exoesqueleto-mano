from PySide6.QtWidgets import (QApplication, QMainWindow, QMenuBar, QPushButton,
    QSizePolicy, QStatusBar, QWidget)
import sys
import time
import serial

from exoesqueleto import Ui_MainWindow

class Exoesqueleto(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.ser = None
        self.speedLabel = ["Muy Alta","Alta","Intermedia","Baja","Muy Baja"]
        self.thumb_v = False
        self.index_v = False
        self.middle_v = False
        self.ring_v = False
        self.little_v = False
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.label_thumb.setVisible(False)
        self.ui.label_index.setVisible(False)
        self.ui.label_middle.setVisible(False)
        self.ui.label_ring.setVisible(False)
        self.ui.label_little.setVisible(False)
        self.ui.mvdwn_bt.clicked.connect(self.move_down)
        self.ui.mvup_bt.clicked.connect(self.move_up)
        self.ui.home_bt.clicked.connect(self.home)
        self.ui.bluetooth_bt.clicked.connect(self.connect_bluetooth)
        self.ui.indx_bt.clicked.connect(self.index)
        self.ui.thumb_bt.clicked.connect(self.thumb)
        self.ui.middle_bt.clicked.connect(self.middle)
        self.ui.ring_bt.clicked.connect(self.ring)
        self.ui.little_bt.clicked.connect(self.little)
        self.ui.speed_slider.valueChanged.connect(self.speed)
        self.ui.angle_slider.valueChanged.connect(self.gotopos)
        self.ui.Reps_Slider.valueChanged.connect(self.reps)
        self.ui.stop_boton.clicked.connect(self.stopped)
        self.speedcmd=["FB","FC","FD","FE","FF"]
       

    def reps(self, value):   
        self.ui.Reps_label_2.setText(str(value))

    def stopped (self):
        print("stop")
        self.ser.write(bytes.fromhex("7F"))
        print(self.ser)
    
         
    def gotopos(self, value):
        self.ui.angle_label.setText(str(value))
        #self.ser.write(bytes.fromhex("05"))
        #self.ser.write(bytes.fromhex(hex(value)))

    def speed(self, value):
        self.ui.speed_label.setText(self.speedLabel[value-1])
        self.ser.write(bytes.fromhex(self.speedcmd[value-1]))

    def closeEvent(self, event):
        if self.ser is not None:
            print(self.ser)
            self.ser.close()
            print(self.ser)
        print("hola")
        event.accept() # let the window close

    def connect_bluetooth(self):
        self.ser = serial.Serial(port='COM12', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        print("connected")
        
    def thumb(self):
        self.thumb_v = not self.thumb_v
        if self.thumb_v:        
            self.ser.write(bytes.fromhex("7A"))
        else:
            self.ser.write(bytes.fromhex("75"))
        self.ui.label_thumb.setVisible(self.thumb_v)

    def index(self):
        self.index_v = not self.index_v
        if self.index_v:        
            self.ser.write(bytes.fromhex("7B"))
        else:
            self.ser.write(bytes.fromhex("76"))
        self.ui.label_index.setVisible(self.index_v)

    def middle(self):
        self.middle_v = not self.middle_v
        if self.middle_v:        
            self.ser.write(bytes.fromhex("7C"))
        else:
            self.ser.write(bytes.fromhex("77"))
        self.ui.label_middle.setVisible(self.middle_v)

    def ring(self):
        self.ring_v = not self.ring_v
        if self.ring_v:        
            self.ser.write(bytes.fromhex("7D"))
        else:
            self.ser.write(bytes.fromhex("78"))
        self.ui.label_ring.setVisible(self.ring_v)

    def little(self):
        self.little_v = not self.little_v
        if self.little_v:        
            self.ser.write(bytes.fromhex("7E"))
        else:
            self.ser.write(bytes.fromhex("79"))
        self.ui.label_little.setVisible(self.little_v)

    def move_up(self):
        print("up")
        self.ser.write(bytes.fromhex("03"))
        print(self.ser)
    
    def move_down(self):
        print("down")
        self.ser.write(bytes.fromhex("02"))
        print(self.ser)

    def ciclos(self):
        for i in range(10):
            #print("down")
            self.ser.write(bytes.fromhex("02"))
            time.sleep(4)
            self.ser.write(bytes.fromhex("03"))
            time.sleep(4)
        print(self.ser)

    def home(self):
        print("home")
        self.ser.write(bytes.fromhex("01"))
        #time.sleep(4)
        print(self.ser)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Exoesqueleto()
    win.show()
    app.exec()


