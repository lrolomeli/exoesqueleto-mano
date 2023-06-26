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
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.mvdwn_bt.clicked.connect(self.move_down)
        self.ui.mvup_bt.clicked.connect(self.move_up)
        self.ui.rock_bt.clicked.connect(self.rock)
        self.ui.elgnt_bt.clicked.connect(self.elegant)
        self.ui.home_bt.clicked.connect(self.home)
        self.ui.bluetooth_bt.clicked.connect(self.connect_bluetooth)
        
    def closeEvent(self, event):
        if self.ser is not None:
            print(self.ser)
            self.ser.close()
            print(self.ser)
        print("hola")
        event.accept() # let the window close

    def connect_bluetooth(self):
        self.ser = serial.Serial(port='COM3', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        print("connected")
    def rock(self):
        print("rock")
        h = 'h'
        f = 'f'
        b = 'b'
        self.ser.write(h.encode())
        time.sleep(4)
        self.ser.write(f.encode())
        time.sleep(1)
        self.ser.write(f.encode())
        time.sleep(1)
        self.ser.write(f.encode())
        time.sleep(1)
        self.ser.write(b.encode())
        print(self.ser)



    def move_up(self):
        print("up")
        b = 'b'
        self.ser.write(b.encode())
        time.sleep(1)
        print(self.ser)
    
    def move_down(self):
        print("down")
        f = 'f'
        self.ser.write(f.encode())
        time.sleep(1)
        print(self.ser)

    def home(self):
        print("home")
        h = 'h'
        self.ser.write(h.encode())
        time.sleep(4)
        print(self.ser)

    def elegant(self):
        print("elegant")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Exoesqueleto()
    win.show()
    app.exec()


