from PySide6.QtWidgets import (QApplication, QMainWindow, QMenuBar, QPushButton,
    QSizePolicy, QStatusBar, QWidget)
import sys
import time
import serial

from exoesqueleto import Ui_MainWindow

Maxsteps = 3000
steps_per_angle = 25 
angle = 0

def calc_steps(angle):
    steps = Maxsteps - ( steps_per_angle * angle )
    steps = int(steps / 25)
    steps = hex(steps)
    value = str(steps)[2:]
    return value

class Exoesqueleto(QMainWindow):

    
    def __init__(self):
        super().__init__()
        self.ser = None
        # por cada angulo que nos desplacemos nos tomara 25 milisegundos llegar en la maxima velocidad
        self.speedval = 1 # velocidad de 1 milisegundos por paso cada angulo son 25 pasos por lo tanto
        self.speedtimer = 4
        self.reps = 1
        self.initialposition = 0
        self.finalposition = 0
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
        self.ui.mvdwn_bt.setEnabled(False)
        self.ui.mvup_bt.clicked.connect(self.move_up)
        self.ui.mvup_bt.setEnabled(False)
        self.ui.home_bt.clicked.connect(self.home)
        self.ui.home_bt.setEnabled(False)
        self.ui.bluetooth_bt.clicked.connect(self.connect_bluetooth)
        self.ui.indx_bt.clicked.connect(self.index)
        self.ui.indx_bt.setEnabled(False)
        self.ui.thumb_bt.clicked.connect(self.thumb)
        self.ui.thumb_bt.setEnabled(False)
        self.ui.middle_bt.clicked.connect(self.middle)
        self.ui.middle_bt.setEnabled(False)
        self.ui.ring_bt.clicked.connect(self.ring)
        self.ui.ring_bt.setEnabled(False)
        self.ui.little_bt.clicked.connect(self.little)
        self.ui.little_bt.setEnabled(False)
        self.ui.speed_slider.valueChanged.connect(self.speed)
        self.ui.speed_slider.setEnabled(False)
        self.ui.stop_boton.clicked.connect(self.stopped)
        self.ui.stop_boton.setEnabled(False)
        self.ui.start_boton.clicked.connect(self.start)
        self.ui.start_boton.setEnabled(False)
        self.ui.spinb_reps.valueChanged.connect(self.repetition)
        self.ui.spinb_initpos.valueChanged.connect(self.initpos)
        self.ui.spinb_finalpos.valueChanged.connect(self.finalpos)
        self.speedcmd=["FB","FC","FD","FE","FF"]
       
    def repetition(self, value):
        self.reps = value 

    def initpos(self, value):
        self.initialposition = value

    def finalpos(self, value):
        self.finalposition = value

    def updateSpeedTimer(self):
        self.speedtimer = ((self.speedval * 25 * abs(self.finalposition - self.initialposition))/1000)+.5

    def start(self):
        print("\nresumen de rutina\n")
        print("posicion inicial: " + str(self.initialposition))
        print("posicion final: " + str(self.finalposition))
        print("Repeticiones: " + str(self.reps))
        print("Iniciando rutina")
        init = calc_steps(int(self.initialposition))
        final = calc_steps(int(self.finalposition))
        self.updateSpeedTimer()
        self.routine(init, final)

    def gotoposition(self, position):
        self.ser.write(bytes.fromhex("05"))
        time.sleep(.5)
        self.ser.write(bytes.fromhex(position))

    def routine(self, init, final):
        for i in range(self.reps):
            #print("down")
            self.gotoposition(init)
            time.sleep(self.speedtimer)
            self.gotoposition(final)
            time.sleep(self.speedtimer)
        print(self.ser)

    def stopped (self):
        print("stop")
        self.ser.write(bytes.fromhex("7F"))
        print(self.ser)

    def speed(self, value):
        self.speedval = value
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
        self.ser = serial.Serial(port='COM11', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.ui.mvdwn_bt.setEnabled(False)
        self.ui.mvup_bt.setEnabled(False)
        self.ui.home_bt.setEnabled(False)
        self.ui.indx_bt.setEnabled(False)
        self.ui.thumb_bt.setEnabled(False)
        self.ui.middle_bt.setEnabled(False)
        self.ui.ring_bt.setEnabled(False)
        self.ui.little_bt.setEnabled(False)
        self.ui.speed_slider.setEnabled(False)
        self.ui.stop_boton.setEnabled(False)
        self.ui.start_boton.setEnabled(False)
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
        for i in range(self.reps):
            #print("down")
            self.ser.write(bytes.fromhex("05"))
            self.ser.write(bytes.fromhex("78"))
            time.sleep(4)
            self.ser.write(bytes.fromhex("05"))
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


