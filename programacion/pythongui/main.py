import time
import serial

ser = serial.Serial(port='COM3', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
h = 'h'
f = 'f'
b = 'b'

ser.write(h.encode())
time.sleep(4)
ser.write(f.encode())
time.sleep(1)
ser.write(f.encode())
time.sleep(1)
ser.write(f.encode())
time.sleep(1)
ser.write(b.encode())
print(ser)


ser.close()