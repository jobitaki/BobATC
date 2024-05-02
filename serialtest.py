import serial
import threading
from time import sleep

serial_port = serial.Serial()

def read():
    while True:
        data = serial_port.read(1);
        print("hello")
        if len(data) > 0:
            print('Got:', data)

        sleep(0.5)
        print('not blocked')

def main():
    serial_port.baudrate = 9600
    serial_port.port = '/dev/cu.usbserial-A10MPCQ8'
    serial_port.timeout = 0
    if serial_port.isOpen(): serial_port.close()
    serial_port.open()
    print(serial_port.name)
    t1 = threading.Thread(target=read, args=())
    for i in range(0b11111111):
      serial_port.write(bytes([0x0A]))
      print(i)
      sleep(0.5)
    serial_port.close()
  
main()
