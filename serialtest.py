import serial
from time import sleep

ser = serial.Serial('/dev/cu.usbserial-A10MPCQ8')
print(ser.name)         # check which port was really used
for i in range(10):
  sleep(1)
  ser.write(0b00001110)     # write a string
  # print(ser.read())
ser.close()             # close port
