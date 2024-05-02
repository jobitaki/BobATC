import serial
import threading
from time import sleep

T_REQUEST    = 0b000
T_DECLARE    = 0b001
T_EMERGENCY  = 0b010
T_CLEAR      = 0b011
T_HOLD       = 0b100
T_SAY_AGAIN  = 0b101
T_DIVERT     = 0b110
T_ID_PLEASE  = 0b111

C_RUNWAY_0   = 0b0 # Cleared
C_RUNWAY_1   = 0b1

R_TAKEOFF    = 0b0 # Request
R_LANDING    = 0b1

D_RUNWAY_0  = 0b0 # Declare
D_RUNWAY_1  = 0b1

E_DECLARE    = 0b1
E_RESOLVE    = 0b0

ser = serial.Serial()
stop_flag = False

def initialize_serial():
  ser.baudrate = 115200
  ser.port = '/dev/cu.usbserial-A10MPCQ8'
  ser.timeout = 0
  if ser.isOpen(): ser.close()
  ser.open()
  print(f"Opened serial port at {ser.name}")

def interpret(data):
  reply = ord(data)
  print("***************************************************")
  if (reply & 0b00001110) == T_CLEAR << 1:
    if (reply & 0b00000001) == 0b0:
      print(f"Bob          : Plane {"{:02d}".format(reply >> 4)} cleared runway {reply & 0b1}")
    elif (reply & 0b00000001) == 0b1:
      print(f"Bob          : Plane {"{:02d}".format(reply >> 4)} cleared runway {reply & 0b1}")
  elif (reply & 0b00001110) == T_HOLD << 1:
    print(f"Bob          : Plane {"{:02d}".format(reply >> 4)} hold")
  elif (reply & 0b00001110) == T_ID_PLEASE << 1:
    if (reply & 0b00000001) == 0b0:
      print(f"Bob          : ID {reply >> 4} is available")
    elif (reply & 0b00000001) == 0b1:
      print(f"Bob          : My airspace is full")
  elif (reply & 0b00001110) == T_DIVERT << 1:
    print(f"Bob          : Plane {"{:02d}".format(reply >> 4)} divert due to congestion or emergency")
  elif (reply & 0b00001110) == T_SAY_AGAIN << 1:
    print(f"Bob          : Plane {"{:02d}".format(reply >> 4)} say again")
  print("***************************************************")

def translate(request):
  id = (request & 0b11110000) >> 4
  type = (request & 0b00001110) >> 1
  action = request & 0b1
  print("***************************************************")
  if type == T_ID_PLEASE:
    print(f"New Plane    : Requesting ID for entry")
  elif type == T_REQUEST:
    if action == R_TAKEOFF:
      print(f"Plane {"{:02d}".format(id)}     : Requesting takeoff")
    elif action == R_LANDING:
      print(f"Plane {"{:02d}".format(id)}     : Requesting landing")
  elif type == T_DECLARE:
    if action == D_RUNWAY_0:
      print(f"Plane {"{:02d}".format(id)}     : Declaring takeoff/landing runway 0")
    elif action == D_RUNWAY_1:
      print(f"Plane {"{:02d}".format(id)}     : Declaring takeoff/landing runway 1")
  elif type == T_EMERGENCY:
    if action == E_DECLARE:
      print(f"Plane {"{:02d}".format(id)}     : Declaring emergency")
    elif action == E_RESOLVE:
      print(f"Plane {"{:02d}".format(id)}     : Resolving emergency")
  else:
    print(f"Plane {"{:02d}".format(id)}     : Making invalid request")
  print("***************************************************")
        
def keep_reading():
  while not stop_flag:
    data = ser.read(1)
    if len(data) > 0:
      interpret(data)

# Main routine
initialize_serial()
t1 = threading.Thread(target=keep_reading, args=())
t1.start()
while True:
  id =      int(input("Plane ID     : "))
  if id == 44:
    stop_flag = True
    break
  request = int(input("Request type : "))
  action =  int(input("Action bit   : "))
  packet = (id << 4) + (request << 1) + action
  translate(packet)
  ser.write(bytes([packet]))
  sleep(0.5)
t1.join()
ser.close()
print("\nClosed serial port")