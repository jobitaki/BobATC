import cocotb 
from cocotb.triggers import *
from cocotb.clock import Clock
from cocotb.utils import get_sim_time
import random

T_REQUEST    = 0b000
T_DECLARE    = 0b001
T_EMERGENCY  = 0b010
T_CLEAR      = 0b011
T_HOLD       = 0b100
T_SAY_AGAIN  = 0b101
T_DIVERT     = 0b110
T_ID_PLEASE  = 0b111

C_TAKEOFF_0   = 0b00 # Clear takeoff
C_TAKEOFF_1   = 0b01
C_LANDING_0   = 0b10 # Clear landing
C_LANDING_1   = 0b11

R_TAKEOFF    = 0b00 # Request
R_LANDING    = 0b10

D_TAKEOFF_0  = 0b00 # Declare takeoff
D_TAKEOFF_1  = 0b01
D_LANDING_0  = 0b10 # Declare landing
D_LANDING_1  = 0b11

async def send_uart_request(dut, data):
  dut.uart_rx_data.value = data # 0101 Declare takeoff 0
  dut.uart_rx_valid.value = True
  await FallingEdge(dut.clock)
  dut.uart_rx_valid.value = False

def detect_uart_reply(dut, expected):
  reply = dut.uart_tx_data.value
  if dut.uart_tx_send == 0b1:
    if (reply & 0b000011100) == 0b011 << 2:
      if (reply & 0b000000010) == 0b00:
        print(f"Bob     : Plane {"{:02d}".format(reply >> 5)} cleared to takeoff runway {reply & 0b1}")
      elif (reply & 0b000000010) == 0b10:
        print(f"Bob     : Plane {"{:02d}".format(reply >> 5)} cleared to land runway {reply & 0b1}")
    if (reply & 0b000011100) == 0b100 << 2:
        print(f"Bob     : Plane {"{:02d}".format(reply >> 5)} hold")
        
  return reply == expected

@cocotb.test()
async def basic_test(dut):
  print("////////////////////////////////////////")
  print("//         Begin basic tests          //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, 100, units="ns").start())

  dut.reset.value = True
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)

  dut.uart_tx_ready.value = True # UART always ready to transmit

  # Plane id_1 requests takeoff
  id_1 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_1 + (T_REQUEST << 2) + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_1 >> 5)}: Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if detect_uart_reply(dut, id_1 + (T_CLEAR << 2) + C_TAKEOFF_0):
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Takeoff request success! //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)

  # Plane id_2 requests landing
  id_2 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_2 + (T_REQUEST << 2) + R_LANDING)
  start = get_sim_time(units = "ns")
  print(f"Plane {"{:02d}".format(id_2 >> 5)}: Requesting landing at time {start}")
  
  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if detect_uart_reply(dut, id_2 + (T_CLEAR << 2) + C_LANDING_1):
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Landing request success! //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)

  # Plane id_3 requests takeoff
  id_3 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_3 + (T_REQUEST << 2) + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_3 >> 5)}: Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive hold")
    if detect_uart_reply(dut, id_3 + (T_HOLD << 2)):
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Received hold success!   //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)

  # Plane id_1 declares takeoff runway 0
  await send_uart_request(dut, id_1 + (T_DECLARE << 2) + D_TAKEOFF_0)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_1 >> 5)}: Declaring takeoff on runway 0 at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive takeoff clearance")
    if detect_uart_reply(dut, id_3 + (T_CLEAR << 2) + C_TAKEOFF_0):
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Takeoff request success! //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)

  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)