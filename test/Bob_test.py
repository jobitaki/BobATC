import cocotb 
from cocotb.triggers import *
from cocotb.clock import Clock
from cocotb.utils import get_sim_time
import random

T_REQUEST    = 0b000000000
T_DECLARE    = 0b000000100
T_EMERGENCY  = 0b000001000
T_CLEAR      = 0b000001100
T_HOLD       = 0b000010000
T_SAY_AGAIN  = 0b000010100
T_DIVERT     = 0b000011000

C_RUNWAY_0   = 0b000000000
C_RUNWAY_1   = 0b000000001

R_TAKEOFF    = 0b000000000
R_LANDING    = 0b000000010

D_TAKEOFF_0  = 0b000000000
D_TAKEOFF_1  = 0b000000001
D_LANDING_0  = 0b000000010
D_LANDING_1  = 0b000000011

async def send_uart_request(dut, data):
  dut.uart_rx_data.value = data # 0101 Declare takeoff 0
  dut.uart_rx_valid.value = True
  await FallingEdge(dut.clock)
  dut.uart_rx_valid.value = False

@cocotb.test()
async def basic_test(dut):
  print("Begin basic takeoff request")

  

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, 100, units="ns").start())

  dut.reset_n.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset_n.value = True

  dut.uart_tx_ready.value = True

  # Plane id_1 requests takeoff
  id_1 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_1 + T_REQUEST + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {id_1 >> 5}: Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if (dut.uart_tx_data.value & 0b111111100) == id_1 + T_CLEAR: # Clear takeoff
      end = get_sim_time(units="ns")
      print(f"Bob: Plane {id_1 >> 5} cleared to take off runway {dut.uart_tx_data.value & 0b000000011} at time {end}")
      break
  await FallingEdge(dut.clock)

  # Plane id_2 requests landing
  id_2 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_2 + T_REQUEST + R_LANDING)
  start = get_sim_time(units = "ns")
  print(f"Plane {id_2 >> 5}: Requesting landing at time {start}")
  
  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if (dut.uart_tx_data.value & 0b111111100) == id_2 + T_CLEAR: # Clear landing
      end = get_sim_time(units="ns")
      print(f"Bob: Plane {id_2 >> 5} cleared to land runway {dut.uart_tx_data.value & 0b000000011} at time {end}")
      break
  await FallingEdge(dut.clock)

  # Plane id_3 reqeusts takeoff
  id_3 = (random.randint(0, 15) << 5)
  await send_uart_request(dut, id_3 + T_REQUEST + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {id_3 >> 5}: Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if (dut.uart_tx_data.value & 0b111111100) == id_3 + T_HOLD: # Hold
      end = get_sim_time(units="ns")
      print(f"Bob: Plane {id_3 >> 5} hold at time {end}")
      break
  await FallingEdge(dut.clock)

  # Plane id_1 declares takeoff runway 0
  await send_uart_request(dut, id_1 + T_DECLARE + D_TAKEOFF_0)
  start = get_sim_time(units="ns")
  print(f"Plane {id_1 >> 5}: Declaring takeoff on runway 0 at time {start}")

  # Await clearance of plane id_3 on now free runway
  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if (dut.uart_tx_data.value & 0b111111100) == id_3 + T_CLEAR: # Clear takeoff
      end = get_sim_time(units="ns")
      print(f"Bob: Plane {id_3 >> 5} clear to takeoff runway {dut.uart_tx_data.value & 0b000000011} at time {end}")
      break
  await FallingEdge(dut.clock)

  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)