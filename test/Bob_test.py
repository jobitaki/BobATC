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

  rand_id = (random.randint(0, 15) << 5)

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, 100, units="ns").start())

  dut.reset_n.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset_n.value = True

  dut.uart_tx_ready.value = True
  await send_uart_request(dut, rand_id + T_REQUEST + R_TAKEOFF)
  start = get_sim_time(units="ns")
  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if dut.uart_tx_data.value == rand_id + T_CLEAR + C_RUNWAY_0: # Clear takeoff 0
      print(f"Received clearance for takeoff ID 5 at time {get_sim_time(units="ns")}")
      break
  await FallingEdge(dut.clock)
  await send_uart_request(dut, 0b010100100)
  for i in range(100):
    await FallingEdge(dut.clock)