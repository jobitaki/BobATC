import cocotb 
from cocotb.triggers import *
from cocotb.clock import Clock

@cocotb.test()
async def basic_test(dut):
  print("Welcome to Bob International Airport")
  print("Begin basic request sending without UART")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, 100, units="ns").start())

  dut.reset_n.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset_n.value = True

  dut.uart_rx_data.value = 0b010100000 # Aircraft 0101 is requesting takeoff
  dut.uart_rx_valid.value = True
  dut.uart_tx_ready.value = True
  await FallingEdge(dut.clock)
  dut.uart_rx_valid.value = False
  for i in range(100):
    await FallingEdge(dut.clock)
    if dut.uart_tx_data.value == 0b010101100: # 0101 Clear takeoff 0
      break
  await FallingEdge(dut.clock)
  dut.uart_rx_data.value = 0b010100100 # 0101 Declare takeoff 0
  dut.uart_rx_valid.value = True
  await FallingEdge(dut.clock)
  dut.uart_rx_valid.value = False
  for i in range(100):
    await FallingEdge(dut.clock)