import cocotb 
from cocotb.triggers import *
from cocotb.clock import Clock
from cocotb.utils import get_sim_time

PERIOD = 6510

async def read(dut):
  data = 0b000000000
  fail = 0
  while True:
    if fail == 3:
      TimeoutError("UART read spurious start too many times")

    start = get_sim_time(units="ns")
    while dut.tx.value == 0b1:
      await FallingEdge(dut.clock)
      if start - get_sim_time(units="ns") > 10000:
        raise TimeoutError("UART read timed out")
    
    # Detect start bit
    bit_start = get_sim_time(units="ns")
    while get_sim_time(units="ns") - bit_start < PERIOD / 2:
      await FallingEdge(dut.clock)
    if dut.tx.value == 0b1:
      # Spurious start
      fail += 1
      continue
    
    # Finish start bit
    while get_sim_time(units="ns") - bit_start < PERIOD:
      await FallingEdge(dut.clock)
    
    # Collect data
    num_bits = 0
    while num_bits < 9:
      bit_start = get_sim_time(units="ns")
      while get_sim_time(units="ns") - bit_start < PERIOD / 2:
        await FallingEdge(dut.clock)
      data >>= 1
      data |= (dut.tx.value << 8)
      num_bits += 1
      while get_sim_time(units="ns") - bit_start < PERIOD:
        await FallingEdge(dut.clock)
    
    # TODO STOP bit
    bit_start = get_sim_time(units="ns")
    while get_sim_time(units="ns") - bit_start < PERIOD / 2:
      await FallingEdge(dut.clock)

    assert dut.tx.value

    bit_start = get_sim_time(units="ns")
    while get_sim_time(units="ns") - bit_start < PERIOD / 2:
      await FallingEdge(dut.clock)
    
    return data
  
async def write(dut, data):
  data_to_send = data
  bit_start = get_sim_time(units="ns")

  while get_sim_time(units="ns") - bit_start < PERIOD:
    await FallingEdge(dut.clock)
    dut.rx.value = 0

  num_bits = 0

  while num_bits < 9:
    bit_start = get_sim_time(units="ns")
    dut.rx.value = data_to_send & 1
    while get_sim_time(units="ns") - bit_start < PERIOD:
      await FallingEdge(dut.clock)
    data_to_send >>= 1
    num_bits += 1
  
  bit_start = get_sim_time(units="ns")

  while get_sim_time(units="ns") - bit_start < PERIOD:
    await FallingEdge(dut.clock)
    dut.rx.value = 1

@cocotb.test()
async def basic_test(dut):
  cocotb.start_soon(Clock(dut.clock, 40, units="ns").start())

  dut.rx.value = 1

  dut.reset.value = True
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)

  await write(dut, 0b010101101)

  dut.rx.value = 1

  await write(dut, 0b011111110)

  dut.rx.value = 1

  await FallingEdge(dut.clock)

  dut.rx.value = 0

  for i in range(5000):
    await FallingEdge(dut.clock)
  
  dut.rx.value = 1
  
  timer = get_sim_time(units="ns")
  while get_sim_time(units="ns") - timer < PERIOD * 4:
    await FallingEdge(dut.clock)

  await write(dut, 0b011111110)

  # Now test TX of UART

  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)

  dut.data_tx.value = 0b010101101
  dut.send.value = True
  assert dut.ready.value
  await FallingEdge(dut.clock)
  dut.send.value = False
  assert ~dut.ready.value

  data = await read(dut)

  timer = get_sim_time(units="ns")
  while get_sim_time(units="ns") - timer < PERIOD * 4:
    await FallingEdge(dut.clock)

  return 0

@cocotb.test()
async def tx_test(dut):
  # Exhaustive test of TX
  cocotb.start_soon(Clock(dut.clock, 40, units="ns").start())

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  for i in range(512):
    dut.data_tx.value = i
    dut.send.value = True
    assert dut.ready.value
    await FallingEdge(dut.clock)
    dut.send.value = False
    assert ~dut.ready.value

    assert await read(dut) == i
    
    timer = get_sim_time(units="ns")
    while dut.ready.value != 1:
      await FallingEdge(dut.clock)
      if (get_sim_time(units="ns") - timer) > 1000000:
        raise TimeoutError("TX took too long to get ready")

  timer = get_sim_time(units="ns")
  while get_sim_time(units="ns") - timer < PERIOD * 4:
    await FallingEdge(dut.clock)

@cocotb.test()
async def rx_test(dut):
  # Exhaustive test of RX
  cocotb.start_soon(Clock(dut.clock, 40, units="ns").start())

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)
