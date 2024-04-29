import cocotb 
from cocotb.triggers import *
from cocotb.clock import Clock
from cocotb.utils import get_sim_time

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
      if get_sim_time(units="ns") - start > 10000:
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
    
    # STOP bit
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

async def send_uart_request(dut, data):
  await write(dut, data)

async def detect_uart_reply(dut, expected):
  reply = await read(dut)
  if (reply & 0b000011100) == T_CLEAR << 2:
    if (reply & 0b000000010) == 0b00:
      print(f"Bob      : Plane {"{:02d}".format(reply >> 5)} cleared to takeoff runway {reply & 0b1}")
    elif (reply & 0b000000010) == 0b10:
      print(f"Bob      : Plane {"{:02d}".format(reply >> 5)} cleared to land runway {reply & 0b1}")
  if (reply & 0b000011100) == T_HOLD << 2:
    print(f"Bob      : Plane {"{:02d}".format(reply >> 5)} hold")
  if (reply & 0b000011100) == T_ID_PLEASE << 2:
    if (reply & 0b000000011) == 0b00:
      print(f"Bob      : ID {reply >> 5} is available")
    elif (reply & 0b000000011) == 0b11:
      print(f"Bob      : My airspace is full")
  if (reply & 0b000011100) == T_DIVERT << 2:
    print(f"Bob      : Plane {"{:02d}".format(reply >> 5)} divert due to congestion")
        
  return (reply == expected, reply >> 5)

async def request(dut, id, type, action, expected_reply):
  await send_uart_request(dut, (id << 5) + (type << 2) + action)
  start = get_sim_time(units="ns")
  if type == T_ID_PLEASE:
    print(f"New Plane: Requesting ID for entry at time {start}")
  elif type == T_REQUEST:
    if action == R_TAKEOFF:
      print(f"Plane {"{:02d}".format(id)} : Requesting takeoff at time {start}")
    elif action == R_LANDING:
      print(f"Plane {"{:02d}".format(id)} : Requesting landing at time {start}")
  else:
    print(f"Plane {"{:02d}".format(id)} : Making invalid request at time {start}")

  detect = await detect_uart_reply(dut, expected_reply)
  while not detect[0]:
    detect = await detect_uart_reply(dut, expected_reply)
  assert detect[0]
  print("")
  print("////////////////////////////////////////")
  print(f"// TB      : Transaction success!     //")
  print("////////////////////////////////////////\n")
  return detect[1]


@cocotb.test()
async def basic_test(dut):
  print("////////////////////////////////////////")
  print("//         Begin basic tests          //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, 40, units="ns").start())

  dut.reset.value = True
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)

  # Plane requests ID
  id_1 = await request(dut, 0, T_ID_PLEASE, 0, (T_ID_PLEASE << 2))

  # Plane id_1 requests takeoff
  await request(dut, id_1, T_REQUEST, R_TAKEOFF, (id_1 << 5) + (T_CLEAR << 2) + C_TAKEOFF_0)
  
  # Plane requests ID
  id_2 = await request(dut, 0, T_ID_PLEASE, 0, (1 << 5) + (T_ID_PLEASE << 2))
  
  # Plane id_2 requests landing
  await request(dut, id_2, T_REQUEST, R_LANDING, (id_2 << 5) + (T_CLEAR << 2) + C_LANDING_1)

  # Plane requests ID
  id_3 = await request(dut, 0, T_ID_PLEASE, 0, (2 << 5) + (T_ID_PLEASE << 2))

  # Plane id_3 requests takeoff
  await request(dut, id_3, T_REQUEST, R_TAKEOFF, (id_3 << 5) + (T_HOLD << 2))

  # Plane id_1 declares takeoff runway 0, id_3 should be cleared
  await request(dut, id_1, T_DECLARE, D_TAKEOFF_0, (id_3 << 5) + (T_CLEAR << 2) + C_TAKEOFF_0)

  # Plane requests ID
  await request(dut, 0, T_ID_PLEASE, 0, (T_ID_PLEASE << 2))
  
  print("////////////////////////////////////////")
  print("//         Finish basic tests         //")
  print("////////////////////////////////////////\n")

# @cocotb.test()
async def stress_test_takeoff(dut):
  print("////////////////////////////////////////")
  print("//     Begin takeoff stress tests     //")
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
  id = []
  for i in range(16):
    # Plane requests ID
    await send_uart_request(dut, (T_ID_PLEASE << 2))
    start = get_sim_time(units="ns")
    print(f"New plane: Requesting ID for entry at time {start}")

    while True:
      await FallingEdge(dut.clock)
      if get_sim_time(units="ns") - start > 10000:
        raise TimeoutError("Did not receive ID")
      detect = detect_uart_reply(dut, (i << 5) + (T_ID_PLEASE << 2))
      if detect[0]:
        id.append(detect[1])
        print("")
        print("////////////////////////////////////////")
        print("// TB      : ID request success!      //")
        print("////////////////////////////////////////\n")
        break
    await FallingEdge(dut.clock)
  
  assert dut.all_id.value == 0xFFFF
  assert dut.id_full.value

  for i in range(16):
    # Plane requests takeoff
    await send_uart_request(dut, (i << 5) + (T_REQUEST << 2) + R_TAKEOFF)
    start = get_sim_time(units="ns")
    print(f"Plane {"{:02d}".format(i)} : Requesting takeoff at time {start}")

    while True:
      await FallingEdge(dut.clock)
      if get_sim_time(units="ns") - start > 10000:
        raise TimeoutError("Did not receive hold")
      if detect_uart_reply(dut, (i << 5) + (T_HOLD << 2))[0]:
        print("")
        print("////////////////////////////////////////")
        print("// TB      : Received hold success!   //")
        print("////////////////////////////////////////\n")
        break
    await FallingEdge(dut.clock)

  return 0