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

C_RUNWAY_0   = 0b0 # Cleared
C_RUNWAY_1   = 0b1

R_TAKEOFF    = 0b0 # Request
R_LANDING    = 0b1

D_RUNWAY_0  = 0b0 # Declare
D_RUNWAY_1  = 0b1

E_DECLARE    = 0b1
E_RESOLVE    = 0b0

BAUD_RATE = 115200
PERIOD = (1 / BAUD_RATE) * 10**9
CLOCK_PERIOD = 40

async def read(dut):
  data = 0b00000000
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
    while num_bits < 8:
      bit_start = get_sim_time(units="ns")
      while get_sim_time(units="ns") - bit_start < PERIOD / 2:
        await FallingEdge(dut.clock)
      data >>= 1
      data |= (dut.tx.value << 7)
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

  while num_bits < 8:
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
  if (reply & 0b00001110) == T_CLEAR << 1:
    if (reply & 0b00000001) == 0b0:
      print(f"Bob      : Plane {"{:02d}".format(reply >> 4)} cleared runway {reply & 0b1}")
    elif (reply & 0b00000001) == 0b1:
      print(f"Bob      : Plane {"{:02d}".format(reply >> 4)} cleared runway {reply & 0b1}")
  if (reply & 0b00001110) == T_HOLD << 1:
    print(f"Bob      : Plane {"{:02d}".format(reply >> 4)} hold")
  if (reply & 0b00001110) == T_ID_PLEASE << 1:
    if (reply & 0b00000001) == 0b0:
      print(f"Bob      : ID {reply >> 4} is available")
    elif (reply & 0b00000001) == 0b1:
      print(f"Bob      : My airspace is full")
  if (reply & 0b00001110) == T_DIVERT << 1:
    print(f"Bob      : Plane {"{:02d}".format(reply >> 4)} divert due to congestion or emergency")
        
  return (reply == expected, reply >> 4)

async def request(dut, id, type, action, expected_reply, ignore_reply):
  await send_uart_request(dut, (id << 4) + (type << 1) + action)
  start = get_sim_time(units="ns")
  if type == T_ID_PLEASE:
    print(f"New Plane: Requesting ID for entry at time {start}")
  elif type == T_REQUEST:
    if action == R_TAKEOFF:
      print(f"Plane {"{:02d}".format(id)} : Requesting takeoff at time {start}")
    elif action == R_LANDING:
      print(f"Plane {"{:02d}".format(id)} : Requesting landing at time {start}")
  elif type == T_DECLARE:
    if action == D_RUNWAY_0:
      print(f"Plane {"{:02d}".format(id)} : Declaring takeoff/landing runway 0 at time {start}")
    elif action == D_RUNWAY_1:
      print(f"Plane {"{:02d}".format(id)} : Declaring takeoff/landing runway 1 at time {start}")
  elif type == T_EMERGENCY:
    if action == E_DECLARE:
      print(f"Plane {"{:02d}".format(id)} : Declaring emergency")
    elif action == E_RESOLVE:
      print(f"Plane {"{:02d}".format(id)} : Resolving emergency")
  else:
    print(f"Plane {"{:02d}".format(id)} : Making invalid request at time {start}")

  if not ignore_reply:
    detect = await detect_uart_reply(dut, expected_reply)
    while not detect[0]:
      detect = await detect_uart_reply(dut, expected_reply)
    assert detect[0]
    print("")
    print("////////////////////////////////////////")
    print(f"// TB      : Transaction success!     //")
    print("////////////////////////////////////////\n")
    return detect[1]

@cocotb.test(skip=True)
async def basic_test(dut):
  print("////////////////////////////////////////")
  print("//         Begin basic tests          //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b00
  
  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  # Plane requests ID
  id_1 = await request(dut, 0, T_ID_PLEASE, 0, (T_ID_PLEASE << 1), False)

  # Plane id_1 requests takeoff
  await request(dut, id_1, T_REQUEST, R_TAKEOFF, (id_1 << 4) + (T_CLEAR << 1) + C_RUNWAY_0, False)
  
  # Plane requests ID
  id_2 = await request(dut, 0, T_ID_PLEASE, 0, (1 << 4) + (T_ID_PLEASE << 1), False)
  
  # Plane id_2 requests landing
  await request(dut, id_2, T_REQUEST, R_LANDING, (id_2 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)

  # Plane requests ID
  id_3 = await request(dut, 0, T_ID_PLEASE, 0, (2 << 4) + (T_ID_PLEASE << 1), False)

  # Plane id_3 requests takeoff
  await request(dut, id_3, T_REQUEST, R_TAKEOFF, (id_3 << 4) + (T_HOLD << 1), False)

  # Plane id_1 declares takeoff runway 0, id_3 should be cleared
  await request(dut, id_1, T_DECLARE, D_RUNWAY_0, (id_3 << 4) + (T_CLEAR << 1) + C_RUNWAY_0, False)

  # Plane requests ID
  await request(dut, 0, T_ID_PLEASE, 0, (T_ID_PLEASE << 1), False)
  
  print("////////////////////////////////////////")
  print("//         Finish basic tests         //")
  print("////////////////////////////////////////\n")

@cocotb.test(skip=True)
async def stress_test_takeoff(dut):
  print("////////////////////////////////////////")
  print("//     Begin takeoff stress tests     //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b00

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  id = []
  for i in range(16):
    # Plane requests ID
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))
  
  assert dut.bobby.all_id.value == 0xFFFF
  assert dut.bobby.id_full.value

  for i in range(2):
    # Planes 00, 01 request takeoff, immediately cleared
    await request(dut, i, T_REQUEST, R_TAKEOFF, (i << 4) + (T_CLEAR << 1) + i, False)

  assert dut.runway_active.value == 0b11 
  assert dut.bobby.takeoff_fifo.empty.value

  for i in range(2, 10):
    # 4 planes request takeoff, all on hold (2, 3, 4, 5, 6, 7, 8, 9, 10)
    await request(dut, i, T_REQUEST, R_TAKEOFF, (i << 4) + (T_HOLD << 1), False)

  assert dut.bobby.takeoff_fifo.full.value

  for i in range(10, 16):
    # Planes request takeoff, diverted, they lose their IDs
    await request(dut, i, T_REQUEST, R_TAKEOFF, (i << 4) + (T_DIVERT << 1), False)
  
  dut.bobby.all_id.value == 0x00FF
  dut.bobby.takeoff_fifo.count.value == 0b1000
  
  for i in range(8):
    # Planes declare takeoff
    await request(dut, i, T_DECLARE, i % 2, ((i + 2) << 4) + (T_CLEAR << 1) + (i % 2), False)
  
  assert dut.bobby.takeoff_fifo.empty.value
  assert dut.bobby.runway_manager.runway.value == 0b1001110001

  # Active IDs at this point should be 8 and 9 only
  assert dut.bobby.all_id.value == 0x0300

  for i in range(0, 8):
    # Fill up ID space from 0 to 8
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))

  assert dut.bobby.all_id.value == 0x03FF

  for i in range(10, 16):
    # Diverted planes requests ID again
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))

  assert dut.bobby.all_id.value == 0xFFFF

  for i in range(10, 16):
    await request(dut, i, T_REQUEST, R_TAKEOFF, (i << 4) + (T_HOLD << 1), False)
    await request(dut, i - 2, T_DECLARE, (i - 2) % 2, (i << 4) + (T_CLEAR << 1) + (i - 2) % 2, False)
  
  assert dut.bobby.all_id.value == 0xC0FF

  for i in range(14, 16):
    await request(dut, i, T_DECLARE, i % 2, 0, True)
  
  assert dut.bobby.all_id.value == 0x00FF
  assert dut.bobby.takeoff_fifo.empty.value
  assert dut.bobby.landing_fifo.empty.value

  print("////////////////////////////////////////")
  print("//     Finish takeoff stress tests    //")
  print("////////////////////////////////////////\n")

@cocotb.test(skip=True)
async def stress_test_landing(dut):
  print("////////////////////////////////////////")
  print("//     Begin landing stress tests     //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b00

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  id = []
  for i in range(16):
    # Plane requests ID
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))
  
  assert dut.bobby.all_id.value == 0xFFFF
  assert dut.bobby.id_full.value

  for i in range(2):
    # Planes 00, 01 request landing, immediately cleared
    await request(dut, i, T_REQUEST, R_LANDING, (i << 4) + (T_CLEAR << 1) + i, False)

  assert dut.runway_active.value == 0b11 
  assert dut.bobby.landing_fifo.empty.value

  for i in range(2, 10):
    # 4 planes request landing, all on hold (10, 11, 100, 101)
    await request(dut, i, T_REQUEST, R_LANDING, (i << 4) + (T_HOLD << 1), False)

  assert dut.bobby.landing_fifo.full.value

  for i in range(10, 16):
    # Planes request landing, diverted, they lose their IDs
    await request(dut, i, T_REQUEST, R_LANDING, (i << 4) + (T_DIVERT << 1), False)
  
  dut.bobby.all_id.value == 0x00FF
  dut.bobby.landing_fifo.count.value == 0b1000
  
  for i in range(8):
    # Planes 0, 1 declare landing, 2, 3, cleared. 2, 3 declare landing, 4, 5 cleared.
    await request(dut, i, T_DECLARE, i % 2, ((i + 2) << 4) + (T_CLEAR << 1) + (i % 2), False)
  
  assert dut.bobby.landing_fifo.empty.value
  assert dut.bobby.runway_manager.runway.value == 0b1001110001
  
  # Active IDs at this point should be 8 and 9 only
  assert dut.bobby.all_id.value == 0x0300

  for i in range(0, 8):
    # Fill up ID space from 0 to 3
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))

  assert dut.bobby.all_id.value == 0x03FF

  for i in range(10, 16):
    # Diverted planes requests ID again
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))

  assert dut.bobby.all_id.value == 0xFFFF
  
  for i in range(10, 16):
    await request(dut, i, T_REQUEST, R_LANDING, (i << 4) + (T_HOLD << 1), False)
    await request(dut, i - 2, T_DECLARE, (i - 2) % 2, (i << 4) + (T_CLEAR << 1) + (i - 2) % 2, False)
  
  assert dut.bobby.all_id.value == 0xC0FF

  for i in range(14, 16):
    await request(dut, i, T_DECLARE, i % 2, 0, True)
  
  assert dut.bobby.all_id.value == 0x00FF
  assert dut.bobby.takeoff_fifo.empty.value
  assert dut.bobby.landing_fifo.empty.value

  print("////////////////////////////////////////")
  print("//     Finish takeoff stress tests    //")
  print("////////////////////////////////////////\n")

@cocotb.test(skip=True)
async def stress_test_id(dut):
  print("////////////////////////////////////////")
  print("//        Begin ID stress tests       //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b00

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  id = []
  for i in range(16):
    # Plane requests ID
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))
  
  assert dut.bobby.all_id.value == 0xFFFF
  assert dut.bobby.id_full.value

  for i in range(16):
    await request(dut, i, T_ID_PLEASE, 0, (T_ID_PLEASE << 1) + 0b1, False)
  
  assert dut.bobby.all_id.value == 0xFFFF
  assert dut.bobby.id_full.value

  print("////////////////////////////////////////")
  print("//       Finish ID stress tests       //")
  print("////////////////////////////////////////\n")

  return 0

@cocotb.test(skip=False)
async def stress_test_alternate(dut):
  # Queue up both landing and takeoff FIFOs
  print("////////////////////////////////////////////////////")
  print("// Begin alternating takeoff/landing stress tests //")
  print("////////////////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b01

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  id = []
  for i in range(16):
    # Plane requests ID
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))

  # Plane 0 is cleared to takeoff
  await request(dut, 0, T_REQUEST, R_TAKEOFF, (T_CLEAR << 1) + C_RUNWAY_1, False)

  for i in range(1, 8):
    # 7 planes are queued to takeoff
    await request(dut, i, T_REQUEST, R_TAKEOFF, (i << 4) + (T_HOLD << 1), False)
  
  assert not dut.bobby.takeoff_fifo.full.value
  assert dut.bobby.takeoff_fifo.count.value == 0b111

  for i in range(8, 16):
    # 8 planes are queued to land
    await request(dut, i, T_REQUEST, R_LANDING, (i << 4) + (T_HOLD << 1), False)
  
  assert dut.bobby.landing_fifo.full.value
  assert dut.bobby.landing_fifo.count.value == 0b1000
  
  # Plane 0 declares takeoff, expect landing
  await request(dut, 0, T_DECLARE, D_RUNWAY_1, (8 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 8 declares landing, expect takeoff
  await request(dut, 8, T_DECLARE, D_RUNWAY_1, (1 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 1 declares takeoff, expect landing
  await request(dut, 1, T_DECLARE, D_RUNWAY_1, (9 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Random id request to check takeoff_first remains the same
  await request(dut, 0, T_ID_PLEASE, 0, (T_ID_PLEASE << 1), False)
  # Plane 9 declares landing, expect takeoff
  await request(dut, 9, T_DECLARE, D_RUNWAY_1, (2 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 2 declares takeoff, expect landing
  await request(dut, 2, T_DECLARE, D_RUNWAY_1, (10 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 10 declares landing, expect takeoff
  await request(dut, 10, T_DECLARE, D_RUNWAY_1, (3 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 3 declares takeoff, expect landing
  await request(dut, 3, T_DECLARE, D_RUNWAY_1, (11 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 10 declares landing, expect takeoff
  await request(dut, 11, T_DECLARE, D_RUNWAY_1, (4 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 4 declares takeoff, expect landing
  await request(dut, 4, T_DECLARE, D_RUNWAY_1, (12 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 12 declares landing, expect takeoff
  await request(dut, 12, T_DECLARE, D_RUNWAY_1, (5 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 5 declares takeoff, expect landing
  await request(dut, 5, T_DECLARE, D_RUNWAY_1, (13 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 13 declares landing, expect takeoff
  await request(dut, 13, T_DECLARE, D_RUNWAY_1, (6 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 6 declares takeoff, expect landing
  await request(dut, 6, T_DECLARE, D_RUNWAY_1, (14 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 14 declares landing, expect takeoff
  await request(dut, 14, T_DECLARE, D_RUNWAY_1, (7 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 7 declares takeoff, expect landing
  await request(dut, 7, T_DECLARE, D_RUNWAY_1, (15 << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)
  # Plane 15 declares landing, expect takeoff
  await request(dut, 15, T_DECLARE, D_RUNWAY_1, 0, True)

  assert dut.bobby.landing_fifo.empty.value
  assert dut.bobby.takeoff_fifo.empty.value
  assert dut.runway_active.value == 0b01

@cocotb.test(skip=False)
async def emergency_test(dut):
  print("////////////////////////////////////////")
  print("//       Begin emergency tests        //")
  print("////////////////////////////////////////\n")

  # Run the clock
  cocotb.start_soon(Clock(dut.clock, CLOCK_PERIOD, units="ns").start())

  dut.runway_override.value = 0b00

  dut.reset.value = True
  await FallingEdge(dut.clock)
  dut.reset.value = False
  await FallingEdge(dut.clock)

  id = []
  for i in range(10):
    # Plane requests ID
    id.append(await request(dut, 0, T_ID_PLEASE, 0, (i << 4) + (T_ID_PLEASE << 1), False))
  
  assert dut.bobby.all_id.value == 0x03FF
  assert not dut.bobby.id_full.value

  await request(dut, id[0], T_REQUEST, R_LANDING, (id[0] << 4) + (T_CLEAR << 1) + C_RUNWAY_0, False)
  await request(dut, id[5], T_REQUEST, R_TAKEOFF, (id[5] << 4) + (T_CLEAR << 1) + C_RUNWAY_1, False)

  assert dut.bobby.runway_active == 0b11

  for i in range(1, 5):
    # 4 planes get queued for landing
    await request(dut, id[i], T_REQUEST, R_LANDING, (id[i] << 4) + (T_HOLD << 1), False)

  await request(dut, id[0], T_EMERGENCY, E_DECLARE, 0, True)
  detect = await detect_uart_reply(dut, (1 << 4) + (T_DIVERT << 1))
  assert detect[0]
  detect = await detect_uart_reply(dut, (2 << 4) + (T_DIVERT << 1))
  assert detect[0]
  detect = await detect_uart_reply(dut, (3 << 4) + (T_DIVERT << 1))
  assert detect[0]
  detect = await detect_uart_reply(dut, (4 << 4) + (T_DIVERT << 1))
  assert detect[0]

  assert dut.bobby.all_id.value == 0x03E1

  for i in range(6, 10):
    # 4 planes get queued for landing
    await request(dut, id[i], T_REQUEST, R_TAKEOFF, (id[i] << 4) + (T_HOLD << 1), False)
  
  # Invalid resolving plane ID
  await request(dut, id[1], T_EMERGENCY, E_RESOLVE, 0, True)

  assert dut.bobby.emergency.value

  await request(dut, id[0], T_EMERGENCY, E_RESOLVE, 0, True)
  
  assert not dut.bobby.emergency.value

  print("////////////////////////////////////////")
  print("//       Finish emergency tests       //")
  print("////////////////////////////////////////\n")
  
@cocotb.test(skip=True)
async def invalid_behavior_test(dut):
  # Invalid aircraft tries to unlock runway
  # Invalid aircraft tries to unlock emergency
  
  return 0