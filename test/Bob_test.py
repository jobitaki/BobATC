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

async def send_uart_request(dut, data):
  dut.uart_rx_data.value = data # 0101 Declare takeoff 0
  dut.uart_rx_valid.value = True
  await FallingEdge(dut.clock)
  dut.uart_rx_valid.value = False

def detect_uart_reply(dut, expected):
  reply = dut.uart_tx_data.value
  if dut.uart_tx_send == 0b1:
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

# @cocotb.test()
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
  
  # Plane requests ID
  id_1 = -1
  await send_uart_request(dut, (T_ID_PLEASE << 2))
  start = get_sim_time(units="ns")
  print(f"New plane: Requesting ID for entry at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive ID")
    detect = detect_uart_reply(dut, (T_ID_PLEASE << 2))
    if detect[0]:
      id_1 = detect[1]
      print("")
      print("////////////////////////////////////////")
      print("// TB      : ID request success!      //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)

  # Plane id_1 requests takeoff
  await send_uart_request(dut, (id_1 << 5) + (T_REQUEST << 2) + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_1)} : Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if detect_uart_reply(dut, (id_1 << 5) + (T_CLEAR << 2) + C_TAKEOFF_0)[0]:
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Takeoff request success! //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)
  
  # Plane requests ID
  id_2 = -1
  await send_uart_request(dut, (T_ID_PLEASE << 2))
  start = get_sim_time(units="ns")
  print(f"New plane: Requesting ID for entry at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive ID")
    detect = detect_uart_reply(dut, (0b1 << 5) + (T_ID_PLEASE << 2))
    if detect[0]:
      id_2 = detect[1]
      print("")
      print("////////////////////////////////////////")
      print("// TB      : ID request success!      //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)

  # Plane id_2 requests landing
  await send_uart_request(dut, (id_2 << 5) + (T_REQUEST << 2) + R_LANDING)
  start = get_sim_time(units = "ns")
  print(f"Plane {"{:02d}".format(id_2)} : Requesting landing at time {start}")
  
  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive clearance")
    if detect_uart_reply(dut, (id_2 << 5) + (T_CLEAR << 2) + C_LANDING_1)[0]:
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Landing request success! //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)

  # Plane requests ID
  id_3 = -1
  await send_uart_request(dut, (T_ID_PLEASE << 2))
  start = get_sim_time(units="ns")
  print(f"New plane: Requesting ID for entry at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive ID")
    detect = detect_uart_reply(dut, (2 << 5) + (T_ID_PLEASE << 2))
    if detect[0]:
      id_3 = detect[1]
      print("")
      print("////////////////////////////////////////")
      print("// TB      : ID request success!      //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)

  # Plane id_3 requests takeoff
  await send_uart_request(dut, (id_3 << 5) + (T_REQUEST << 2) + R_TAKEOFF)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_3)} : Requesting takeoff at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive hold")
    if detect_uart_reply(dut, (id_3 << 5) + (T_HOLD << 2))[0]:
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Received hold success!   //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)

  # Plane id_1 declares takeoff runway 0
  await send_uart_request(dut, (id_1 << 5) + (T_DECLARE << 2) + D_TAKEOFF_0)
  start = get_sim_time(units="ns")
  print(f"Plane {"{:02d}".format(id_1)} : Declaring takeoff on runway 0 at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive takeoff clearance")
    if detect_uart_reply(dut, (id_3 << 5) + (T_CLEAR << 2) + C_TAKEOFF_0)[0]:
      print("")
      print("////////////////////////////////////////")
      print("// TB      : Takeoff request success! //")
      print("////////////////////////////////////////\n")
      end = get_sim_time(units="ns")
      break
  await FallingEdge(dut.clock)

  # Plane requests ID
  id_4 = -1
  await send_uart_request(dut, (T_ID_PLEASE << 2))
  start = get_sim_time(units="ns")
  print(f"New plane: Requesting ID for entry at time {start}")

  while True:
    await FallingEdge(dut.clock)
    if get_sim_time(units="ns") - start > 10000:
      raise TimeoutError("Did not receive ID")
    detect = detect_uart_reply(dut, (0 << 5) + (T_ID_PLEASE << 2))
    if detect[0]:
      id_4 = detect[1]
      print("")
      print("////////////////////////////////////////")
      print("// TB      : ID request success!      //")
      print("////////////////////////////////////////\n")
      break
  await FallingEdge(dut.clock)
  await FallingEdge(dut.clock)
  
  print("////////////////////////////////////////")
  print("//         Finish basic tests         //")
  print("////////////////////////////////////////\n")

@cocotb.test()
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