from array import array
import time
from struct import *
from pymavlink import mavutil
from digi.xbee.devices import DigiMeshDevice
from ctypes import *

# # Connect pixhawk
# master = mavutil.mavlink_connection('/dev/ttyACM0')
# master.wait_heartbeat() # Wait for the first heartbeat 
# print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# # Initialize data stream
# rate = 4 # desired transmission rate
# master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

# # Connect xbee1
xbee001 = DigiMeshDevice('/dev/ttyUSB1', 115200)
xbee001.open(force_settings=True)


class a:
    b = c_int()
    c = c_int()

class MyStruct(Structure):
    _fields_ = [("a", c_int), ("b", c_int)]

# Connect xbee2
xbee002 = DigiMeshDevice('/dev/ttyUSB0', 115200)
xbee002.open(force_settings=True)

while xbee002.is_open():
    # send = a().b
    # send = MyStruct(2,3)
    send = [1, 2, 255, 266, 9999999]
    send = ' '.join(str(e) for e in send)
    print('send: ',send)
    xbee002.send_data_broadcast(send)

    # command = input("1 to arm, 2 to disarm")
    # try:
    #     send = bytearray([0,0,0,0,10,int(command)])
    #     xbee002.send_data_broadcast(send)
    # except: pass
    try:
        received = xbee001.read_data()
        data = received.data
        print('data: ',data)
        number = [int(i) for i in data.split( )]
        print(number)
    except: pass




