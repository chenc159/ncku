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
# xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
# xbee001.open(force_settings=True)

# Connect xbee2
xbee002 = DigiMeshDevice('/dev/ttyUSB2', 115200)
xbee002.open(force_settings=True)

while xbee002.is_open():
    command = input("1 to arm, 2 to disarm")
    try:
        send = bytearray([0,0,0,0,10,int(command)])
        xbee002.send_data_broadcast(send)
    except: pass




