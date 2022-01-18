import time
from struct import *
from pymavlink import mavutil
from digi.xbee.devices import DigiMeshDevice
from ctypes import *
from info import info

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
xbee002 = DigiMeshDevice('/dev/ttyUSB0', 115200)
xbee002.open(force_settings=True)

send_pkt_num, pkt_item, pkt_space = info.send_pkt_num, info.pkt_item, info.pkt_space
convert, byte_num = info.convert, info.byte_num
msgID = info.msgID

res = {}
for i in send_pkt_num:
    res[i] = [0 for k in range(len(pkt_item[i]))]


t = 0
while xbee002.is_open():

    try:
        received = xbee002.read_data()
        data = received.data
        do = msgID.index(data[4]) + 1
        for i, space in enumerate(pkt_space[do][:]):
            res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print(pkt_item[do])
        print('out: ', res[do])
    except: pass

    t += 1
    if (t%1000 == 0):
        command = input("0 to 9 to set mode, 10 to arm, 11 to disarm")        
        try:
            send = bytearray([0,0,0,0,190,int(command)])
            xbee002.send_data_broadcast(send)
        except: pass



