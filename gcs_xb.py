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
xbee002 = DigiMeshDevice('/dev/ttyUSB0', 115200)
xbee002.open(force_settings=True)

send_pkt_num = [1, 2, 3]
msgID = [128,129,130,10]

# Used to convert unit (e.g. 1 rad to 57.2958 deg) and byte format
convert = {"ATTITUDE.roll": 57.2958, "ATTITUDE.pitch": 57.2958, "ATTITUDE.yaw": 57.2958} # need to include failsafe later
byte_num = {1:'B', 2:'H', 4:'i'}

# Initialize packet
pkt_item = {
    1: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "GLOBAL_POSITION_INT.time_boot_ms", "SYSTEM_TIME.time_boot_ms", 
        "HEARTBEAT.system_status", "checksum"],
    2: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "GLOBAL_POSITION_INT.time_boot_ms", "SYSTEM_TIME.time_boot_ms", 
        "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg","checksum"],
    3: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "ATTITUDE.time_boot_ms", "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "checksum"]
}
pkt_space = {1: [1,1,1,1,1,4,4,1,2], 2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2], 3: [1,1,1,1,1,4,4,4,4,2], 4:[1,1,1,1,1,1]}
pkt_len, pkt_val, res = {}, {}, {}
for i in send_pkt_num:
    pkt_len[i] = len(pkt_item[i])
    res[i] = [0 for j in range(pkt_len[i])]

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
        command = input("1 to arm, 2 to disarm")
        try:
            send = bytearray([0,0,0,0,10,int(command)])
            xbee002.send_data_broadcast(send)
        except: pass



