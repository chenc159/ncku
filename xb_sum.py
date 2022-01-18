import time
from datetime import datetime
from struct import *
from pymavlink import mavutil
from digi.xbee.devices import DigiMeshDevice
from ctypes import *
from info import info

# Connect pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0')
master.wait_heartbeat() # Wait for the first heartbeat 
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 4 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

# # Connect xbee1
# xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
# xbee001.open(force_settings=True)

# # Connect xbee2
# xbee002 = DigiMeshDevice('/dev/ttyUSB2', 115200)
# xbee002.open(force_settings=True)

# Get checksum
chks = mavutil.x25crc()

# Initialize parameters
msgs = info.msgs
msgs["ID"]["sys"], msgs["ID"]["comp"] = master.target_system, master.target_component
# Use c struct and pointer to automatically store values
msgs_c, msgs_p = {}, {}
for key1 in msgs.keys():
    msgs_c[key1], msgs_p[key1] = {}, {}
    for key2 in msgs[key1].keys():
        msgs_c[key1][key2] = c_int(msgs[key1][key2])
        msgs_p[key1][key2] = pointer(msgs_c[key1][key2])


# Initialize packets
# send_pkt_num: list of packet number; item: items in packet; space: space allocation of packet
# val: values of packet items; bytearray: bytearray of the packet value (to be sent by xbee); res: unpacked result of the packed packet
# convert: unit conversion, preferred int; byte_num: get format letter from space allocation for un/pack usage
send_pkt_num, pkt_item, pkt_space = info.send_pkt_num, info.pkt_item, info.pkt_space
convert, byte_num = info.convert, info.byte_num
pkt_val, pkt_bytearray, res = {}, {}, {}
for i in send_pkt_num:
    pkt_val[i] = [c_int(0) for k in range(len(pkt_item[i]))]
    pkt_val[i][0], pkt_val[i][4] = c_int(info.header), c_int(info.msgID[i])
    pkt_bytearray[i] = bytearray([pkt_val[i][0].value])
    res[i] = [0 for k in range(len(pkt_item[i]))]
    for j in range(1, len(pkt_item[i])-1):
        items = pkt_item[i][j].split('.')
        if pkt_val[i][j].value == 0:
            pkt_val[i][j] = msgs_c[items[0]][items[1]]
        pkt_bytearray[i].extend(pack(byte_num[pkt_space[i][j]], pkt_val[i][j].value))
    pkt_bytearray[i].extend(pack(byte_num[2], info.checksum))

# Pack and send packets
def send_pkt():
    for i in send_pkt_num:
        # store computer system time and gps time
        utctime = datetime.utcnow()
        msgs_p["ID"]["time"][0] = int((utctime.minute*60 + utctime.second)*1e6 + utctime.microsecond)
        gpstime = datetime.utcfromtimestamp(msgs_p["SYSTEM_TIME"]["time_unix_usec"][0]/1e6)
        msgs_p["SYSTEM_TIME"]["time_unix_usec"][0] = int((gpstime.minute*60 + gpstime.second)*1e6 + gpstime.microsecond)
        # pack packet values to bytearray (first 5 items shall remain unchanged)
        for j in range(5,len(pkt_item[i])):
            pkt_bytearray[i][sum(pkt_space[i][:j]):sum(pkt_space[i][:j+1])] = pack(byte_num[pkt_space[i][j]], pkt_val[i][j].value)
        # calculae checksum
        chks.accumulate(pkt_bytearray[i][:-2]) 
        pkt_bytearray[i][-2:] = pack(byte_num[2], chks.crc)
        # send by xbee
        try: xbee001.send_data_broadcast(pkt_bytearray[i])
        except: pass

# Unpack and read packets
def read_pkt():
    try:
        # first try to read via xbee, if xbee is connected
        received = xbee002.read_data()
        data = received.data
        do = info.msgID.index(data[4]) + 1
        for i, space in enumerate(pkt_space[do][:]):
            res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('xbee received: ', res[do])
    except:
        # then read it "manually"
        for i in send_pkt_num:
            for j, space in enumerate(pkt_space[i][:]):
                res[i][j] = unpack(byte_num[space],pkt_bytearray[i][sum(pkt_space[i][:j]):sum(pkt_space[i][:j+1])])[0]
            print('out: ', i, res[i])


last_time = 0 
while True:
    # Get message from pixhawk
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    # Check if msg if what we need
    if msg_type not in msgs.keys():
        continue
    # Store message
    for item in msgs[msg_type].keys():
        name = msg_type + '.' + item        
        msgs_p[msg_type][item][0] = round(getattr(msg, item)*convert.get(name, 1))
    
    # print("\n", msg)
    # print(pkt_item)
    # print(pkt_val)

    # send out packets every 1 sec
    if time.time() - last_time >= 1.0:
        print("\n")
        send_pkt()
        read_pkt()
        last_time = time.time()
        
