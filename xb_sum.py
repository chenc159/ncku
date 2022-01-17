import time
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

# Initialize parameters for drone data
send_pkt_num = info.send_pkt_num
# header, checksum  = 255, 256
# msgID = [128,129,130]
# msgs =  {
#     "ID":                   {"sys": 0, "comp": 1, "comm": 22, "msg": msgID},   
#     # "ID":                   {"sys": master.target_system, "comp": master.target_component, "comm": 22, "msg": msgID},   
#     "SYSTEM_TIME":          {"time_unix_usec": 0, "time_boot_ms": 0}, 
#     "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
#     "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
#     "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
#     "HEARTBEAT":            {"system_status": 99},
#     "BATTERY_STATUS":       {"battery_remaining": 0},
#     "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
#     "STATUSTEXT":           {"severity": 99}
# } #AHRS2, AHRS3
convert, byte_num = info.convert, info.byte_num
msgs = info.msgs
msgs["ID"]["sys"], msgs["ID"]["comp"] = master.target_system, master.target_component

msgs_c, msgs_p = {}, {}
for key1 in msgs.keys():
    msgs_c[key1], msgs_p[key1] = {}, {}
    for key2 in msgs[key1].keys():
        # print(msgs[key1][key2])
        msgs_c[key1][key2] = c_int(msgs[key1][key2])
        msgs_p[key1][key2] = pointer(msgs_c[key1][key2])


# Initialize packet
pkt_item, pkt_space, pkt_val = info.pkt_item, info.pkt_space, {}
for i in send_pkt_num:
    pkt_val[i] = [c_int(0) for k in range(len(pkt_item[i]))]
    pkt_val[i][0], pkt_val[i][4] = c_int(info.header), c_int(info.msgID[i])
    for j in range(1, len(pkt_item[i])-1):
        items = pkt_item[i][j].split('.')
        if pkt_val[i][j].value == 0:
            pkt_val[i][j] = msgs_c[items[0]][items[1]]

        # if items[0] == "ID" and items[1] == "mes":
        #     pkt_val[i][j] = info.msgID[i]
        # else:
        #     pkt_val[i][j] = msgs_c[items[0]][items[1]]


def init_pkt_bytearray(pkt_num):
    # Fist five items: header and ids
    init_pkt = bytearray([info.header, msgs["ID"]["comm"], msgs["ID"]["sys"], msgs["ID"]["comp"], msgs["ID"]["msg"][pkt_num-1]])
    # Rest of info
    for i, space in enumerate(pkt_space[pkt_num][5:-1]):
        items = pkt_item[pkt_num][i+5].split('.')
        init_pkt.extend(pack(byte_num[space], msgs[items[0]][items[1]]))
    # Last item: checksum
    init_pkt.extend(pack(byte_num[2], info.checksum))
    return init_pkt

# pkt_to_send = {}
# for i in send_pkt_num:
#     pkt_to_send[i] = init_pkt_bytearray(i)

while True:
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if msg_type not in msgs.keys():
        continue
    # Store messages
    for item in msgs[msg_type].keys():
        name = msg_type + '.' + item        
        msgs_p[msg_type][item][0] = round(getattr(msg, item)*convert.get(name, 1))
    
    print("\n", msg)
    print(pkt_item)
    print(pkt_val)

'''

while xbee001.is_open():
# while True:
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if msg_type not in msgs.keys():
        continue
    # Store messages
    for item in msgs[msg_type].keys():
        name = msg_type + '.' + item        
        msgs_p[msg_type][item][0] = round(getattr(msg, item)*convert.get(name, 1))
        # store values to corresponding packets
        for i in send_pkt_num:
            if name in pkt_item[i]:
                ind = pkt_item[i].index(name)
                pkt_to_send[i][sum(pkt_space[i][:ind]):sum(pkt_space[i][:ind+1])] = pack(byte_num[pkt_space[i][ind]], msgs[msg_type][item]) 
        
    # Before sending the packet out, update checksum
    for k in send_pkt_num:
        chks.accumulate(pkt_to_send[k][:-2]) #exclude checksum
        pkt_to_send[k][-2:] = pack(byte_num[2], chks.crc)
        xbee001.send_data_broadcast(pkt_to_send[k])

    print('\n', msg)
    print(msgs)
    for do in send_pkt_num:
        data = pkt_to_send[do]
        for i, space in enumerate(pkt_space[do][:]):
            res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('ite: ', pkt_item[do])
        print('out: ', res[do])
    

    # for i in send_pkt_num:
    #     print('ite: ', pkt_item[i])
    #     print('res: ', pkt_val[i])





    try:
        received = xbee002.read_data()
        data = received.data
        do = msgID.index(data[4]) + 1
        for i, space in enumerate(pkt_space[do][:]):
            res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('out: ', res[do])
    except:
        pass


'''