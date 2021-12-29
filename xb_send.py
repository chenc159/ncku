import time
from struct import *
from pymavlink import mavutil
from digi.xbee.devices import DigiMeshDevice


# Connect pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0')
master.wait_heartbeat() # Wait for the first heartbeat 
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# # Connect xbee1
# xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
# xbee001.open(force_settings=True)

# # Connect xbee2
# xbee002 = DigiMeshDevice('/dev/ttyUSB2', 115200)
# xbee002.open(force_settings=True)

# Get checksum
chks = mavutil.x25crc()

# Initialize parameters for drone data
send_pkt_num = [2,3]
sysID, compID = master.target_system, master.target_component
start_time, start_uptime = master.start_time, master.uptime
header, checksum  = 255, 256
msgID = [128,129,130]
msgs =  {
    # "ID":                   {"sys": 0, "comp": 1, "comm": 22, "msg": [128,129]},   
    "ID":                   {"sys": master.target_system, "comp": master.target_component, "comm": 22, "msg": msgID},   
    "SYSTEM_TIME":          {"time_boot_ms": 0}, 
    "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
    "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
    "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
    "HEARTBEAT":            {"system_status": 99},
    "BATTERY_STATUS":       {"battery_remaining": 0},
    "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
    "STATUSTEXT":           {"severity": 99}
} #AHRS2, AHRS3

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
pkt_space = {1: [1,1,1,1,1,4,4,1,2], 2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2], 3: [1,1,1,1,1,4,4,4,4,2]}
pkt_len, res = {}, {}
for i in send_pkt_num:
    pkt_len[i] = len(pkt_item[i])
    res[i] = [0 for j in range(pkt_len[i])]

def init_pkt_bytearray(pkt_num):
    # Fist five items: header and ids
    init_pkt = bytearray([header, msgs["ID"]["comm"], msgs["ID"]["sys"], msgs["ID"]["comp"], msgs["ID"]["msg"][pkt_num-1]])
    # Rest of info
    for i, space in enumerate(pkt_space[pkt_num][5:-1]):
        items = pkt_item[pkt_num][i+5].split('.')
        init_pkt.extend(pack(byte_num[space], msgs[items[0]][items[1]]))
    # Last item: checksum
    init_pkt.extend(pack(byte_num[2], checksum))
    return init_pkt

pkt_to_send = {}
for i in send_pkt_num:
    pkt_to_send[i] = init_pkt_bytearray(i)


# # while xbee001.is_open():
while True:
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if msg_type not in msgs.keys():
        continue
    # Store messages
    for item in msgs[msg_type].keys():
        name = msg_type + '.' + item        
        msgs[msg_type][item] = round(getattr(msg, item)*convert.get(name, 1))
        # store values to corresponding packets
        for i in send_pkt_num:
            if name in pkt_item[i]:
                ind = pkt_item[i].index(name)
                pkt_to_send[i][sum(pkt_space[i][:ind]):sum(pkt_space[i][:ind+1])] = pack(byte_num[pkt_space[i][ind]], msgs[msg_type][item]) 
        
    # Before sending the packet out, update checksum
    for k in send_pkt_num:
        chks.accumulate(pkt_to_send[k][:-2]) #exclude checksum
        pkt_to_send[k][-2:] = pack(byte_num[2], chks.crc)
        # xbee001.send_data_broadcast(pkt_to_send[k])

    print('\n', msg)
    print(msgs)
    for do in send_pkt_num:
        data = pkt_to_send[do]
        for i, space in enumerate(pkt_space[do][:]):
            res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('ite: ', pkt_item[do])
        print('out: ', res[do])


    # try:
    #     received = xbee002.read_data()
    #     data = received.data
    #     do = msgID.index(data[4]) + 1
    #     for i, space in enumerate(pkt_space[do][:]):
    #         res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
    #         # if space == 1:
    #         #     res[do][i] = data[sum(pkt_space[do][:i])]
    #         # elif space == 2:
    #         #     res[do][i] = unpack('H',data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
    #         # elif space == 6:
    #         #     res[do][i] = unpack('i',data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
    #     print('out: ', res[do])
    # except:
    #     pass


