import time, math
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
sysID, compID, commID = master.target_system, master.target_component, 22
systime, gpstime = 0, 0
roll, pitch, yaw, xacc, yacc, zacc = 0,0,0,0,0,0                   # in deg, mG
lat, lon, alt, vx, vy, vz, hdg = 0,0,0,0,0,0,0              # in degE7, mm, cm/s, cdeg
last_sent_time = 0 

msgID_send, msgID_receive = info.msgID_send, info.msgID_receive
# Initialize packets
# send_pkt_num: list of packet number; item: items in packet; space: space allocation of packet
# val: values of packet items; bytearray: bytearray of the packet value (to be sent by xbee); res: unpacked result of the packed packet
# convert: unit conversion, preferred int; byte_num: get format letter from space allocation for un/pack usage
pkt_item, pkt_space = info.pkt_item, info.pkt_space
convert, byte_num = info.convert, info.byte_num
pkt_val, pkt_bytearray, res = {}, {}, {}
for i in msgID_send:
    res[i] = [0 for k in range(len(pkt_item[i]))]
    pkt_val[i] = [0 for k in range(len(pkt_item[i]))]
    pkt_val[i][0], pkt_val[i][1], pkt_val[i][-1] = info.header, i, info.checksum
    pkt_val[i][2], pkt_val[i][3], pkt_val[i][4] = sysID, compID, commID
    pkt_bytearray[i] = bytearray([pkt_val[i][0]])
    for j in range(1, len(pkt_item[i])):
        pkt_bytearray[i].extend(pack(byte_num[pkt_space[i][j]], pkt_val[i][j]))
pkt_val[127][pkt_item[127].index('Mode')], pkt_val[127][pkt_item[127].index('Arm')] = 255, 255
pkt_val[127][pkt_item[127].index('MAVstatus')], pkt_val[127][pkt_item[127].index('Failsafe')] = 255, 255

while True:
    msgID_to_send = []
    if pkt_val[127][pkt_item[127].index('Mode')] != master.flightmode:
        pkt_val[127][pkt_item[127].index('Mode')] = master.flightmode
        msgID_to_send.extend(127)
    if pkt_val[127][pkt_item[127].index('Arm')] != master.sysid_state[master.sysid].armed:
        pkt_val[127][pkt_item[127].index('Arm')] = master.sysid_state[master.sysid].armed
        msgID_to_send.extend(127)

    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if msg == None:
        continue
    elif msg_type == "SYSTEM_TIME":             # gps utc time
        gpstime = datetime.utcfromtimestamp(msg.time_unix_usec/1e6)
        pkt_val[128][pkt_item[128].index('SYSTEM_TIME.time_unix_usec')] = int((gpstime.minute*60 + gpstime.second)*1e6 + gpstime.microsecond)
    elif msg_type == "ATTITUDE":              # imu: time, roll, pitch, yaw
        pkt_val[128][pkt_item[128].index('ATTITUDE.roll')] = round(msg.roll*57.2958)
        pkt_val[128][pkt_item[128].index('ATTITUDE.pitch')] = round(msg.pitch*57.2958)
        pkt_val[128][pkt_item[128].index('ATTITUDE.yaw')] = round(msg.yaw*57.2958)
    elif msg_type == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.lat')] = msg.lat
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.lon')] = msg.lon
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.alt')] = msg.alt
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vx')] = msg.vx
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vy')] = msg.vy
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vz')] = msg.vz
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.hdg')] = msg.hgd
    elif msg_type == "SCALED_IMU":
        pkt_val[128][pkt_item[128].index('SCALED_IMU.xacc')] = msg.xacc
        pkt_val[128][pkt_item[128].index('SCALED_IMU.yacc')] = msg.yacc
        pkt_val[128][pkt_item[128].index('SCALED_IMU.zacc')] = msg.zacc

    elif msg_type == "HEARTBEAT":             # MAV_STATE
        pkt_val[127][pkt_item[127].index('MAVstatus')] = msg.system_status 
        msgID_to_send.extend(127)
    elif msg_type == "HIGH_LATENCY2": # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
        pkt_val[127][pkt_item[127].index('Failsafe')] = int(math.log(msg.HL_FAILURE_FLAG,2))
        msgID_to_send.extend(127)
    elif msg_type == "STATUSTEXT":
        if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            pkt_val[127][pkt_item[127].index('Failsafe')] = int(math.log(msg.HL_FAILURE_FLAG,2))
            msgID_to_send.extend(127)

    if (time.time() - last_sent_time) >= 1.0:
        msgID_to_send.extend(128)
        last_sent_time = time.time()
    
    # Send packet
    msgID_to_send = set(msgID_to_send)
    for i in msgID_to_send:
        # store computer system time and gps time
        utctime = datetime.utcnow()
        pkt_val[i][-2] = int((utctime.minute*60 + utctime.second)*1e6 + utctime.microsecond)
        for j in range(5,len(pkt_item[i])-1):
            pkt_bytearray[i][sum(pkt_space[i][:j]):sum(pkt_space[i][:j+1])] = pack(byte_num[pkt_space[i][j]], pkt_val[i][j].value)
        # calculae checksum
        chks.accumulate(pkt_bytearray[i][:-2]) 
        pkt_bytearray[i][-2:] = pack(byte_num[2], chks.crc)
        # send by xbee
        try: xbee001.send_data_broadcast(pkt_bytearray[i])
        except: pass

    # Read packet
    try:
        # first try to read via xbee, if xbee is connected
        received = xbee001.read_data()
        data = received.data
        received_msgID = data[1]
        if received_msgID == 131:
            Desired_dist, Waypt_count = 0,0


        # for i, space in enumerate(pkt_space[do][:]):
        #     res[do][i] = unpack(byte_num[space],data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        # print('xbee received: ', res[do])
    except:
        for i in send_pkt_num:
            for j, space in enumerate(pkt_space[i][:]):
                res[i][j] = unpack(byte_num[space],pkt_bytearray[i][sum(pkt_space[i][:j]):sum(pkt_space[i][:j+1])])[0]
            print('out: ', i, res[i])
        pass



# 127: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID", 
#         "Mode", "Arm", "MAVstatus", "Failsafe", "OTHER.systime", "checksum"],
#     128: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",  
#         "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg",
#         "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "SCALED_IMU.xacc", "SCALED_IMU.yacc", "SCALED_IMU.zacc", 
#         "Dyn_waypt_lat", "Dyn_waypt_lon", "SYSTEM_TIME.time_unix_usec", "OTHER.systime", "checksum"],
#     129: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
#             "Commandmessage", "OTHER.systime", "checksum"],
#     130: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
#             "Other_UAV_lat", "Other_UAV_lon", "Other_UAV_alt", "Other_UAV_vx", "Other_UAV_vy", "Other_UAV_vz", "Other_UAV_hdg", "Other_UAV_gpstime", "OTHER.systime", "checksum"],
# 131: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
#              "Desired_dist", "Waypt_count", "OTHER.systime", "checksum"],
#         132: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
#              "Waypt_seqID", "Formation", "Pass_radius", "lat", "lon", "alt","OTHER.systime", "checksum"],
#         133: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
#              "Mode_Arm", "OTHER.systime", "checksum"]



