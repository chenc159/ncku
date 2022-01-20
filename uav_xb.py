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
sysID, compID, commID = c_int(master.target_system), c_int(master.target_component), c_int(22)
systime, gpstime = c_int(0), c_int(0)
roll, pitch, yaw, xacc, yacc, zacc = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)                   # in deg, mG
lat, lon, alt, vx, vy, vz, hdg = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)              # in degE7, mm, cm/s, cdeg
Mode, Arm, MAV_state, failsafe = c_int(255), c_int(255), c_int(255), c_int(255)

# sysID_p, compID_p = pointer(sysID), pointer(compID)
systime_p, gpstime_p = pointer(systime), pointer(gpstime)
roll_p, pitch_p, yaw_p, xacc_p, yacc_p, zacc_p = pointer(roll), pointer(pitch), pointer(yaw), pointer(xacc), pointer(yacc), pointer(zacc)                   
lat_p, lon_p, alt_p, vx_p, vy_p, vz_p, hdg_p = pointer(lat), pointer(lon), pointer(alt), pointer(vx), pointer(vy), pointer(vz), pointer(hdg)
Mode_p, Arm_p, MAV_state_p, failsafe_p = pointer(Mode), pointer(Arm), pointer(MAV_state), pointer(failsafe)

msgID_send, msgID_receive = info.msgID_send, info.msgID_receive
pkt_item, pkt_space = info.pkt_item, info.pkt_space
convert, byte_num = info.convert, info.byte_num
pkt_val, pkt_bytearray, res = {}, {}, {}
for i in msgID_send:
    pkt_val[i] = [c_int(0) for k in range(len(pkt_item[i]))]
    pkt_val[i][0], pkt_val[i][1], pkt_val[i][-2] = c_int(info.header), c_int(i), c_int(systime)
    pkt_val[i][2], pkt_val[i][3], pkt_val[i][4] = sysID, compID, commID

pkt_val[127][5], pkt_val[127][6], pkt_val[127][7], pkt_val[127][8] = Mode, Arm, MAV_state, failsafe
pkt_val[128][5], pkt_val[128][6], pkt_val[128][7], pkt_val[128][8], pkt_val[128][9], pkt_val[128][10], pkt_val[128][11] = lat, lon, alt, vx, vy, vz, hdg
pkt_val[128][12], pkt_val[128][13], pkt_val[128][14], pkt_val[128][15], pkt_val[128][16], pkt_val[128][17] = roll, pitch, yaw, xacc, yacc, zacc
# pkt_val[128][18], pkt_val[128][19] = "Dyn_waypt_lat", "Dyn_waypt_lon"
pkt_val[128][20] = gpstime





pkt_item = {
    # standard packet
    0: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "OTHER.systime", "checksum"],
    # UAV to GCS
    127: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID", 
        "Mode", "Arm", "HEARTBEAT.system_status", "Failsafe", "OTHER.systime", "checksum"],
    128: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",  
        "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg",
        "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "SCALED_IMU.xacc", "SCALED_IMU.yacc", "SCALED_IMU.zacc", 
        "Dyn_waypt_lat", "Dyn_waypt_lon", "SYSTEM_TIME.time_unix_usec", "OTHER.systime", "checksum"],
    129: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "Commandmessage", "OTHER.systime", "checksum"],
    130: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "Other_UAV_lat", "Other_UAV_lon", "Other_UAV_alt", "Other_UAV_vx", "Other_UAV_vy", "Other_UAV_vz", "Other_UAV_hdg", "Other_UAV_gpstime", "OTHER.systime", "checksum"],
    # GCS to UAV
    131: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "Desired_dist", "Waypt_count", "OTHER.systime", "checksum"],
    132: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "Waypt_seqID", "Formation", "Pass_radius", "lat", "lon", "alt","OTHER.systime", "checksum"],
    133: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
            "Mode_Arm", "OTHER.systime", "checksum"],
}

'''
msgs = info.msgs
msgs["ID"]["sys"], msgs["ID"]["comp"] = master.target_system, master.target_component
# Use c struct and pointer to automatically store values
msgs_c, msgs_p = {}, {}
for key1 in msgs.keys():
    msgs_c[key1], msgs_p[key1] = {}, {}
    for key2 in msgs[key1].keys():
        msgs_c[key1][key2] = c_int(msgs[key1][key2])
        msgs_p[key1][key2] = pointer(msgs_c[key1][key2])
'''


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
        
