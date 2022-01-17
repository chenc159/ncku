import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy
from ctypes import *


# # Connect xbee1
# xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
# xbee001.open(force_settings=True)

# # Connect xbee2
# xbee002 = DigiMeshDevice('/dev/ttyUSB1', 115200)
# xbee002.open(force_settings=True)

# pkt1 = bytearray([header, commID, sysID, compID, mesID[0]])
# pkt1.extend(pack('ii', GPSACC_time_boot, SYS_time))
# pkt1.extend(bytearray([MAV_state, checksum]))

# pkt2 = bytearray([header, commID, sysID, compID, mesID[1]])
# pkt2.extend(pack('9i', GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading))
# pkt2.extend(bytearray([checksum]))

# # Packet 1 (9 in total):    Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), UAV component ID (1 byte), MessageID=128 (1 byte),
# #  GPS time (4 byte), system time (4 byte), uav system mav system (1 byte), checksum (1 byte)
# # Space: [1,1,1,1,1,4,4,1,1]



def failsafe_map(num):
    '''
    0~13 check HIGH_LATENCY2.HL_FAILURE_FLAG and https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
    14~17 check STATUSTEXT.severity and https://mavlink.io/en/messages/common.html#MAV_SEVERITY
    99 for None/not getting any messages regarding failsafe
    '''
    return {
        0:    "HL_FAILURE_FLAG_GPS",
        1:    "HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE",
        2:    "HL_FAILURE_FLAG_ABSOLUTE_PRESSURE",
        3:    "HL_FAILURE_FLAG_3D_ACCEL",
        4:    "HL_FAILURE_FLAG_3D_GYRO",
        5:    "HL_FAILURE_FLAG_3D_MAG",
        6:    "HL_FAILURE_FLAG_TERRAIN",
        7:    "HL_FAILURE_FLAG_BATTERY",
        8:    "HL_FAILURE_FLAG_RC_RECEIVER",
        9:    "HL_FAILURE_FLAG_OFFBOARD_LINK",
        10:   "HL_FAILURE_FLAG_ENGINE",
        11:   "HL_FAILURE_FLAG_GEOFENCE",
        12:   "HL_FAILURE_FLAG_ESTIMATOR",
        13:   "HL_FAILURE_FLAG_MISSION",
        14:   "MAV_SEVERITY_EMERGENCY",
        15:   "MAV_SEVERITY_ALERT",
        16:   "MAV_SEVERITY_CRITICAL",
        17:   "MAV_SEVERITY_ERROR",
        99:   "None"
    }.get(num)


# int(math.log(msg.HL_FAILURE_FLAG,2))
# if msg.severity < 4:
# msg.severity += 14

msgs =  {   
    "SYSTEM_TIME":          {"time_unix_usec": 0, "time_boot_ms": 0}, 
    "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
    "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
    "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
    "HEARTBEAT":            {"system_status": 99},
    "BATTERY_STATUS":       {"battery_remaining": 0},
    "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
    "STATUSTEXT":           {"severity": 99}
}

msgs_c =  {   
    "SYSTEM_TIME":          {"time_unix_usec": c_int(0), "time_boot_ms": c_int(0)}, 
    "ATTITUDE":             {"time_boot_ms": c_int(0), "roll": c_int(0), "pitch": c_int(0), "yaw": c_int(0)},
    "GPS_RAW_INT":          {"time_usec": c_int(0), "fix_type": c_int(0), "satellites_visible": c_int(0)},
    "GLOBAL_POSITION_INT":  {"time_boot_ms": c_int(0), "lat": c_int(0), "lon": c_int(0), "alt": c_int(0), "vx": c_int(0), "vy": c_int(0), "vz": c_int(0), "hdg": c_int(0)},
    "HEARTBEAT":            {"system_status": c_int(99)},
    "BATTERY_STATUS":       {"battery_remaining": c_int(0)},
    "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": c_int(99)},
    "STATUSTEXT":           {"severity": c_int(99)}
}

msgs_p = {}
for key1 in msgs_c.keys():
    # print(key1)
    msgs_p[key1] = {}
    for key2 in msgs_c[key1].keys():
        msgs_p[key1][key2] = pointer(msgs_c[key1][key2])

res = [msgs_c["SYSTEM_TIME"]["time_unix_usec"], msgs_c["SYSTEM_TIME"]["time_unix_usec"]]
print(res)
msgs_p["SYSTEM_TIME"]["time_unix_usec"][0] = 888
print(msgs_c["SYSTEM_TIME"]["time_unix_usec"])
print(msgs_c)
print(res)

# class PACKET1(Structure):
#     _field_ =  [("header", c_int), 
#                 ("commID", c_int),
#                 ("sysID", POINTER(c_int))]


# i = c_int(99)
# j = i
# pi = pointer(i)

# class packet1():
#     header = i

# # print(j.value)
# pk1=packet1()
# # print(pi[0])
# print(pk1.header)
# i=c_int(8)
# pi[0] = 7
# print(i.value)
# print(pk1.header)



# store packet values using the pointer method
# for every sending rate, regenerate the bytearray for xbee


