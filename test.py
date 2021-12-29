import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy

# # Connect xbee1
# xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
# xbee001.open(force_settings=True)

# # Connect xbee2
# xbee002 = DigiMeshDevice('/dev/ttyUSB1', 115200)
# xbee002.open(force_settings=True)



# while True:
#     send = bytearray(2)
#     send[0] = 22
#     send[1] = 25
#     send.extend(pack('ii',9898989,10))
#     # xbee001.send_data_broadcast("hello")
#     xbee001.send_data_broadcast(send)

#     xbee_message = xbee002.read_data()
#     try:
#         data = xbee_message.data
#         print(data, data[0], data[1], unpack('i',data[2:2+4])[0], unpack('i',data[2+4:2+8])[0])
#         # print(data[0], data[1])
#     except:
#         pass
# # xbee002.

# header = [bytearray([2])]
# value = 66
# hey = [value]
# pk = {"header": header, 'v': value, 'h': hey}

# class packet():
#     def __init__(self):
#         self.header = header
#         self.lat = 999
#         self.h = hey
        
# p1 = packet()
# a = header
# # print(p1.header)
# # print(pk)
# # print(a)
# header[0] = bytearray([3])
# header.append(bytearray([value]))
# print(header)
# print(hey)
# value = 777
# print(header)
# print(hey)
# # hey[0] 
# # print(p1.h)
# # print(pk)
# # print(a)


# a,b,c = [3],[4],[5]
# g = [a,b,c]
# j = sum(g,[])
# h = g
# print(h)
# a[0] = 900
# # g[2] = bytearray([100])
# print(h)
# print(j)

# header = [1]
# checksum = [255]
# header_byt = [bytearray(header)]
# checksum_byt = [bytearray(checksum)]
# packet1 = [header_byt, checksum_byt]
# print(packet1)
# checksum[0] = 1
# checksum_byt[0] = bytearray(checksum)
# print(packet1)
# packet1_sum = sum(packet1,[])
# print(packet1_sum)


# pkt1 = bytearray([header, commID, sysID, compID, mesID[0]])
# pkt1.extend(pack('ii', GPSACC_time_boot, SYS_time))
# pkt1.extend(bytearray([MAV_state, checksum]))

# pkt2 = bytearray([header, commID, sysID, compID, mesID[1]])
# pkt2.extend(pack('9i', GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading))
# pkt2.extend(bytearray([checksum]))

# # Packet 1 (9 in total):    Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), UAV component ID (1 byte), MessageID=128 (1 byte),
# #  GPS time (4 byte), system time (4 byte), uav system mav system (1 byte), checksum (1 byte)
# # Space: [1,1,1,1,1,4,4,1,1]


# pkt1 = {}

# elif msg.get_type() == "SYSTEM_TIME":             # system boot time
#     SYS_time = msg.time_boot_ms
#     # cs.accumulate(msg)
#     # print('heyyyy', cs.crc)
# elif msg.get_type() == "ATTITUDE":              # imu: time, roll, pitch, yaw
#     IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw
# elif msg.get_type() == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
#     GPS_time_usec, fix, num = msg.time_usec, msg.fix_type, msg.satellites_visible
# elif msg.get_type() == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
#     GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat/1e7, msg.lon/1e7, msg.alt/1e3 # originally in degE7 and mm
#     vx, vy, vz, heading = msg.vx/100.0, msg.vy/100.0, msg.vz/100.0, msg.hdg/100.0 # originally in cm/s and cdeg    
# elif msg.get_type() == "HEARTBEAT":             # MAV_STATE
#     # MAV_state = system_status(msg.system_status)
#     MAV_state = msg.system_status
#     # cs.accumulate(bytearray([MAV_state]))
#     # print('heyyyy', cs.crc)
# elif msg.get_type() == "BATTERY_STATUS":        # Battery status
#     battery = msg.battery_remaining
# elif msg.get_type() == "HIGH_LATENCY2":
#     if msg.HL_FAILURE_FLAG > 0: # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
#         failsafe = False
# elif msg.get_type() == "STATUSTEXT":
#     if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
#         failsafe = False

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
    "SYSTEM_TIME":          {"time_boot_ms": 0}, 
    "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
    "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
    "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
    "HEARTBEAT":            {"system_status": 99},
    "BATTERY_STATUS":       {"battery_remaining": 0},
    "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
    "STATUSTEXT":           {"severity": 99}
}

# for key1 in msgs.keys():
#     print(key1)
#     for key2 in msgs[key1].keys():
#         print(key2)

print(msgs)






