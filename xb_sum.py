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

# Initialize parameters for drone data
sysID, compID = master.target_system, master.target_component
sysID_b, compID_b = [bytearray([sysID])], [bytearray([compID])]
SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot = [pack('i',0)], [pack('i',0)], [pack('i',0)], [pack('i',0)]   # in ms
'''need to change to int to send via xbee'''
roll, pitch, yaw = [0], [0], [0]                                                # in deg 
fix, num, lat, lon, alt = [0], [0], [0], [0], [0]                               # in degE7 and mm
vx, vy, vz, heading = [0], [0], [0], [0]                                        # in cm/s and cdeg
MAV_state, battery, failsafe = [bytearray([0])], [0], [1]                                    # int, num in %, bool

# Initialize packet
header, checksum, commID = 255, 255, 22
header_b, checksum_b, commID_b = [bytearray([header])], [bytearray([checksum])], [bytearray([commID])]
mesID = [128, 129]
pkt_len = {1: 9, 2: 15}
pkt_space = {1: [1,1,1,1,1,4,4,1,1], 2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,1]}
res = {1: [0 for i in range(pkt_len[1])], 2: [0 for i in range(pkt_len[2])]}
send_num = [1,2]

class pkt1():
    def __init__(self):
        self.header, self.commID, self.sysID, self.compID, self.mesID = header_b, commID_b, sysID_b, compID_b, [bytearray([mesID[0]])]
        self.GPSACC_time_boot, self.SYS_time, self.MAV_state = GPSACC_time_boot, SYS_time, MAV_state
        self.checksum = checksum_b
        self.byte_list = [header_b, commID_b, sysID_b, compID_b, [bytearray([mesID[0]])], GPSACC_time_boot, SYS_time, MAV_state, checksum_b]
        self.byte_array = sum(self.byte_list, [])

        # print(self.byte_list)
        # print(self.byte_array)

pkt11=pkt1()

# do = 1
# for i, space in enumerate(pkt_space[do][:]):
#     if space == 1:
#         res[do][i] = pkt11.byte_array[i]
#     else:
#         print(pkt11.byte_array[i])
#         res[do][i] = unpack('i',pkt11.byte_array[i])[0]
#         # res[do][i] = unpack('i',pkt11.byte_array[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
# print('out: ', res[do])

cs = mavutil.x25crc()
pkt1 = bytearray([header, commID, sysID, compID, mesID[0]])
pkt1.extend(pack('ii', 9999, 8888))
# pkt1.extend(GPSACC_time_boot)
print(pkt1)
cs.accumulate(pkt1)
print('heyyyy', cs.crc)
print('crc', pack('H', cs.crc))

# Packet 1 (9 in total):    Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), UAV component ID (1 byte), MessageID=128 (1 byte),
#  GPS time (4 byte), system time (4 byte), uav system mav system (1 byte), checksum (1 byte)
# Space: [1,1,1,1,1,4,4,1,1]
# Packet 2 (14 in total):   Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), UAV component ID (1 byte), MessageID=129 (1 byte),
#  GPS time (4 byte), system time (4 byte), lat (4 byte), lon (4 byte), alt (4 byte), vx (4 byte), vy (4 byte), vz (4 byte), heading (4 byte), checksum (1 byte)
# Space: [1,1,1,1,4,4,4,4,4,4,4,4,4,1]

# def message_checksum(msg):
#     '''calculate a 8-bit checksum of the key fields of a message, so we
#        can detect incompatible XML changes'''
#     from .mavcrc import x25crc
#     crc = x25crc()
#     crc.accumulate_str(msg.name + ' ')
#     # in order to allow for extensions the crc does not include
#     # any field extensions
#     crc_end = msg.base_fields()
#     for i in range(crc_end):
#         f = msg.ordered_fields[i]
#         crc.accumulate_str(f.type + ' ')
#         crc.accumulate_str(f.name + ' ')
#         if f.array_length:
#             crc.accumulate([f.array_length])
#     return (crc.crc&0xFF) ^ (crc.crc>>8)


'''

while xbee001.is_open():
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    if msg.get_type() == "SYSTEM_TIME":             # system boot time
        SYS_time = msg.time_boot_ms
    elif msg.get_type() == "ATTITUDE":              # imu: time, roll, pitch, yaw
        IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, round(msg.roll*57.2958), round(msg.pitch*57.2958), round(msg.yaw*57.2958)
    elif msg.get_type() == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
        GPS_time_usec, fix, num = msg.time_usec, msg.fix_type, msg.satellites_visible
    elif msg.get_type() == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat, msg.lon, msg.alt # originally in degE7 and mm
        vx, vy, vz, heading = msg.vx, msg.vy, msg.vz, msg.hdg # originally in cm/s and cdeg    
    elif msg.get_type() == "HEARTBEAT":             # MAV_STATE
        MAV_state = msg.system_status
    elif msg.get_type() == "BATTERY_STATUS":        # Battery status
        battery = msg.battery_remaining
    elif msg.get_type() == "HIGH_LATENCY2":
        if msg.HL_FAILURE_FLAG > 0: # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
            failsafe = False
    elif msg.get_type() == "STATUSTEXT":
        if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            failsafe = False

    # mavfile_state, master.sysid_state, flightmode

    
    if 1 in send_num:
        print('in: ', header, commID, sysID, compID, mesID[0], GPSACC_time_boot, SYS_time, MAV_state, checksum)
        pkt1 = bytearray([header, commID, sysID, compID, mesID[0]])
        pkt1.extend(pack('ii', GPSACC_time_boot, SYS_time))
        pkt1.extend(bytearray([MAV_state, checksum]))
        xbee001.send_data_broadcast(pkt1)
    if 2 in send_num:
        print('in: ', header, commID, sysID, compID, mesID[1], GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading, checksum)
        pkt2 = bytearray([header, commID, sysID, compID, mesID[1]])
        pkt2.extend(pack('9i', GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading))
        pkt2.extend(bytearray([checksum]))
        xbee001.send_data_broadcast(pkt2)

    try:
        received = xbee002.read_data()
        data = received.data
        do = mesID.index(data[4]) + 1
        for i, space in enumerate(pkt_space[do][:]):
            if space == 1:
                res[do][i] = data[sum(pkt_space[do][:i])]
            else:
                res[do][i] = unpack('i',data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('out: ', res[do])
    except:
        pass

'''
