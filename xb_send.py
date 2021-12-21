import time
from struct import *
from pymavlink import mavutil
from digi.xbee.devices import DigiMeshDevice


# Connect pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0')
master.wait_heartbeat() # Wait for the first heartbeat 
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Connect xbee1
xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
xbee001.open(force_settings=True)

# Connect xbee2
xbee002 = DigiMeshDevice('/dev/ttyUSB1', 115200)
xbee002.open(force_settings=True)

# Initialize parameters for drone data
sysID, compID = master.target_system, master.target_component
SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot = 0, 0, 0, 0   # in ms
'''need to change to int to send via xbee'''
roll, pitch, yaw = 0.0, 0.0, 0.0                                        # in rad 
fix, num, lat, lon, alt = 0, 0, 0, 0, 0                                 # in degE7 and mm
vx, vy, vz, heading = 0, 0, 0, 0                                        # in cm/s and cdeg
MAV_state, battery, failsafe = 0, 0, True                               # int, num in %, bool

# Initialize packet
header, checksum, commID, mesID_1, mesID_2 = 255, 255, 22, 128, 129
'''use dictionary .... easiler....'''
pkt1_len, pkt2_len = 9, 15 # Packet lengh
space1, space2 = [1,1,1,1,1,4,4,1,1], [1,1,1,1,1,4,4,4,4,4,4,4,4,4,1] # space allocation
res1, res2 = [0 for i in range(pkt1_len)], [0 for i in range(pkt2_len)] # results

# Packet 1 (9 in total):    Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), UAV component ID (1 byte), MessageID=128 (1 byte),
#  GPS time (4 byte), system time (4 byte), uav system mav system (1 byte), checksum (1 byte)
# Space: [1,1,1,1,1,4,4,1,1]
# Packet 2 (14 in total):   Header (1 byte), Communication ID (designed = 22) (1 byte), UAV system ID (1=leader, 2,3=follower) (1 byte), MessageID=129 (1 byte),
#  GPS time (4 byte), system time (4 byte), lat (4 byte), lon (4 byte), alt (4 byte), vx (4 byte), vy (4 byte), vz (4 byte), heading (4 byte), checksum (1 byte)
# Space: [1,1,1,1,4,4,4,4,4,4,4,4,4,1]


while xbee001.is_open():
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    if msg.get_type() == "SYSTEM_TIME":             # system boot time
        SYS_time = msg.time_boot_ms
    elif msg.get_type() == "ATTITUDE":              # imu: time, roll, pitch, yaw
        IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw
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

    # print('in: ', header, commID, sysID, compID, mesID_1, GPSACC_time_boot, SYS_time, MAV_state, checksum)
    # pkt1 = bytearray([header, commID, sysID, compID, mesID_1])
    # pkt1.extend(pack('ii', GPSACC_time_boot, SYS_time))
    # pkt1.extend(bytearray([MAV_state, checksum]))
    # xbee001.send_data_broadcast(pkt1)

    print('in: ', header, commID, sysID, compID, mesID_2, GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading, checksum)
    pkt2 = bytearray([header, commID, sysID, compID, mesID_2])
    pkt2.extend(pack('9i', GPSACC_time_boot, SYS_time, lat, lon, alt, vx, vy, vz, heading))
    pkt2.extend(bytearray([checksum]))
    xbee001.send_data_broadcast(pkt2)

    try:
        received = xbee002.read_data()
        data = received.data
        for i, space in enumerate(space2):
            if space == 1:
                res2[i] = data[sum(space2[:i])]
            else:
                res2[i] = unpack('i',data[sum(space2[:i]):sum(space2[:i+1])])[0]
        print('out: ', res2)
    except:
        pass


