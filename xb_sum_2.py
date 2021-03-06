import time, math
from datetime import datetime
from struct import *
from pymavlink import mavutil, mavwp
from digi.xbee.devices import DigiMeshDevice,RemoteDigiMeshDevice,XBee64BitAddress
from ctypes import *
from info import info

# Connect pixhawk
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud = 57600)
master.wait_heartbeat() # Wait for the first heartbeat 
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 4 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

# Connect xbee1
xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
remote002 = RemoteDigiMeshDevice(xbee001,XBee64BitAddress.from_hex_string("0013A20040F5C5DB"))
xbee001.open(force_settings=True)

# # Connect xbee2
# xbee002 = DigiMeshDevice('/dev/ttyUSB2', 115200)
# xbee002.open(force_settings=True)

# Get checksum
chks = mavutil.x25crc()

# Initialize parameters
sysID, compID, commID = master.target_system, master.target_component, 22
# systime, gpstime = 0, 0
# roll, pitch, yaw, xacc, yacc, zacc = 0,0,0,0,0,0                   # in deg, mG
# lat, lon, alt, vx, vy, vz, hdg = 0,0,0,0,0,0,0              # in degE7, mm, cm/s, cdeg
fix, sat_num = 0, 0
last_sent_time, msgID_to_send = 0, [] 

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
    for j in range(1, len(pkt_item[i])-1):
        pkt_bytearray[i].extend(pack(byte_num[pkt_space[i][j]], pkt_val[i][j]))
    pkt_bytearray[i].extend(pack(byte_num[2], info.checksum))
pkt_val[127][pkt_item[127].index('Mode')], pkt_val[127][pkt_item[127].index('Arm')] = 255, 255
pkt_val[127][pkt_item[127].index('HEARTBEAT.system_status')], pkt_val[127][pkt_item[127].index('Failsafe')] = 255, 255

while True:
    if pkt_val[127][pkt_item[127].index('Mode')] !=  list(info.mode_mapping_acm.keys())[list(info.mode_mapping_acm.values()).index(master.flightmode)]:
        pkt_val[127][pkt_item[127].index('Mode')] = list(info.mode_mapping_acm.keys())[list(info.mode_mapping_acm.values()).index(master.flightmode)]
        msgID_to_send.extend([127])
    if pkt_val[127][pkt_item[127].index('Arm')] != master.sysid_state[master.sysid].armed:
        pkt_val[127][pkt_item[127].index('Arm')] = master.sysid_state[master.sysid].armed
        print("armed!!!!!!!!!!", pkt_val[127][pkt_item[127].index('Arm')])
        msgID_to_send.extend([127])

    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if msg == None:
        continue
    elif msg_type == "SYSTEM_TIME":             # gps utc time
        gpstime = datetime.utcfromtimestamp(msg.time_unix_usec/1e6)
        pkt_val[128][pkt_item[128].index('SYSTEM_TIME.time_unix_usec')] = int((gpstime.minute*60 + gpstime.second)*1e3 + round(gpstime.microsecond/1e3))
    elif msg_type == "ATTITUDE":              # imu: time, roll, pitch, yaw
        pkt_val[128][pkt_item[128].index('ATTITUDE.roll')] = round(msg.roll*57.2958)
        pkt_val[128][pkt_item[128].index('ATTITUDE.pitch')] = round(msg.pitch*57.2958)
        pkt_val[128][pkt_item[128].index('ATTITUDE.yaw')] = round(msg.yaw*57.2958)
    elif msg_type == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
        fix, sat_num = msg.fix_type, msg.satellites_visible
    elif msg_type == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.lat')] = msg.lat
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.lon')] = msg.lon
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.alt')] = msg.alt
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vx')] = msg.vx
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vy')] = msg.vy
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.vz')] = msg.vz
        pkt_val[128][pkt_item[128].index('GLOBAL_POSITION_INT.hdg')] = msg.hdg
    elif msg_type == "SCALED_IMU2":
        pkt_val[128][pkt_item[128].index('SCALED_IMU2.xacc')] = msg.xacc
        pkt_val[128][pkt_item[128].index('SCALED_IMU2.yacc')] = msg.yacc
        pkt_val[128][pkt_item[128].index('SCALED_IMU2.zacc')] = msg.zacc

    elif msg_type == "HEARTBEAT":             # MAV_STATE
        if pkt_val[127][pkt_item[127].index('HEARTBEAT.system_status')] != msg.system_status:
            pkt_val[127][pkt_item[127].index('HEARTBEAT.system_status')] = msg.system_status
            msgID_to_send.extend([127])
    elif msg_type == "HIGH_LATENCY2": # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
        if pkt_val[127][pkt_item[127].index('Failsafe')] != int(math.log(msg.HL_FAILURE_FLAG,2)):
            pkt_val[127][pkt_item[127].index('Failsafe')] = int(math.log(msg.HL_FAILURE_FLAG,2))
            msgID_to_send.extend([127])
    elif msg_type == "STATUSTEXT":
        if msg.severity < 4 and (pkt_val[127][pkt_item[127].index('Failsafe')] != msg.severity+14): # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            pkt_val[127][pkt_item[127].index('Failsafe')] = msg.severity+14
            msgID_to_send.extend([127])

    if (time.time() - last_sent_time) >= 1.0:
        msgID_to_send.extend([128])
        last_sent_time = time.time()
    
    # Send packet
    msgID_to_send = set(msgID_to_send)
    for i in msgID_to_send:
        # store computer system time and gps time
        utctime = datetime.utcnow()
        pkt_val[i][-2] = int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))
        print(pkt_val[i])
        for j in range(5,len(pkt_item[i])-1):
            pkt_bytearray[i][sum(pkt_space[i][:j]):sum(pkt_space[i][:j+1])] = pack(byte_num[pkt_space[i][j]], pkt_val[i][j])
        # calculae checksum
        chks.accumulate(pkt_bytearray[i][:-2]) 
        pkt_bytearray[i][-2:] = pack(byte_num[2], chks.crc)
        # send by xbee
        try: xbee001.send_data(remote002,pkt_bytearray[i])
        # try: xbee001.send_data_broadcast(pkt_bytearray[i])
        except: pass
    msgID_to_send = []

    # Read packet
    try:
        received = xbee001.read_data()
        data = received.data
        received_msgID = data[1]
        #print('total_waypoint: ', data)
        print('received id: ', received_msgID)
        if received_msgID == 131:
            j = pkt_item[received_msgID].index('Desired_dist')
            Desired_dist = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('Waypt_count')
            Waypt_count = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            print('total_count: ', Waypt_count)
            print('Broadcast: ', data[4])
            print('waypt_count: ', data[5])
            print('Desired_dist: ',unpack('i',data[6:10])[0])
            print('system131: ',unpack('i',data[10:14])[0])
            Mission_seq = [k for k in range(Waypt_count)]
            Mission_modes = [999 for k in range(Waypt_count)]
            Mission_formation, Mission_pass_radius = [999 for k in range(Waypt_count)], [999 for k in range(Waypt_count)]
            Mission_lat, Mission_lon, Mission_alt = [999 for k in range(Waypt_count)], [999 for k in range(Waypt_count)], [999 for k in range(Waypt_count)]
        elif received_msgID == 132:
            print('Broadcast: ', data[4])
            print('waypt_seqID: ', data[5]) 
            print('Mission_mode: ', unpack('i',data[6:10])[0]) 
            print('Formation: ',  unpack('i',data[10:14])[0])           
            print('Pass_radius: ',unpack('i',data[14:18])[0])
            print('lat: ',unpack('i',data[18:22])[0])
            print('lng: ',unpack('i',data[22:26])[0])
            print('alt: ',unpack('i',data[26:30])[0])
            print('system: ',unpack('i',data[30:34])[0])
            j = pkt_item[received_msgID].index('Waypt_seqID')
            seq = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('Mission_mode')
            num = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            Mission_modes[seq] = info.mission_mode_mapping[num]
            j = pkt_item[received_msgID].index('Formation')
            Mission_formation[seq] = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('Pass_radius')
            Mission_pass_radius[seq] = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('lat')
            Mission_lat[seq] = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('lon')
            Mission_lon[seq] = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            j = pkt_item[received_msgID].index('alt')
            Mission_alt[seq] = unpack(byte_num[pkt_space[received_msgID][j]],data[sum(pkt_space[received_msgID][:j]):sum(pkt_space[received_msgID][:j+1])])[0]
            print("!!!!", Mission_alt)

            if (999 not in Mission_alt):
                wp = mavwp.MAVWPLoader()
                print(data)
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                for i in range(Waypt_count):
                    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                        sysID, compID,
                        Mission_seq[i],
                        frame,
                        Mission_modes[i],
                        0, 0, Desired_dist, Mission_pass_radius[i], 0, 0,
                        Mission_lat[i]/1e7, Mission_lon[i]/1e7, Mission_alt[i]))
                master.waypoint_clear_all_send()
                master.waypoint_count_send(wp.count())
                print(wp.count())
                for i in range(wp.count()):
                    msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
                    print(msg)
                    print(wp.wp(msg.seq))
                    master.mav.send(wp.wp(msg.seq))
                   # inp = input('waiting...')
                msg = master.recv_match(type=['MISSION_ACK'],blocking=True)
                mission_ack = msg.type # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
                print('ack: ', mission_ack)
                # print("mission result: ", mission_ack)
                # msgID_to_send.extend(the_packet_that_includes_mission)
                # MAV_CMD_MISSION_START ?? or just switch to AUTO mode??
        
        # for guided set global position: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # master.mav.send(mavutil.mavlink.SET_POSITION_TARGET_GLOBAL_INT(10, sysID, compID, 3,
        #     int(0b110111111000), lat, lon, alt, 0, 0, 0, 0, 0, 0, 0, 0))

                 

        elif received_msgID == 133:
            print(data[5], data)
            if int(data[5]) < 10:
                master.set_mode(int(data[5]))
            elif int(data[5]) == 10:
                master.arducopter_arm()
                master.motors_armed_wait()
            elif int(data[5]) == 11:
                master.arducopter_disarm()
    except:
        pass




