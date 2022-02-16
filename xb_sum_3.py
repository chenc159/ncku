from socket import timeout
import time, math
from datetime import datetime
from struct import *
from ctypes import *
import threading
import pymap3d as pm
from pymavlink import mavutil, mavwp
from digi.xbee.devices import DigiMeshDevice,RemoteDigiMeshDevice,XBee64BitAddress
from info import info, packet127, packet128, packet129, packet130, packet131, packet132, packet133, packet134


# Connect pixhawk
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud = 57600)
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud = 57600)
msg = None
while msg == None:
    print("waiting for Heartbeat and RAW_IMU...")
    try: 
        master.wait_heartbeat() # Wait for the first heartbeat 
        msg = master.recv_match(type=['RAW_IMU'], blocking=True, timeout=1)
    except: pass
print("RAW_IMU received")
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 2 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

# # Connect xbee1
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
gps_time = c_int(0)
roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)  
lat, lon, alt, vx, vy, vz, hdg = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)     
fix, sat_num = c_int(0), c_int(0)
mode, arm, system_status, failsafe = c_int(255), c_int(255), c_int(255), c_int(255)
command, result = c_int(255), c_int(255)
Dyn_waypt_lat, Dyn_waypt_lon = c_int(0), c_int(0)

others_sysID, others_compID, others_commID = c_int(0), c_int(0), c_int(0)
others_lat, others_lon, others_alt = c_int(0), c_int(0), c_int(0)
others_vx, others_vy, others_vz, others_hdg, others_gps_time = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)

pkt= {127: packet127(sysID, compID, commID, mode, arm, system_status, failsafe),
    128: packet128(sysID, compID, commID, lat, lon, alt, fix, sat_num, vx, vy, vz, hdg, roll, pitch, yaw, xacc, yacc, zacc, Dyn_waypt_lat, Dyn_waypt_lon, gps_time),
    129: packet129(sysID, compID, commID, command, result),
    130: packet130(sysID, others_sysID, others_commID, others_lat, others_lon, others_alt, others_vx, others_vy, others_vz, others_hdg, others_gps_time),
    131: packet131(),
    132: packet132(),
    133: packet133(),
    134: packet134(sysID, compID, commID, lat, lon, alt, vx, vy, vz, xacc, yacc, xgyro, ygyro, zgyro, hdg, gps_time)
}


last_sent_time, msgID_to_send = 0, [] 
seq_togo = 0
send_mav_command = False
confirmation = 0

while True:
    try:
        if mode.value !=  list(info.mode_mapping_acm.keys())[list(info.mode_mapping_acm.values()).index(master.flightmode)]:
            mode.value = list(info.mode_mapping_acm.keys())[list(info.mode_mapping_acm.values()).index(master.flightmode)]
            msgID_to_send.extend([127])
    except:
        mode.value = 99
        msgID_to_send.extend([127])
    if arm.value != master.sysid_state[master.sysid].armed:
        arm.value = master.sysid_state[master.sysid].armed #0: disarmed, 125: armed
        msgID_to_send.extend([127])

    # Get data from pixhawk via pymavlink
    msg = None
    try:
        msg = master.recv_match(blocking=True, timeout=1)
        msg_type = msg.get_type()
    except: pass
    if msg == None:
        continue
    elif msg_type == "SYSTEM_TIME":             # gps utc time
        gpstime = datetime.utcfromtimestamp(msg.time_unix_usec/1e6)
        gps_time.value = int((gpstime.minute*60 + gpstime.second)*1e3 + round(gpstime.microsecond/1e3))
    elif msg_type == "ATTITUDE":              # imu: roll, pitch, yaw angle
        roll.value = round(msg.roll*57.2958)
        pitch.value = round(msg.pitch*57.2958)
        yaw.value = round(msg.yaw*57.2958)
    elif msg_type == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        lat.value, lon.value, alt.value = msg.lat, msg.lon, msg.relative_alt
        vx.value, vy.value, vz.value, hdg.value = msg.vx, msg.vy, msg.vz, msg.hdg
    elif msg_type == "SCALED_IMU2":           # imu: linear acceleration and angular velocity
        xacc.value, yacc.value, zacc.value = msg.xacc, msg.yacc, msg.zacc
        xgyro.value, ygyro.value, zgyro.value = msg.xgyro, msg.ygyro, msg.zgyro
    elif msg_type == "GPS_RAW_INT":           # GPS status: fix, sat_num
        fix.value, sat_num.value = msg.fix_type, msg.satellites_visible
    elif msg_type == "HEARTBEAT":             # MAV_STATE
        if system_status.value != msg.system_status:
            system_status.value = msg.system_status
            msgID_to_send.extend([127])
    elif msg_type == "HIGH_LATENCY2": # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
        if failsafe.value != int(math.log(msg.HL_FAILURE_FLAG,2)):
            failsafe.value = int(math.log(msg.HL_FAILURE_FLAG,2))
            msgID_to_send.extend([127])
    elif msg_type == "STATUSTEXT":
        if msg.severity < 4 and (failsafe.value != msg.severity+14): # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            failsafe.value = msg.severity+14
            msgID_to_send.extend([127])
    # elif msg_type == "MISSION_ACK":
    #     mission_ack.value = msg.type # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
    #     msgID_to_send.extend([129])
        # print('ack: ', mission_ack.value) 
    elif msg_type == "MISSION_CURRENT":
        current_mission_seq = msg.seq
        if (master.flightmode == 'AUTO'):
            Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[msg.seq], pkt[132].Mission_lon[msg.seq]
    elif msg_type == "COMMAND_ACK":
        command.value = msg.command # 22: NAV_TAKEOFF, 176: DO_SET_MODE, 300: MISSION_START, 400: ARM_DISARM
        result.value = msg.result # https://mavlink.io/en/messages/common.html#MAV_RESULT
        msgID_to_send.extend([129])
    elif msg_type == "SERVO_OUTPUT_RAW":
        # print(msg)
        pass

    if (time.time() - last_sent_time) >= 1.0:
        msgID_to_send.extend([128])
        msgID_to_send.extend([134])
        last_sent_time = time.time()
    
    # Send packet
    msgID_to_send = set(msgID_to_send)
    for i in msgID_to_send:
        pkt_bytearray = bytearray([255])
        pkt_bytearray.extend(pkt[i].packpkt()) # pack the pkt info
        # store computer system time and gps time
        utctime = datetime.utcnow()
        pkt_bytearray.extend(pack('i',int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))))
        # calculae checksum
        chks.accumulate(pkt_bytearray[:]) 
        pkt_bytearray.extend(pack('H', chks.crc))
        # send by xbee
        if i == 134:
            try: xbee001.send_data_broadcast(pkt_bytearray)
            except: pass
        else:
            try: xbee001.send_data(remote002,pkt_bytearray)
            except: pass
        # print(i, pkt_bytearray)
    msgID_to_send = []

    
    # Read packet
    try:
        received = xbee001.read_data()
        data = received.data
        received_msgID = data[1]
        #print('total_waypoint: ', data)
        print('received id: ', received_msgID)
        if received_msgID == 131:
            pkt[received_msgID].unpackpkt(data)
            pkt[132].mission_init(pkt[received_msgID].Waypt_count)
            print('total_count: ', pkt[received_msgID].Waypt_count)
            print('Broadcast: ', data[4])
            print('waypt_count: ', data[5])
            print('Desired_dist: ',unpack('i',data[6:10])[0])
            print('system131: ',unpack('i',data[10:14])[0])
        elif received_msgID == 132:
            pkt[received_msgID].unpackpkt(data)
            pkt[received_msgID].mission_save()
            print('Broadcast: ', data[4])
            print('waypt_seqID: ', data[5]) 
            print('Mission_mode: ', unpack('i',data[6:10])[0]) 
            print('Formation: ',  unpack('i',data[10:14])[0])           
            print('Accept_radius: ',unpack('i',data[14:18])[0])
            print('lat: ',unpack('i',data[18:22])[0])
            print('lng: ',unpack('i',data[22:26])[0])
            print('alt: ',unpack('i',data[26:30])[0])
            print('system: ',unpack('i',data[30:34])[0])
            print("!!!!", pkt[received_msgID].Mission_alt)

            # https://hamishwillee.gitbooks.io/ham_mavdevguide/content/en/services/mission.html
            if (999 not in pkt[received_msgID].Mission_alt): # if all mission waypts are received
                wp = mavwp.MAVWPLoader()
                print(data)
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                for i in range(pkt[131].Waypt_count):
                    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                        sysID, compID,
                        i,
                        frame,
                        pkt[received_msgID].Mission_modes[i],
                        0, 0, pkt[received_msgID].Mission_accept_radius[i], 0, 0, 0,
                        pkt[received_msgID].Mission_lat[i]/1e7, pkt[received_msgID].Mission_lon[i]/1e7, pkt[received_msgID].Mission_alt[i]))
                master.waypoint_clear_all_send()
                master.waypoint_count_send(wp.count())
                print(wp.count())
                result.value = 255
                start_time = time.time()
                while (result.value == 255):
                    msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=1)
                    if msg == None:
                        print('MISSION_REQUEST msg is none')
                        continue
                    print(msg)
                    print(wp.wp(msg.seq))
                    master.mav.send(wp.wp(msg.seq))
                    msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=0.1)
                    try: 
                        command.value = 999 # self-defined numbmer
                        result.value = msg.type
                        Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[0], pkt[132].Mission_lon[0]
                        print(result.value)
                    except: pass
                    if (time.time() - start_time > 30): # is time exceeds 30s, ask gcs to resend
                        command.value = 999
                        result.value = 99 # failed, please send again
                        break
                msgID_to_send.extend([129]) # send out mission_ack packet
                
        elif received_msgID == 133:
            print(data[5], data)
            current_alt, current_lon = lat.value, lon.value
            pkt[received_msgID].unpackpkt(data)
            if (pkt[received_msgID].mode_arm < 10): # disarm
                master.set_mode(pkt[received_msgID].mode_arm)
            elif (pkt[received_msgID].mode_arm == 10): # arm
                master.arducopter_arm()
            elif (pkt[received_msgID].mode_arm == 11): # disarm
                master.arducopter_disarm()
            elif (pkt[received_msgID].mode_arm == 12): # takeoff
                takeoff_alt = 20
                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, confirmation, 0, 0, 0, 0, 0, 0, takeoff_alt)
                send_mav_command = True
            elif (pkt[received_msgID].mode_arm == 13): # mission start
                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_MISSION_START, confirmation, 0, 0, 0, 0, 0, 0, 0)
                send_mav_command = True
        
        elif received_msgID == 134: # received v2v
            others_sysID.value, others_compID.value, others_commID.value, others_lat.value, others_lon.value, others_alt.value, others_vx.value, others_vy.value, others_vz.value, others_hdg.value, others_gps_time.value = pkt[received_msgID].unpackpkt(data)
            msgID_to_send.extend([130]) # send out v2g regarding neighboring info
    except: pass


    if (command.value == 22) and (result.value == 0) and (pkt[133].mode_arm == 12) and send_mav_command:
        master.set_mode(4) # guided
        send_mav_command = False
    
    if (command.value == 300) and (result.value == 0) and (pkt[133].mode_arm == 13) and send_mav_command:
        master.set_mode(3) # auto
        send_mav_command = False

    '''
    # for guided set global position: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    if (master.flightmode == 'GUIDED') and (len(pkt[132].Mission_alt)!=0) and (999 not in pkt[132].Mission_alt):
        dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value, pkt[132].Mission_lat[seq_togo]/1e7, pkt[132].Mission_lon[seq_togo]/1e7, pkt[132].Mission_alt[seq_togo])
        if (seq_togo < pkt[131].Waypt_count - 1) and (dx**2 + dy**2 + dz**2 <= pkt[131].Desired_dist**2):
            seq_togo += 1
            Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[seq_togo], pkt[132].Mission_lon[seq_togo]
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, sysID, compID, 3, int(0b110111111000), 
            pkt[132].Mission_lat[seq_togo], pkt[132].Mission_lon[seq_togo], pkt[132].Mission_alt[seq_togo], 0, 0, 0, 0, 0, 0, 0, 0))
        # master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 
        #     pkt[132].Mission_lat[seq_togo], pkt[132].Mission_lon[seq_togo], pkt[132].Mission_alt[seq_togo])
        # msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    '''     



    # if (time.time() - pkt[133].time < 3.0):
    #     if (pkt[133].mode_arm < 10) and (pkt[133].mode_arm != mode.value): # change mode
    #         t4 = threading.Thread(master.set_mode(pkt[133].mode_arm))
    #         t4.start()
    #         # master.set_mode(pkt[133].mode_arm)
    #     elif (pkt[133].mode_arm == 10) and not master.sysid_state[master.sysid].armed: # arm
    #         t5 = threading.Thread(master.arducopter_arm())
    #         t5.start()
    #         # master.motors_armed_wait()
    #     elif (pkt[133].mode_arm == 11) and master.sysid_state[master.sysid].armed: # disarm
    #         t6 = threading.Thread(master.arducopter_disarm())
    #         t6.start()
    #     elif (pkt[133].mode_arm == 12): # takeoff
    #         takeoff_alt = 20
    #         t7 = threading.Thread(master.set_mode(4)) # guided
    #         t7.start()
    #         # master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, sysID, compID, 3, int(0b110111111000), 
    #         #     current_alt, current_lon, takeoff_alt, 0, 0, 0, 0, 0, 0, 0, 0))
    #         master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)
    #         # msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    #     elif (pkt[133].mode_arm == 13): # mission start
    #         t8 = threading.Thread(master.set_mode(3)) # auto
    #         t8.start()
    #         master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)


    '''v2v to gcs relative distance and heading
    another packet for guided start (mission waypoints)
    failsafe seperate to two
    '''

    '''
    gcs: receive command ack every command (arm/disarm/set_mode/takeoff/mission start)
    present command ack in a way that history shows.... can see the previous command ack
    '''
