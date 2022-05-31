from ast import For
from asyncio import FastChildWatcher
from socket import timeout
import time, math
import sys
import os
import csv
from datetime import datetime
from struct import *
from ctypes import *
import threading
import plan
from serial.serialutil import SerialException
import pymap3d as pm
from pymavlink import mavutil, mavwp
from digi.xbee.devices import DigiMeshDevice,RemoteDigiMeshDevice,XBee64BitAddress
from info import * 

# Write data to csv
def write_csv(data):
    utctime = datetime.utcnow() # let file name be the utc time when its write (-m-d-h-m-s:month,day,hour,minute,sec)
    file_time = str(utctime.month) + "-" + str(utctime.day) + "-" + str(utctime.hour) + "-" + str(utctime.minute) + "-" + str(utctime.second)
    address = os.path.dirname(os.path.realpath('__file__')) + '/result/' + file_time + ".csv"
    with open(address, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(data)
    print(address + " saved!!")
    time.sleep(1)

# To save data to csv, the first argument shall be 1
# Second argument shall give the data saving frequency (default 1 Hz)
data_list = [['time', 'mode', 'lat', 'lon', 'alt', 'dlat', 'dlon', 'dalt', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']]
data_list_n = [['time', 'neighboring_id', 'relative_dis', 'relative_ang']]
last_save_time = time.time()
if (len(sys.argv) >= 2) and (sys.argv[1] == str(1)):
    save_csv = True
    print('Will save data to csv file when armed!')
    if (len(sys.argv) > 2):
        save_freq = int(sys.argv[2])
    else: save_freq = 1
    print('Data saving frequency in Hz: ', save_freq)
else:
    save_csv = False
    print('Will NOT save data to csv file!')


# Connect pixhawk
# master = mavutil.mavlink_connection('/dev/ttyTHS1', baud = 57600)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud = 57600)

# Connect xbee1 and declare gcs xbee address
xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
remote002 = RemoteDigiMeshDevice(xbee001,XBee64BitAddress.from_hex_string("0013A20040D8DCD5")) # 0013A20040F5C5DB
xbee001.open(force_settings=True)

# Get checksum
chks = mavutil.x25crc()

# Get the first heartbeat
msg = None
while not msg:
    print("waiting for the first Heartbeat ...")
    try: 
        # master.wait_heartbeat() # Wait for the first heartbeat 
        msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
    except: pass
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Initialize data stream (request_data_stream_send)
rate = 2 # desired transmission rate
msg = None
# Get the first data
while not msg:
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
    time.sleep(1)
    print("waiting for RAW_IMU ...")
    try: msg = master.recv_match(type=['RAW_IMU'], blocking=True, timeout=1)
    except: pass
print("RAW_IMU received")


# Initialize parameters and packets
sysID, compID, commID = master.target_system, master.target_component, 22
gps_time = c_int(0)
roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)  
lat, lon, alt, vx, vy, vz, hdg = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)     
fix, sat_num = c_int(0), c_int(0)
mode, arm, system_status, failsafe = c_int(255), c_int(255), c_int(255), c_int(255)
command, result = c_int(255), c_int(255)
Dyn_waypt_lat, Dyn_waypt_lon, Dyn_waypt_alt, waypt_id = c_int(0), c_int(0), c_int(0), c_int(255)
servo1, servo2, servo3, servo4 = c_int(0), c_int(0), c_int(0), c_int(0)

others_sysID, others_compID, others_commID = c_int(0), c_int(0), c_int(0)
others_lat, others_lon, others_alt = c_int(0), c_int(0), c_int(0)
others_vx, others_vy, others_vz, others_hdg, others_yaw, others_mode = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(255)
others_gps_time, others_sys_time = c_int(0), c_int(0)

pkt= {127: packet127(sysID, compID, commID, mode, arm, system_status, failsafe),
    128: packet128(sysID, compID, commID, lat, lon, alt, fix, sat_num, vx, vy, vz, hdg, roll, pitch, yaw, xacc, yacc, zacc, gps_time),
    129: packet129(sysID, compID, commID, command, result),
    130: packet130(sysID, others_sysID, others_commID, others_lat, others_lon, others_alt, others_vx, others_vy, others_vz, others_hdg, others_gps_time),
    131: packet131(),
    132: packet132(),
    133: packet133(),
    134: packet134(sysID, compID, commID, lat, lon, alt, vx, vy, vz, xacc, yacc, xgyro, ygyro, zgyro, hdg, yaw, mode, gps_time),
    135: packet135(),
    136: packet136(sysID, compID, commID, Dyn_waypt_lat, Dyn_waypt_lon, waypt_id),
    137: packet137(sysID, compID, commID, servo1, servo2, servo3, servo4),
    138: packet138(sysID, compID, commID)
}

# Get already loaded mission
msg = None
while not msg: # Get mission count
    master.waypoint_request_list_send()
    time.sleep(1)
    print('Waiting for mission count...')
    msg = master.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=1)
print('Preloaded mission count: ', msg.count)
count, seq = msg.count, 0
pkt[132].mission_init(count)
while (seq < count): # Get mission item
    master.waypoint_request_send(seq)
    msg = master.recv_match(type=['MISSION_ITEM'],blocking=True,timeout=1)
    if not msg:
        print('MISSION_ITEM is none ...')
        continue
    seq = msg.seq + 1
    print('Preloaded mission seq, command, x, y, z: ', msg.seq, msg.command, msg.x, msg.y, msg.z)
    pkt[132].mission_save_input(msg.seq, msg.command, int(msg.x*1e7), int(msg.y*1e7), int(msg.z))
print('Done downloading preloaded mission.')

# some parameter initialization
target_lat, target_lon = 0.0, 0.0
stop_lat, stop_lon, stop_alt = 0.0, 0.0, 0
guide_lat, guide_lon, guide_alt = [], [], []
last_mavguide_time = 0
last_sent_time, last_seq_sent_time, msgID_to_send = 0, 0, [] 
Mission_guided, Formation_start, Formation_stop = False, False, False
last_cmd_time, send_cmd, confirmation = 0, False, 0
missionseq2gcs, wayptseq2gcs = 99, 99
other_uavs = {}
ca_lat, ca_lon, ca_alt = 0.0, 0.0, 0
col_avoid, ca_enable = False, False # col_avoid: if ca is needed; ca_enable: cmd from packet/gcs 

while True:
    try:
        if mode.value != info.mode_map_s2n[master.flightmode]:
            mode.value = info.mode_map_s2n[master.flightmode]
            msgID_to_send.extend([127])
    except: # for some other less seem modes
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
    except SerialException: pass
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
        vx.value, vy.value, vz.value = msg.vx, msg.vy, msg.vz # hdg.value = msg.hdg
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
    elif msg_type == "STATUSTEXT": # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
        if msg.severity < 4 and (failsafe.value != msg.severity+14):
            failsafe.value = msg.severity+14
            msgID_to_send.extend([127])
    elif msg_type == "MISSION_CURRENT":
        # print(msg.seq)
        if (master.flightmode == 'AUTO') and (len(pkt[132].Mission_lat) > msg.seq):
            waypt_id.value = msg.seq
            Dyn_waypt_lat.value = pkt[132].Mission_lat[msg.seq] 
            Dyn_waypt_lon.value = pkt[132].Mission_lon[msg.seq]
            Dyn_waypt_alt.value = pkt[132].Mission_alt[msg.seq]
    elif msg_type == "COMMAND_ACK":
        print(msg.command, msg.result)
        command.value = msg.command # 16: NAV_WAYPOINT, 22: NAV_TAKEOFF, 176: DO_SET_MODE, 300: MISSION_START, 400: ARM_DISARM
        result.value = msg.result # https://mavlink.io/en/messages/common.html#MAV_RESULT
        if command.value == 176 or command.value == 400:
            send_cmd = False
        if (command.value == 16 or command.value == 22 or command.value == 300) and (result.value != 0):
            confirmation += 1
        else: confirmation = 0
        msgID_to_send.extend([129])
    elif msg_type == "SERVO_OUTPUT_RAW":
        servo1.value, servo2.value, servo3.value, servo4.value = msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw
        # print(msg)
    elif msg_type =='POSITION_TARGET_GLOBAL_INT':
        target_lat, target_lon = msg.lat_int, msg.lon_int
        Dyn_waypt_lat.value, Dyn_waypt_lon.value, Dyn_waypt_alt.value = int(msg.lat_int), int(msg.lon_int), int(msg.alt) #degE7, degE7, m 
        # print('POSITION_TARGET_GLOBAL_INT lat, lon, alt: ', msg.lat_int, msg.lon_int, msg.alt)


    # send out some pkts every 1 sec
    if (time.time() - last_sent_time) >= 1.0: 
        # update heading as the "desired" heading, not as yaw angle
        dx, dy, dz = pm.geodetic2enu(Dyn_waypt_lat.value/1e7, Dyn_waypt_lon.value/1e7, Dyn_waypt_alt.value, lat.value/1e7, lon.value/1e7, alt.value/1e3)
        hdg.value = int(math.atan2(dy, dx)*180/math.pi)
        msgID_to_send.extend([128, 134, 136, 137])
        last_sent_time = time.time()
        # delete overtimed neighbor - did not received its msg for more than 5 sec
        utctime = datetime.utcnow()
        cur_sys_time = int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))
        for n_id in other_uavs.copy():
            if (cur_sys_time - other_uavs[n_id].sys_time >= 5000):
                other_uavs.pop(n_id)



    if (time.time() - last_seq_sent_time) >= 1.0: # 0.3 sec may lost packet, 1 sec won't
        # sent out pixhawk mission info 
        if (missionseq2gcs < len(pkt[132].Mission_alt)):
            pkt[138].save_data(missionseq2gcs, pkt[132].Mission_modes[missionseq2gcs], pkt[132].Mission_lat[missionseq2gcs], pkt[132].Mission_lon[missionseq2gcs], pkt[132].Mission_alt[missionseq2gcs])
            missionseq2gcs += 1
            msgID_to_send.extend([138])
        elif (wayptseq2gcs < len(guide_lat)):
            pkt[138].save_data(wayptseq2gcs, 0, int(guide_lat[wayptseq2gcs]), int(guide_lon[wayptseq2gcs]), int(guide_alt[wayptseq2gcs]))
            wayptseq2gcs += 1
            msgID_to_send.extend([138])
        last_seq_sent_time = time.time()

    
    # Send packet
    if (len(list(msgID_to_send)) != 0):
        msgID_to_send = set(msgID_to_send) # remove duplicate pkt 
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
        if i == 134: # xbee broadcast v2v data
            try: xbee001.send_data_broadcast(pkt_bytearray)
            except: pass
        else: # v2g data (with ground xbee address)
            try: xbee001.send_data(remote002,pkt_bytearray)
            except: pass
        # print(i, pkt_bytearray)
    if (len(list(msgID_to_send)) != 0):
        print('MsgID_to_send: ', msgID_to_send)
        msgID_to_send = [] # reset pkt-to-send list

    
    # Read packet
    try:
        received = xbee001.read_data()
        data = received.data
        received_msgID = data[1]
        print('Received id: ', received_msgID)
        
        if received_msgID == 131:
            pkt[received_msgID].unpackpkt(data)
            pkt[132].mission_init(pkt[received_msgID].Waypt_count)
            missionseq2gcs = 99
        
        elif received_msgID == 132:
            pkt[received_msgID].unpackpkt(data)
            pkt[received_msgID].mission_save()
            print("!!!!", pkt[received_msgID].Mission_alt)

            # https://hamishwillee.gitbooks.io/ham_mavdevguide/content/en/services/mission.html
            if (999 not in pkt[received_msgID].Mission_alt): # if all mission waypts are received
                if pkt[131].Formation == 0:
                    wp = mavwp.MAVWPLoader()
                    print(data)
                    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                    for i in range(pkt[131].Waypt_count):
                        wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                            sysID, compID, i, frame,
                            pkt[received_msgID].Mission_modes[i], 0, 0, 0, pkt[received_msgID].Mission_accept_radius[i], 0, 0,
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
                        else:
                            print(msg)
                            print(wp.wp(msg.seq))
                            master.mav.send(wp.wp(msg.seq))
                            # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
                        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=0.1)
                        try: 
                            command.value, result.value= 999, msg.type # 999 is a self-defined numbmer
                            Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[0], pkt[132].Mission_lon[0]
                            print(result.value)
                        except: pass
                        if (time.time() - start_time > 30): # is time exceeds 30s, ask gcs to resend
                            command.value, result.value = 999, 99 # failed, please send again
                            break
                    msgID_to_send.extend([129]) # send out mission_ack packet
                else:
                    wayptseq2gcs = 999
                    x_list, y_list = [], []
                    for i in range(pkt[131].Waypt_count): # convert lla to enu
                        des_lat, des_lon, des_alt = pkt[132].Mission_lat[i], pkt[132].Mission_lon[i], pkt[132].Mission_alt[i]
                        x, y, z = pm.geodetic2enu(des_lat/1e7, des_lon/1e7, des_alt, lat.value/1e7, lon.value/1e7, alt.value/1e3)
                        x_list.append(x)
                        y_list.append(y)
                    # pre-calculate triangle/straight-line formation waypoints
                    wp_x, wp_y = plan.triangle_straight(pkt[131].Formation, pkt[131].LF, pkt[131].Desired_dist, pkt[131].Radius, pkt[131].Angle, pkt[131].Waypt_num, x_list, y_list)
                    guide_lat, guide_lon, guide_alt = [], [], []
                    for i in range(len(wp_x)): # convert enu to lla, and store them in guide_lat & guide_lon
                        guide_alt.append(int(pkt[132].Mission_alt[0])) # assume altitudes are the same, and store them in guide_alt
                        a, b, c = pm.enu2geodetic(wp_x[i], wp_y[i], guide_alt[-1], lat.value/1e7, lon.value/1e7, alt.value/1e3)  
                        guide_lat.append(int(a*1e7))
                        guide_lon.append(int(b*1e7))
                    print('Finish Formation Calculation!', guide_lat, guide_lon)

                    if save_csv:
                        utctime = datetime.utcnow()
                        data_list_wpt = [['time: '+str((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3)), 'Formation: '+str(pkt[131].Formation), 'LF: '+str(pkt[131].LF)]]
                        data_list_wpt.append(['seq', 'lat', 'lon', 'alt'])
                        for i in range(len(wp_x)):
                            data_list_wpt.append([i, guide_lat[i], guide_lon[i], guide_alt[i]])
                        write_csv(data_list_wpt)

                
        elif received_msgID == 133:
            print('CMD from 133 Packet: ', data[5])
            pkt[received_msgID].unpackpkt(data)
            Mission_guided, Formation_start, Formation_stop = False, False, False
            if (pkt[received_msgID].mode_arm <= 11): # set_mode, arm, disarm
                send_cmd = True
                last_cmd_time, last_cmd_send_time = time.time(), time.time()
            elif (pkt[received_msgID].mode_arm == 12): # takeoff
                takeoff_alt = 5
                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, confirmation, 0, 0, 0, 0, 0, 0, takeoff_alt)
            elif (pkt[received_msgID].mode_arm == 13): # mission start
                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_MISSION_START, confirmation, 0, 0, 0, 0, 0, 0, 0)
            elif (pkt[received_msgID].mode_arm == 14): # mission with guided mode
                waypt_id.value = 0
                if (len(pkt[132].Mission_alt)!=0) and (999 not in pkt[132].Mission_alt):
                    # Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value]
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                        pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value], 0, 0, 0, 0, 0, 0, 0, 0))
                    # master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, confirmation, 0, 0, 0, 0, 
                    #     pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value])
                    Mission_guided = True
            elif (pkt[received_msgID].mode_arm == 15): # get mission from pixhawk and sent them to gcs
                missionseq2gcs = 0
                mc_msg = None
                while not mc_msg: # Get mission count
                    master.waypoint_request_list_send()
                    time.sleep(1)
                    print('Waiting for mission count from Pixhawk...')
                    mc_msg = master.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=1)
                print('Mission count from Pixhawk: ', mc_msg.count)
                count, seq = mc_msg.count, 0
                pkt[132].mission_init(count)
                command.value, result.value = 998, count # send out the totoal number of mission item
                msgID_to_send.extend([129])
                start_time = time.time() 
                while (seq < count and time.time()-start_time<40.0): # Get mission item
                    master.waypoint_request_send(seq)
                    mi_msg = master.recv_match(type=['MISSION_ITEM'],blocking=True,timeout=1)
                    if not mi_msg:
                        print('MISSION_ITEM is none ...')
                        continue
                    seq = mi_msg.seq + 1
                    print('Mission seq, command, x, y, z from Pixhawk: ', mi_msg.seq, mi_msg.command, mi_msg.x, mi_msg.y, mi_msg.z)
                    pkt[132].mission_save_input(mi_msg.seq, mi_msg.command, int(mi_msg.x*1e7), int(mi_msg.y*1e7), int(mi_msg.z))
                print('Done downloading Pixhawk mission. Time: ', start_time)
            elif (pkt[received_msgID].mode_arm == 16): # set out guided waypts to gcs
                wayptseq2gcs = 0
                command.value, result.value = 997, len(guide_lat) # send out the totoal number of mission item
                if len(guide_lat) != len(guide_lon):
                    result.value = 99 # failed ...
                msgID_to_send.extend([129])
            elif (pkt[received_msgID].mode_arm == 17): # go_first: go to the first guided waypt
                waypt_id.value = 0
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                        guide_lat[waypt_id.value], guide_lon[waypt_id.value], guide_alt[waypt_id.value], 0, 0, 0, 0, 0, 0, 0, 0))
                print('Formation first point sent!')
            elif (pkt[received_msgID].mode_arm == 18): # Formation_start: start guided mission
                Formation_start = True
                print('Formation flight STARTED! Lat: ', guide_lat)
            elif (pkt[received_msgID].mode_arm == 19): # Formation stop: stop guided mission
                Formation_stop = True
                stop_lat, stop_lon, stop_alt = lat.value, lon.value, alt.value
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                        lat.value, lon.value, alt.value/1e3, 0, 0, 0, 0, 0, 0, 0, 0))
                print('Formation flight STOPPED!')
            elif (pkt[received_msgID].mode_arm == 20): # Eable collision avoidance capability
                ca_enable = True
                print('Enable CA!')
            elif (pkt[received_msgID].mode_arm == 21): # Disable collision avoidance capability
                ca_enable = False
                print('Disable CA!')

        
        elif received_msgID == 134: # received v2v
            others_sysID.value, others_compID.value, others_commID.value, others_lat.value, others_lon.value, others_alt.value, others_vx.value, others_vy.value, others_vz.value, others_hdg.value, others_yaw.value, others_mode.value, others_gps_time.value, others_sys_time.value = pkt[received_msgID].unpackpkt(data)
            if others_sysID.value not in other_uavs:
                other_uavs[others_sysID.value] = uav_info(others_sysID.value, others_compID.value, others_commID.value, others_lat.value, others_lon.value, others_alt.value, others_vx.value, others_vy.value, others_vz.value, others_hdg.value, others_mode.value, others_gps_time.value, others_sys_time.value)
            else:
                other_uavs[others_sysID.value].update(others_lat.value, others_lon.value, others_alt.value, others_vx.value, others_vy.value, others_vz.value, others_hdg.value, others_mode.value, others_gps_time.value, others_sys_time.value)
            dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value, others_lat.value/1e7, others_lon.value/1e7, others_alt.value)
            pkt[130].calculated((dx**2 + dy**2)**0.5, int(math.atan2(dy,dx)*180/math.pi))
            msgID_to_send.extend([130]) # send out v2g regarding neighboring info
            
            if save_csv and master.sysid_state[master.sysid].armed: 
                utctime = datetime.utcnow()
                data_step = [int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))]
                data_step.extend([others_sysID.value, (dx**2 + dy**2)**0.5, int(math.atan2(dy,dx)*180/math.pi)])
                data_list_n.append(data_step)
    except: pass

    # Continuously setting out set_mode/arm/disarm if the UAV did not react
    if send_cmd and (time.time() - last_cmd_time < 3.0) and (time.time()-last_cmd_send_time > 0.25):
        last_cmd_send_time = time.time()
        if (pkt[133].mode_arm < 10): # disarm
            if (pkt[133].mode_arm == 8): # convert position mode number
                master.set_mode(16)
            else: master.set_mode(pkt[133].mode_arm)
        elif (pkt[133].mode_arm == 10): # arm   
            master.arducopter_arm()
        elif (pkt[133].mode_arm == 11): # disarm
            master.arducopter_disarm()

    if (master.flightmode == 'GUIDED') and (time.time()-last_mavguide_time > 0.25):
        # Collision avoidance (CA) above all
        col_avoid_temp = False # to check for this iteration 
        if ca_enable and pkt[131].LF != 0: # only non-leader uav need to CA
            for n_id in other_uavs: # check the neighbors
                if n_id < sysID: # check if the neighbor has higher precedence (lower id number)
                    dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, 10, other_uavs[n_id].lat/1e7, other_uavs[n_id].lon/1e7, 10)
                    if (dx**2 + dy**2)**0.5 < pkt[131].Desired_dist:  
                        col_avoid_temp = True
                        if col_avoid != True: # remember the current location. If continous CA, use the location of the initial CA.
                            ca_lat, ca_lon, ca_alt = lat.value, lon.value, alt.value
                        break
        if not col_avoid_temp:
            col_avoid = False
        if col_avoid:
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                            ca_lat, ca_lon, ca_alt/1e3, 0, 0, 0, 0, 0, 0, 0, 0)) 
            # master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111000111), 
            #     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) 
            print('Collision Avoidance !!! UAVs: ', sysID, n_id)
        # for guided set global position: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        elif Mission_guided and (len(pkt[132].Mission_alt)!=0) and (999 not in pkt[132].Mission_alt):
            des_lat, des_lon, des_alt = pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value]
            if (des_lat != target_lat) or (des_lon != target_lon):
                print('Guided mission command sending out: ', des_lat, des_lon, des_alt)
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                        des_lat, des_lon, des_alt, 0, 0, 0, 0, 0, 0, 0, 0))
            dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value/1e3, des_lat/1e7, des_lon/1e7, des_alt)
            if (waypt_id.value < pkt[131].Waypt_count - 1) and (dx**2 + dy**2 + dz**2 <= 1.0**2):
                waypt_id.value += 1
        
        elif Formation_stop: # stop formation, uav stay at where it is when the stop command is ordered (v=0)
            if (stop_lat != target_lat) or (stop_lon != target_lon):
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                                stop_lat, stop_lon, stop_alt/1e3, 0, 0, 0, 0, 0, 0, 0, 0))

        elif Formation_start and len(guide_lat)!=0:
            self_fly = True # uav use the pre-calculated path (not folowing the leader)
            if (1 in other_uavs):
                if (other_uavs[1].mode == 4):
                    self_fly = False # if the leader is detected and it is in guided mode, the uav follows the leader
            
            if pkt[131].LF == 0: 
                # if this uav is the leader, just follow the pre-planned path
                des_lat, des_lon, des_alt = guide_lat[waypt_id.value], guide_lon[waypt_id.value], guide_alt[waypt_id.value]
                if (des_lat != target_lat) or (des_lon != target_lon):
                    print('Guided mission command sending out (Leader): ', des_lat, des_lon, des_alt)
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                            des_lat, des_lon, des_alt, 0, 0, 0, 0, 0, 0, 0, 0))
                dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value/1e3, des_lat/1e7, des_lon/1e7, des_alt)
                if (waypt_id.value < len(guide_lat) - 1) and (dx**2 + dy**2 + dz**2 <= 1.0**2):
                    waypt_id.value += 1

            elif self_fly:
                # leader undetected or not in guided mode
                # Follower follow the the pre-calculated path (fly without leader)
                if waypt_id.value == 0: # try to find out which waypt the uav shall start following
                    dis = 9999
                    for i in range(len(guide_lat)):
                        dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value/1e3, guide_lat[i]/1e7, guide_lon[i]/1e7, guide_alt[i])
                        if dis > (dx**2 + dy**2)**0.5:
                            waypt_id.value = i # go to the nearest waypt
                    if waypt_id.value < len(guide_lat) - 1: waypt_id.value += 1 # advance one waypt to prevent from uav going backward
                des_lat, des_lon, des_alt = guide_lat[waypt_id.value], guide_lon[waypt_id.value], guide_alt[waypt_id.value]
                if (des_lat != target_lat) or (des_lon != target_lon):
                    print('Guided mission command sending out (Follower self_fly): ', des_lat, des_lon, des_alt)
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                            des_lat, des_lon, des_alt, 0, 0, 0, 0, 0, 0, 0, 0))
                dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value/1e3, des_lat/1e7, des_lon/1e7, des_alt)
                if (waypt_id.value < len(guide_lat) - 1) and (dx**2 + dy**2 + dz**2 <= 1.0**2):
                    waypt_id.value += 1

            elif pkt[131].Formation == 1: # triangle
                # if this uav is a follower, plan its desired location based on the leader's location
                waypt_id.value = 0
                dx, dy, dz = pm.geodetic2enu(other_uavs[1].lat/1e7, other_uavs[1].lon/1e7, 10, lat.value/1e7, lon.value/1e7, 10)
                ang = pkt[131].Angle*math.pi/180
                if pkt[131].LF == 2: ang *= -1
                des_x = dx + pkt[131].Desired_dist*math.cos(math.pi+other_uavs[1].hdg+ang)
                des_y = dy + pkt[131].Desired_dist*math.sin(math.pi+other_uavs[1].hdg+ang)
                des_lat, des_lon, des_alt = pm.enu2geodetic(des_x, des_y, 10, lat.value/1e7, lon.value/1e7, 10)
                des_lat, des_lon, des_alt = int(des_lat*1e7), int(des_lon*1e7), int(des_alt)
                if (des_lat != target_lat) or (des_lon != target_lon):
                    print('Guided mission command sending out (Follower/Triangle): ', des_lat, des_lon, des_alt)
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                                des_lat, des_lon, guide_alt[0], 0, 0, 0, 0, 0, 0, 0, 0))

            elif pkt[131].Formation == 2: # straight line
                # if this uav is a follower, plan its desired location based on the leader's location
                waypt_id.value = 0
                dx, dy, dz = pm.geodetic2enu(other_uavs[1].lat/1e7, other_uavs[1].lon/1e7, 10, lat.value/1e7, lon.value/1e7, 10)
                des_x = dx + pkt[131].LF*pkt[131].Desired_dist*math.cos(math.pi+other_uavs[1].hdg)
                des_y = dy + pkt[131].LF*pkt[131].Desired_dist*math.sin(math.pi+other_uavs[1].hdg)
                des_lat, des_lon, des_alt = pm.enu2geodetic(des_x, des_y, 10, lat.value/1e7, lon.value/1e7, 10)
                des_lat, des_lon, des_alt = int(des_lat*1e7), int(des_lon*1e7), int(des_alt)
                if (des_lat != target_lat) or (des_lon != target_lon):
                    print('Guided mission command sending out (Follower/SL): ', des_lat, des_lon, des_alt)
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                                des_lat, des_lon, guide_alt[0], 0, 0, 0, 0, 0, 0, 0, 0))
        last_mavguide_time = time.time()
           
    # Save data to memory
    if save_csv and master.sysid_state[master.sysid].armed and (time.time() - last_save_time >= 1/save_freq):
        utctime = datetime.utcnow()
        data_step = [int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))]
        data_step.extend([mode.value, lat.value, lon.value, alt.value, Dyn_waypt_lat.value, Dyn_waypt_lon.value, Dyn_waypt_alt.value, vx.value, vy.value, vz.value, roll.value, pitch.value, yaw.value])
        data_list.append(data_step)
        last_save_time = time.time()

    # Write data to hardware and initialize data list memory
    if (not master.sysid_state[master.sysid].armed) and (len(data_list) != 1):
        write_csv(data_list)
        data_list = [['time', 'mode', 'lat', 'lon', 'alt', 'dlat', 'dlon', 'dalt', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']]
        write_csv(data_list_n)
        data_list_n = [['time', 'neighboring_id', 'relative_dis', 'relative_ang']]

# formation start not workign
# delect v2v info once 5 sec (if not received) (to prevent continuously collision avoidance)