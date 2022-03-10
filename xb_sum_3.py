from socket import timeout
import time, math
import sys
import os
import csv
from datetime import datetime
from struct import *
from ctypes import *
import threading
from serial.serialutil import SerialException
import pymap3d as pm
from pymavlink import mavutil, mavwp
from digi.xbee.devices import DigiMeshDevice,RemoteDigiMeshDevice,XBee64BitAddress
from info import info, packet127, packet128, packet129, packet130, packet131, packet132, packet133, packet134, packet135, packet136, packet137, packet138


# Write data to csv
def write_csv(data):
    utctime = datetime.utcnow() # let file name be the utc time when its write (-m-d-h-m-s:month,day,hour,minute,sec)
    file_time = str(utctime.month) + "m" + str(utctime.day) + "d" + str(utctime.hour) + "h" + str(utctime.minute) + "m" + str(utctime.second) + "s"
    address = os.path.dirname(os.path.realpath('__file__')) + '/result/' + file_time + ".csv"
    with open(address, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(data)
    print(address + " saved!!")

# To save data to csv, the first argument shall be 1
# Second argument shall give the data saving frequency (default 1 Hz)
data_list = [['time', 'mode', 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']]
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
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud = 57600)
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud = 57600)

# Connect xbee1 and declare gcs xbee address
xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
remote002 = RemoteDigiMeshDevice(xbee001,XBee64BitAddress.from_hex_string("0013A20040D8DCD5")) # 0013A20040F5C5DB
xbee001.open(force_settings=True)

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
Dyn_waypt_lat, Dyn_waypt_lon, waypt_id = c_int(0), c_int(0), c_int(255)
servo1, servo2, servo3, servo4 = c_int(0), c_int(0), c_int(0), c_int(0)

others_sysID, others_compID, others_commID = c_int(0), c_int(0), c_int(0)
others_lat, others_lon, others_alt = c_int(0), c_int(0), c_int(0)
others_vx, others_vy, others_vz, others_hdg, others_gps_time = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)

pkt= {127: packet127(sysID, compID, commID, mode, arm, system_status, failsafe),
    128: packet128(sysID, compID, commID, lat, lon, alt, fix, sat_num, vx, vy, vz, hdg, roll, pitch, yaw, xacc, yacc, zacc, gps_time),
    129: packet129(sysID, compID, commID, command, result),
    130: packet130(sysID, others_sysID, others_commID, others_lat, others_lon, others_alt, others_vx, others_vy, others_vz, others_hdg, others_gps_time),
    131: packet131(),
    132: packet132(),
    133: packet133(),
    134: packet134(sysID, compID, commID, lat, lon, alt, vx, vy, vz, xacc, yacc, xgyro, ygyro, zgyro, hdg, gps_time),
    135: packet135(),
    136: packet136(sysID, compID, commID, Dyn_waypt_lat, Dyn_waypt_lon, waypt_id),
    137: packet137(sysID, compID, commID, servo1, servo2, servo3, servo4),
    138: packet138(sysID, compID, commID)
}

last_sent_time, msgID_to_send = 0, [] 
mission_guided = False
last_cmd_time, send_cmd, confirmation = 0, False, 0
missionseq2gcs = 99
# time.sleep(5)

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

    # send out some pkts every 1 sec
    if (time.time() - last_sent_time) >= 1.0: 
        msgID_to_send.extend([128, 134, 136, 137])
        last_sent_time = time.time()

    # sent out pixhawk mission info 
    if (missionseq2gcs < len(pkt[132].Mission_alt)):
        pkt[138].save_data(missionseq2gcs, pkt[132].Mission_modes[missionseq2gcs], pkt[132].Mission_lat[missionseq2gcs], pkt[132].Mission_lon[missionseq2gcs], pkt[132].Mission_alt[missionseq2gcs])
        missionseq2gcs += 1
        msgID_to_send.extend([138])

    
    # Send packet
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
            print('total_count: ', pkt[received_msgID].Waypt_count)
            print('Broadcast: ', data[4])
            print('waypt_count: ', data[5])
            print('Desired_dist: ',unpack('i',data[6:10])[0])
            print('system131: ',unpack('i',data[10:14])[0])
            missionseq2gcs = 99
        
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
                        sysID, compID, i, frame,
                        pkt[received_msgID].Mission_modes[i], 0, 0, pkt[received_msgID].Mission_accept_radius[i], 0, 0, 0,
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
                
        elif received_msgID == 133:
            print(data[5], data)
            pkt[received_msgID].unpackpkt(data)
            current_alt, current_lon = lat.value, lon.value
            mission_guided = False
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
                    Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value]
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                        pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value], 0, 0, 0, 0, 0, 0, 0, 0))
                    # master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, confirmation, 0, 0, 0, 0, 
                    #     pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value])
                    mission_guided = True
            elif (pkt[received_msgID].mode_arm == 15): # get mission from pixhawk and sent them to gcs
                missionseq2gcs = 0
                mc_msg = None
                while not mc_msg: # Get mission count
                    master.waypoint_request_list_send()
                    time.sleep(1)
                    print('Waiting for mission count from Pixhawk...')
                    mc_msg = master.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=1)
                print('Mission count from Pixhawk: ', msg.count)
                count, seq = mc_msg.count, 0
                pkt[132].mission_init(count)
                while (seq < count): # Get mission item
                    master.waypoint_request_send(seq)
                    mi_msg = master.recv_match(type=['MISSION_ITEM'],blocking=True,timeout=1)
                    if not mi_msg:
                        print('MISSION_ITEM is none ...')
                        continue
                    seq = mi_msg.seq + 1
                    print('Mission seq, command, x, y, z from Pixhawk: ', mi_msg.seq, mi_msg.command, mi_msg.x, mi_msg.y, mi_msg.z)
                    pkt[132].mission_save_input(mi_msg.seq, mi_msg.command, int(mi_msg.x*1e7), int(mi_msg.y*1e7), int(mi_msg.z))
                print('Done downloading Pixhawk mission.')

        
        elif received_msgID == 134: # received v2v
            others_sysID.value, others_compID.value, others_commID.value, others_lat.value, others_lon.value, others_alt.value, others_vx.value, others_vy.value, others_vz.value, others_hdg.value, others_gps_time.value = pkt[received_msgID].unpackpkt(data)
            dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value, others_lat.value/1e7, others_lon.value/1e7, others_alt.value)
            pkt[130].calculated((dx**2 + dy**2)**0.5, hdg.value - others_hdg.value)
            msgID_to_send.extend([130]) # send out v2g regarding neighboring info
    except: pass

    # Continuously setting out set_mode/arm/disarm if the UAV did not react
    if send_cmd and (time.time() - last_cmd_time < 3.0) and (time.time() - last_cmd_send_time > 0.25):
        last_cmd_send_time = time.time()
        if (pkt[133].mode_arm < 10): # disarm
            if (pkt[133].mode_arm == 8): # convert position mode number
                master.set_mode(16)
            else: master.set_mode(pkt[133].mode_arm)
        elif (pkt[133].mode_arm == 10): # arm   
            master.arducopter_arm()
        elif (pkt[133].mode_arm == 11): # disarm
            master.arducopter_disarm()

    # for guided set global position: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    if (master.flightmode == 'GUIDED') and mission_guided and (len(pkt[132].Mission_alt)!=0) and (999 not in pkt[132].Mission_alt):
        dx, dy, dz = pm.geodetic2enu(lat.value/1e7, lon.value/1e7, alt.value, pkt[132].Mission_lat[waypt_id.value]/1e7, pkt[132].Mission_lon[waypt_id.value]/1e7, pkt[132].Mission_alt[waypt_id.value])
        if (waypt_id.value < pkt[131].Waypt_count - 1) and (dx**2 + dy**2 + dz**2 <= pkt[131].Desired_dist**2):
            waypt_id.value += 1
            Dyn_waypt_lat.value, Dyn_waypt_lon.value = pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value]
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, sysID, compID, 6, int(0b110111111000), 
                pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value], 0, 0, 0, 0, 0, 0, 0, 0))
            # master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 
            #     pkt[132].Mission_lat[waypt_id.value], pkt[132].Mission_lon[waypt_id.value], pkt[132].Mission_alt[waypt_id.value])
            
    # Save data to memory
    if save_csv and master.sysid_state[master.sysid].armed and (time.time() - last_save_time >= 1/save_freq):
        utctime = datetime.utcnow()
        data_step = [int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))]
        data_step.extend([mode.value, lat.value, lon.value, alt.value, vx.value, vy.value, vz.value, roll.value, pitch.value, yaw.value])
        data_list.append(data_step)
        last_save_time = time.time()

    # Write data to hardware and initialize data list memory
    if (not master.sysid_state[master.sysid].armed) and (len(data_list) != 1):
        write_csv(data_list)
        data_list = [['time', 'mode', 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']]