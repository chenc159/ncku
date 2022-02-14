'''
This file gets the information of the drone by using pymavlink.
'''

from socket import timeout
from pymavlink import mavutil, mavwp
import time
from datetime import datetime
from info import info


# Start a connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait for the first heartbeat 
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 4 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
# For checksum calculation
cs = mavutil.x25crc()

method = 1 # 1 or 3 to choose the methos to use
ctrl, t = True, 0  # if to send/receive control command from key input

convert = info.convert
msgs =  info.msgs

# Initialize parameters for drone data
sysID, compID = master.target_system, master.target_component
start_time, start_uptime = master.start_time, master.uptime
SYS_time, sysgps_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot = 0, 0, 0, 0, 0   # in ms
roll, pitch, yaw = 0, 0, 0                                              # in deg 
fix, num, lat, lon, alt = 0, 0, 0, 0, 0                                 # in degE7 and mm
vx, vy, vz, heading = 0, 0, 0, 0                                        # in cm/s and cdeg
MAV_state, battery, failsafe = 0, 0, 99                                 # int, num in %, bool


while True:
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)
    msg_type = msg.get_type()
    if (method == 1):  # A simple method
        if msg == None:
            continue
        elif msg_type == "SYSTEM_TIME":             # system boot time
            SYS_time, sysgps_time = msg.time_boot_ms, msg.time_unix_usec
        elif msg_type == "ATTITUDE":              # imu: time, roll, pitch, yaw
            IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, round(msg.roll*57.2958), round(msg.pitch*57.2958), round(msg.yaw*57.2958) #msg.roll, msg.pitch, msg.yaw
        elif msg_type == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
            GPS_time_usec, fix, num = msg.time_usec, msg.fix_type, msg.satellites_visible
        elif msg_type == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
            GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat, msg.lon, msg.alt # originally in degE7 and mm
            vx, vy, vz, heading = msg.vx, msg.vy, msg.vz, msg.hdg # originally in cm/s and cdeg    
        elif msg_type == "HEARTBEAT":             # MAV_STATE
            MAV_state = msg.system_status
        elif msg_type == "BATTERY_STATUS":        # Battery status
            battery = msg.battery_remaining
        elif msg_type == "HIGH_LATENCY2":
            if msg.HL_FAILURE_FLAG > 0: # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
                failsafe = False
        elif msg_type == "STATUSTEXT":
            if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
                failsafe = False
        elif msg_type == "MISSION_CURRENT":
            current_mission_seq = msg.seq
        elif msg_type == "MISSION_ACK":
            print(msg.type)
        # print("\n", msg)
        # print('sys, imu, gps, gpsacc: ', SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot)
        # print('sysgps_time: ', datetime.utcfromtimestamp(sysgps_time/1e6)) # day, hour, minute, second, microsecond
        # print('rpy: ', roll, pitch, yaw)
        # print('gps: ', fix, num, lat, lon, alt)
        # print('v/hdg: ', vx, vy, vz, heading)
        # print('state, bat, fs: ', info.system_status(MAV_state), battery, failsafe)
        # print('mode: ', master.flightmode)
        # print('armed: ', master.sysid_state[master.sysid].armed)
        # print(master.start_time, master.uptime)
        # print(time.localtime(master.start_time))
    elif (method == 2): # A more advanced method
        if msg_type not in msgs.keys():
            continue
        # Store messages
        for item in msgs[msg_type].keys():
            name = msg_type + '.' + item        
            msgs[msg_type][item] = round(getattr(msg, item)*convert.get(name, 1))
        print('\n')
        print('mode: ', master.flightmode)
        print('armed: ', master.sysid_state[master.sysid].armed)
        print(msgs)

    t += 1
    if ctrl and (t%100 == 0):
        command = input("0 to 9 to set mode, 10 to arm, 11 to disarm, 12 to do sth with mission: ")
        try:
            if int(command) < 10:
                master.set_mode(int(command))
            elif int(command) == 10:
                master.arducopter_arm()
                # master.motors_armed_wait()
            elif int(command) == 11:
                master.arducopter_disarm()
                # master.motors_disarmed_wait()
            elif int(command) == 12:
                mission_num = input("Input mission number: ")
                wp = mavwp.MAVWPLoader()                                                    
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                for i in range(int(mission_num)):                  
                    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                        sysID, compID,
                        i,
                        frame,
                        info.mission_mode_mapping[0],
                        0, 0, 0, 0, 0, 0,
                        24+i*0.1, 121+i*0.1, 3))
                master.waypoint_clear_all_send()                                     
                master.waypoint_count_send(int(mission_num))
                # ack = 99
                # while ack==99:
                #     msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
                #     print(msg)
                #     master.mav.send(wp.wp(msg.seq))
                #     msg = master.recv_match(type=['MISSION_ACK'],blocking=True,timeout=0.1)
                #     try: ack = msg.type
                #     except: pass
                # print(ack)

                for i in range(int(mission_num)):
                    msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
                    print(msg)
                    master.mav.send(wp.wp(msg.seq))
                    print(wp.wp(msg.seq))
                
                # c = input('waiting...')
                # mission_ack = msg.type # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
                # print("mission result: ", mission_ack) 
        except: pass

