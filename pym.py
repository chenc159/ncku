'''
This file gets the information of the drone by using pymavlink.
'''

from pymavlink import mavutil, mavwp
import time
from datetime import datetime
from info import info
from serial.serialutil import SerialException


# Start a connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait for the first heartbeat 
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 2 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
# For checksum calculation
cs = mavutil.x25crc()

method = 1 # 1 or 3 to choose the methos to use
ctrl, last_ask_time = True, 0  # if to send/receive control command from key input

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
mode, arm = 0, 0
command, result = 0, 0

# get already loaded mission
master.waypoint_request_list_send()
msg = None
while not msg:
    print('Waiting for mission count...')
    msg = master.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=1)
print(msg.count)
count, seq = msg.count, 0
while (seq < count):
    master.waypoint_request_send(seq)
    msg = master.recv_match(type=['MISSION_ITEM'],blocking=True,timeout=1)
    if not msg:
        print('MISSION_ITEM is none ...')
        continue
    seq = msg.seq + 1
    print(msg.seq, msg.command, msg.x, msg.y, msg.z)




while True:

    if mode != master.flightmode:
        mode = master.flightmode
        print(mode)
    if arm != master.sysid_state[master.sysid].armed:
        arm = master.sysid_state[master.sysid].armed
        print(arm)
    # Get data from pixhawk via pymavlink
    msg = None
    try: 
        msg = master.recv_match(blocking=True)
        msg_type = msg.get_type()
    except SerialException: pass
    
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
            GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat, msg.lon, msg.relative_alt # originally in degE7 and mm
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
            print("MISSION_ACK: ", msg.type) # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
        elif msg_type == "COMMAND_ACK": # https://mavlink.io/en/messages/common.html#MAV_RESULT
            if (command != msg.command) or (result != msg.result):
                command, result = msg.command, msg.result
            print("COMMAND_ACK: ", command, result)
        elif msg_type == "SERVO_OUTPUT_RAW":
            # print(msg)
            pass

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

    if ctrl and (time.time() - last_ask_time > 3):
        input_command = input("0-9: set mode, 10: arm, 11: disarm, 12: mission, 13: takeoff, 14: mission start ...: ")
        try:
            if int(input_command) < 10:
                master.set_mode(int(input_command))
            elif int(input_command) == 10:
                master.arducopter_arm()
                # master.motors_armed_wait()
            elif int(input_command) == 11:
                master.arducopter_disarm()
                # master.motors_disarmed_wait()
            elif int(input_command) == 12:
                mission_num = input("Input mission number: ")
                wp = mavwp.MAVWPLoader()                                                    
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                for i in range(int(mission_num)):                  
                    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                        sysID, compID,
                        i,
                        frame,
                        info.mission_mode_mapping[2],
                        0, 0, 0, 0, 0, 0,
                        24+i*0.1, 121+i*0.1, 3))
                master.waypoint_clear_all_send()                                     
                master.waypoint_count_send(int(mission_num))
                ack = 99
                while ack == 99:
                    msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
                    print(msg)
                    master.mav.send(wp.wp(msg.seq))
                    # https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
                    msg = master.recv_match(type=['MISSION_ACK'],blocking=True,timeout=0.1)
                    try: ack = msg.type
                    except: pass
                print("mission result: ", ack) 

            elif int(input_command) == 13:
                takeoff_alt = 20
                # master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, sysID, compID, 3, int(0b110111111000), 
                #     24, 121, takeoff_alt, 0, 0, 0, 0, 0, 0, 0, 0))
                master.set_mode(4)
                msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
                print("takeoff: ", msg.command, msg.result)

                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, takeoff_alt)
                send_mav_command = True
                print('takeoff command sent!!')
            
            elif int(input_command) == 14:
                master.mav.command_long_send(sysID, compID, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
                send_mav_command = True
                print('mission start!')

        except: pass
        last_ask_time = time.time()
    
    # if (command == 22) and (result == 0) and send_mav_command:
    #     master.set_mode(4)
    #     send_mav_command = False
    #     print('set guided for takeoff')
    
    if (command == 300) and (result == 0) and send_mav_command:
        master.set_mode(3)
        send_mav_command = False
        print('set auto for mission start')    



