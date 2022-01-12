'''
This file gets the information of the drone by using pymavlink.
The info to obtain:
UAV system id, UAV component ID, System_time, gps_time, gps_status, imu, 
uav system mav status, uav failsafe, lat, long, alt, vx, vy, vz, heading
'''

from pymavlink import mavutil
import time
from datetime import datetime

def system_status(num):
    """ 
    COPIED FROM DRONEKIT __init__.py FILE
    System status (:py:class:`SystemStatus`).

    The status has a ``state`` property with one of the following values:

    * ``UNINIT``: Uninitialized system, state is unknown.
    * ``BOOT``: System is booting up.
    * ``CALIBRATING``: System is calibrating and not flight-ready.
    * ``STANDBY``: System is grounded and on standby. It can be launched any time.
    * ``ACTIVE``: System is active and might be already airborne. Motors are engaged.
    * ``CRITICAL``: System is in a non-normal flight mode. It can however still navigate.
    * ``EMERGENCY``: System is in a non-normal flight mode. It lost control over parts
    or over the whole airframe. It is in mayday and going down.
    * ``POWEROFF``: System just initialized its power-down sequence, will shut down now.
    """
    return {
        0: 'UNINIT',
        1: 'BOOT',
        2: 'CALIBRATING',
        3: 'STANDBY',
        4: 'ACTIVE',
        5: 'CRITICAL',
        6: 'EMERGENCY',
        7: 'POWEROFF',
        8: 'TERMINATION'
    }.get(num)



# Start a connection
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud = 57600)
# master.reboot_autopilot()
# Wait for the first heartbeat 
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 4 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
# master.param_fetch_all()
# For checksum calculation
cs = mavutil.x25crc()


# Initialize parameters for drone data
sysID, compID = master.target_system, master.target_component
start_time, start_uptime = master.start_time, master.uptime
# print(start_time, time.localtime(start_time), start_uptime)

SYS_time, sysgps_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot = 0, 0, 0, 0, 0   # in ms
roll, pitch, yaw = 0, 0, 0                                              # in deg 
fix, num, lat, lon, alt = 0, 0, 0, 0, 0                                 # in degE7 and mm
vx, vy, vz, heading = 0, 0, 0, 0                                        # in cm/s and cdeg
MAV_state, battery, failsafe = 0, 0, 99                                 # int, num in %, bool

msgs =  {   
    "SYSTEM_TIME":          {"time_boot_ms": 0}, 
    "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
    "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
    "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
    "HEARTBEAT":            {"system_status": 99},
    "BATTERY_STATUS":       {"battery_remaining": 0},
    "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
    "STATUSTEXT":           {"severity": 99}
} #AHRS2, AHRS3

while True:
    # Get data from pixhawk via pymavlink
    msg = master.recv_match(blocking=True)

    if msg == None:
        continue
    elif msg.get_type() == "SYSTEM_TIME":             # system boot time
        SYS_time, sysgps_time = msg.time_boot_ms, msg.time_unix_usec
    elif msg.get_type() == "ATTITUDE":              # imu: time, roll, pitch, yaw
        IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, round(msg.roll*57.2958), round(msg.pitch*57.2958), round(msg.yaw*57.2958) #msg.roll, msg.pitch, msg.yaw
    elif msg.get_type() == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
        GPS_time_usec, fix, num = msg.time_usec, msg.fix_type, msg.satellites_visible
    elif msg.get_type() == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat, msg.lon, msg.alt # originally in degE7 and mm
        vx, vy, vz, heading = msg.vx, msg.vy, msg.vz, msg.hdg # originally in cm/s and cdeg    
    elif msg.get_type() == "HEARTBEAT":             # MAV_STATE
        MAV_state = system_status(msg.system_status)
        # MAV_state = msg.system_status
    elif msg.get_type() == "BATTERY_STATUS":        # Battery status
        battery = msg.battery_remaining
    elif msg.get_type() == "HIGH_LATENCY2":
        if msg.HL_FAILURE_FLAG > 0: # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
            failsafe = False
    elif msg.get_type() == "STATUSTEXT":
        if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            failsafe = False

    print("\n", msg)
    print('sys, imu, gps, gpsacc: ', SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot)
    print('sysgps_time: ', datetime.utcfromtimestamp(sysgps_time/1e6)) # day, hour, minute, second, microsecond
    print('rpy: ', roll, pitch, yaw)
    print('gps: ', fix, num, lat, lon, alt)
    print('v/hdg: ', vx, vy, vz, heading)
    print('state, bat, fs: ', MAV_state, battery, failsafe)
    print('mode: ', master.flightmode)
    print('armed: ', master.sysid_state[master.sysid].armed)
    # print('armed: ', master.armed)
    # print(master.start_time, master.uptime)
    # print(time.localtime(master.start_time))
