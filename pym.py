'''
This file gets the information of the drone by using pymavlink.
The info to obtain:
UAV system id, UAV component ID, System_time, gps_time, gps_status, imu, 
uav system mav status, uav failsafe, lat, long, alt, vx, vy, vz, heading
'''

from pymavlink import mavutil
import time

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
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait for the first heartbeat 
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

sysID, compID = master.target_system, master.target_component
SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot = 0.0, 0.0, 0.0, 0.0   # in ms
roll, pitch, yaw = 0.0, 0.0, 0.0                                                # in rad
fix, num, lat, lon, alt = 0, 0, 0.0, 0.0, 0.0                                   # in degE7 and mm
vx, vy, vz, heading = 0.0, 0.0, 0.0, 0.0                                        # in m/s and deg
MAV_state, battery, failsafe = None, 0, True                                    # string, num in %, bool

while True:
    msg = master.recv_match(blocking=True)
    if msg.get_type() == "SYSTEM_TIME":             # system boot time
        SYS_time = msg.time_boot_ms
    elif msg.get_type() == "ATTITUDE":              # imu: time, roll, pitch, yaw
        IMU_time_boot, roll, pitch, yaw = msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw
    elif msg.get_type() == "GPS_RAW_INT":           # GPS status: time_usec/boot, fix, sat_num
        GPS_time_usec, fix, num = msg.time_usec, msg.fix_type, msg.satellites_visible
    elif msg.get_type() == "GLOBAL_POSITION_INT":   # Fused GPS and accelerometers: location, velocity, and heading
        GPSACC_time_boot, lat, lon, alt = msg.time_boot_ms, msg.lat, msg.lon, msg.alt # in degE7 and mm
        vx, vy, vz, heading = msg.vx/100.0, msg.vy/100.0, msg.vz/100.0, msg.hdg/100.0 # originally in cm/s and cdeg    
    elif msg.get_type() == "HEARTBEAT":             # MAV_STATE
        MAV_state = system_status(msg.system_status)
    elif msg.get_type() == "BATTERY_STATUS":        # Battery status
        battery = msg.battery_remaining
    elif msg.get_type() == "HIGH_LATENCY2":
        if msg.HL_FAILURE_FLAG > 0: # https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
            failsafe = False
    elif msg.get_type() == "STATUSTEXT":
        if msg.severity < 4: # https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            failsafe = False

    print("\n", msg)
    print(SYS_time, IMU_time_boot, GPS_time_usec, GPSACC_time_boot)
    print(roll, pitch, yaw)
    print(fix, num, lat, lon, alt)
    print(vx, vy, vz, heading)
    print(MAV_state, battery, failsafe)
 
