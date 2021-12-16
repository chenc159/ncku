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
# IDs
sysID, compID = master.target_system, master.target_component
while True:
    # msg = master.recv_match(blocking=True)
    # print(msg)
    
    # system boot time
    SYS_time = master.recv_match(type="SYSTEM_TIME", blocking=True).time_boot_ms
    # imu: time, roll, pitch, yaw
    imu = master.recv_match(type="ATTITUDE", blocking=True)
    IMU_time_boot, roll, pitch, yaw = imu.time_boot_ms, imu.roll, imu.pitch, imu.yaw
    # GPS: time_usec/boot, fix, sat_num, lat, lon, alt
    GPS = master.recv_match(type="GPS_RAW_INT", blocking=True)
    GPS_time_usec, fix, num, lat, lon, alt = GPS.time_usec, GPS.fix_type, GPS.satellites_visible, GPS.lat, GPS.lon, GPS.alt # in degE7 and mm
    GPS_ACC = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True) # fused GPS and accelerometers. Shall use this, but it delays quite bad!
    GPSACC_time_boot, GPSACC_lat, GPSACC_lon, GPSACC_alt = GPS_ACC.time_boot_ms, GPS_ACC.lat, GPS_ACC.lon, GPS_ACC.alt # in degE7 and mm
    # velocity and heading
    GPSACC_vx, GPSACC_vy, GPSACC_vz, GPSACC_hdg = GPS_ACC.vx, GPS_ACC.vy, GPS_ACC.vz, GPS_ACC.hdg # in cm/s and cdeg    
    heading = master.recv_match(type="VFR_HUD", blocking=True).heading # in deg
    # MAV_STATE
    MAV_state = system_status(master.recv_match(type="HEARTBEAT", blocking=True).system_status)
    # Failsafe
    # failsafe = True
    # failsafe = master.recv_match(type="HIGH_LATENCY2", blocking=True).HL_FAILURE_FLAG
    # failsafe = master.recv_match(type="STATUSTEXT", blocking=True).MAV_SEVERITY

    # if not  and not master.recv_match(type="STATUSTEXT", blocking=True):
    #     failsafe = True
    # else:
    #     failsafe = False



    # battery = master.recv_match(type="BATTERY_STATUS", blocking=True).battery_remaining

    print(SYS_time)
    # print('imu, ', IMU_time_boot, roll, pitch, yaw)
    # print('gps, ', GPS_time_usec, fix, num, lat, lon, alt)
    # print('heading, ', heading)
    # print(master.recv_match(type="HIGH_LATENCY2", blocking=True)) #failure_flags == HL_FAILURE_FLAG
    # print(master.recv_match(type="STATUSTEXT", blocking=True)) #MAV_SEVERITY
    # print(master.recv_match(type="HEARTBEAT", blocking=True))
    # print(failsafe)

    # try:
    #     failsafe = master.recv_match(type="HIGH_LATENCY2", blocking=True).HL_FAILURE_FLAG
    #     failsafe = master.recv_match(type="STATUSTEXT", blocking=True).MAV_SEVERITY
    # except:
    #     pass
    
    

