from pymavlink import mavutil
import time

def is_armable(self):
    """
    Code from dronekit
    Returns ``True`` if the vehicle is ready to arm, false otherwise (``Boolean``).

    This attribute wraps a number of pre-arm checks, ensuring that the vehicle has booted,
    has a good GPS fix, and that the EKF pre-arm is complete.
    """
    # check that mode is not INITIALSING
    # check that we have a GPS fix
    # check that EKF pre-arm is complete
    return self.mode != 'INITIALISING' and (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) and self._ekf_predposhorizabs


# Start a connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait for the first heartbeat 
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# Initialize data stream
rate = 4 # desired transmission rate
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)


while True:

    command = input("0 to skip, 1 to arm, 2 to disarm")
    if int(command) == 0:
        pass
    elif int(command) == 1:
        master.arducopter_arm()
        master.motors_armed_wait()
    elif int(command) == 2:
        master.arducopter_disarm()
        master.motors_disarmed_wait()
    else:
        master.set_mode(command)


# arm/disarm
# check mode for px4 or acm
# send waypoints
# 

mode_mapping_acm = {
    0 : 'STABILIZE',
    1 : 'ACRO',
    2 : 'ALT_HOLD',
    3 : 'AUTO',
    4 : 'GUIDED',
    5 : 'LOITER',
    6 : 'RTL',
    7 : 'CIRCLE',
    8 : 'POSITION',
    9 : 'LAND',
    10 : 'OF_LOITER',
    11 : 'DRIFT',
    13 : 'SPORT',
    14 : 'FLIP',
    15 : 'AUTOTUNE',
    16 : 'POSHOLD',
    17 : 'BRAKE',
    18 : 'THROW',
    19 : 'AVOID_ADSB',
    20 : 'GUIDED_NOGPS',
    21 : 'SMART_RTL',
    22 : 'FLOWHOLD',
    23 : 'FOLLOW',
    24 : 'ZIGZAG',
}
