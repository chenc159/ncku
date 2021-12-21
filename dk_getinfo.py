'''
This file gets the information of the drone by using dronekit.
The info to obtain:
UAV system id, UAV component ID, System_time, gps_time, gps_status, imu, 
uav system mav status, uav failsafe, lat, long, alt, vx, vy, vz, heading
'''

import time
from dronekit import connect

# Connect to the Vehicle
# vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = connect('/dev/ttyACM0', wait_ready=True, rate=4, baud=57600)

# for item in vehicle.parameters:
#     print(item)
# for items in vars(vehicle):
#     print(items)
# print(vehicle.master)




while True:

    # Time????
    # IDs (can't get comp.id)
    MAVLink_ID, GCS_ID = vehicle.parameters['SYSID_THISMAV'], vehicle.parameters['SYSID_MYGCS']
    # GPS status: Fix: 0-1: no fix, 2: 2D fix, 3: 3D fix; Number of satellites visible.
    gps_fix, gps_num = vehicle.gps_0.fix_type, vehicle.gps_0.satellites_visible
    # imu: roll, pitch, yaw
    imu = vehicle.attitude
    roll, pitch, yaw = imu.roll, imu.pitch, imu.yaw # in rad
    # lat, long, alt
    location = vehicle.location.global_frame
    lat, lon, alt = location.lat, location.lon, location.alt
    # velocity: vx, vy, vz
    vx, vy, vz = vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2]
    # heading
    heading = vehicle.heading # in deg
    # status https://dronekit-python.readthedocs.io/en/latest/automodule.html#dronekit.Vehicle.system_status
    MAV_state = vehicle.system_status.state

    # Difination for failsasfe??
    # is_armable == vehicle has booted, has a good GPS fix, and that the EKF pre-arm is complete.
    if (not vehicle.is_armable or gps_num < 8 or abs(roll) < 0.2 or abs(pitch) < 0.2): #vehicle.battery.level < 10
        failsafe = False
    else:
        failsafe = True
    
    print(gps_fix, gps_num)
    print(lat, lon, alt)
    # print(roll, pitch, yaw)


    time.sleep(1)


    # vehicle.capabilities.set_attitude_target_local_ned

    



vehicle.close()
