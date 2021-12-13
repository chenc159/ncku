'''
This file gets the information of the drone.
The info to obtain:
UAV system id, UAV component ID, System_time, gps_time, gps_status, imu, 
uav system mav status, uav failsafe, lat, long, alt, vx, vy, vz, heading
'''

import time
from dronekit import connect

# Connect to the Vehicle
vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)

while True:
    time.sleep(5)

    # imu: roll, pitch, yaw
    imu = vehicle.attitude
    roll, pitch, yaw = imu.roll, imu.pitch, imu.yaw
    # lat, long, alt
    location = vehicle.location.global_relative_frame
    lat, lon, alt = location.lat, location.lon, location.alt
    # velocity: vx, vy, vz
    vx, vy, vz = vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2]
    # heading
    heading = vehicle.heading

    # Difination for failsasfe??
    # imu roll pitch angle, gps fix and number, battery percentage, armable, baro, system_status.state
    gps_fix = vehicle.gps_0.fix
    gps_num = vehicle.gps_0.num



vehicle.close()
