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

for key, value in vehicle.parameters.iteritems():
    print " Key:%s Value:%s" % (key,value)


while True:
    time.sleep(5)

    # GPS status
    gps_fix, gps_num = vehicle.gps_0.fix, vehicle.gps_0.num
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

    # Difination for failsasfe??
    # imu roll pitch angle, gps fix and number, battery percentage, armable, baro, system_status.state
    if not vehicle.is_armable or vehicle.system_status.state!="STANDBY" or
        gps_fix < 1 or gps_num < 8 or abs(roll) < 0.2 or abs(pitch) < 0.2: #vehicle.battery.level < 10
        failsafe = False
    else:
        failsafe = True



    



vehicle.close()
