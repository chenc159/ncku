from dronekit import connect

# Connect to the Vehicle
vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)

# vehicle is an instance of the Vehicle class
print('Autopilot Firmware version: %s' % vehicle.version)
print('Autopilot capabilities (supports ftp): %s' % vehicle.capabilities.ftp)
print('Global Location: %s' % vehicle.location.global_frame)
print('Global Location (relative altitude): %s' % vehicle.location.global_relat) 
print('Local Location: %s' % vehicle.location.local_frame)    #NED
print('Attitude: %s' % vehicle.attitude)
print('Velocity: %s' % vehicle.velocity)
print('GPS: %s' % vehicle.gps_0)
print('Groundspeed: %s' % vehicle.groundspeed)
print('Airspeed: %s' % vehicle.airspeed)
print('Gimbal status: %s' % vehicle.gimbal)
print('Battery: %s' % vehicle.battery)
print('EKF OK?: %s' % vehicle.ekf_ok)
print('Last Heartbeat: %s' % vehicle.last_heartbeat)
print('Rangefinder: %s' % vehicle.rangefinder)
print('Rangefinder distance: %s' % vehicle.rangefinder.distance)
print('Rangefinder voltage: %s' % vehicle.rangefinder.voltage)
print('Heading: %s' % vehicle.heading)
print('Is Armable?: %s' % vehicle.is_armable)
print('System status: %s' % vehicle.system_status.state)
print('Mode: %s' % vehicle.mode.name)    # settable
print('Armed: %s' % vehicle.armed)    # settable

vehicle.close()

# Autopilot Firmware version: APM:Copter-3.6.8
# Autopilot capabilities (supports ftp): False
# Global Location: LocationGlobal:lat=0.0,lon=0.0,alt=0.15
# Global Location (relative altitude): LocationGlobalRelative:lat=0.0,lon=0.0,alt=0.153
# Local Location: LocationLocal:north=None,east=None,down=None
# Attitude: Attitude:pitch=-0.0381215363740921,yaw=2.1732099056243896,roll=-0.012934505939483643
# Velocity: [0.0, 0.0, 0.01]
# GPS: GPSInfo:fix=0,num_sat=0
# Groundspeed: 0.011965402401983738
# Airspeed: 0.0
# Gimbal status: Gimbal: pitch=None, roll=None, yaw=None
# Battery: Battery:voltage=0.0,current=None,level=None
# EKF OK?: False
# Last Heartbeat: 0.7889706349999415
# Rangefinder: Rangefinder: distance=None, voltage=None
# Rangefinder distance: None
# Rangefinder voltage: None
# Heading: 124
# Is Armable?: False
# System status: STANDBY
# Mode: STABILIZE
# Armed: False

