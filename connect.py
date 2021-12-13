from dronekit import connect
from digi.xbee.devices import XBeeDevice
from pymavlink import mavutil
from dronekit.mavlink import MAVConnection
# Connect to the Vehicle

vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)

#out = MAVConnection('/dev/ttyUSB0',baud=57600)
#vehicle._handler.pipe(out)
#out.start()

#vehicle2 = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)

vehicle_pymav = mavutil.mavlink_connection("/dev/ttyTHS1",baud=57600)
vehicle_pymav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
# Request all data streams
request = vehicle_pymav.mav.request_data_stream_send(vehicle_pymav.target_system, vehicle_pymav.target_com$
# Get Vechicle Home location -will be 'None' until first set by autopilot
#while not vehicle.home_location:
cmd = vehicle.commands
cmd.download()
cmd.wait_ready()
    #if not vehicle.home_location:
       #print ("Waiting for home location.....")
# Zigbee to PC Zigbee XCTU broadcast
device = XBeeDevice('/dev/ttyUSB0', 57600)
device.open()
device.send_data_broadcast('heartbeat: %s' % vehicle_pymav )
#device.send_data_broadcast('hearbeat: %s' % hearbeat )
device.send_data_broadcast('Global Location (relative altitude): %s' % vehicle.location.global_relative_fr$
#device.send_data_broadcast('GLOBAL_POSITION_INT: %s' % vehicle.add_message_listener('GLOBAL_POSITION_INT'$
device.send_data_broadcast('Velocity: %s' % vehicle.velocity)


# device.close()
# vehicle is an instance of the Vehicle class
print('Autopilot Firmware version: %s' % vehicle.version)
print('Autopilot capabilities (supports ftp): %s' % vehicle.capabilities.ftp)
print('Global Location: %s' % vehicle.location.global_frame)
print('Global Location (relative altitude): %s' % vehicle.location.global_relative_frame)
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
print('Home Location: %s' %vehicle.home_location)

