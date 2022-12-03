from dronekit import connect
import time

vehicle = connect('/dev/ttyUSB0', baud=57600)

time.sleep(10)

# vehicle is an instance of the Vehicle class
print("Autopilot Firmware version: " + str(vehicle.version))

print("Attitude pitch: " + str(vehicle.attitude.pitch))
print("Attitude yaw: " + str(vehicle.attitude.yaw))
print("Attitude roll: " + str(vehicle.attitude.roll))

print("Velocity x : " + str(vehicle.velocity[0]))
print("Velocity y : " + str(vehicle.velocity[1]))
print("Velocity z : " + str(vehicle.velocity[2]))

print("Heading: " + str(vehicle.heading))

#print("GPS: " + str(vehicle.gps_0))

print(float(vehicle.location.global_frame.lat))
print("Global Location lng : " + str(vehicle.location.global_frame.lon))
print("Global Location lng : " + str(vehicle.location.global_frame.alt))
print("Airspeed: " + str(vehicle.airspeed))

print("Groundspeed: " + str(vehicle.groundspeed))

print("Battery voltage: " + str(vehicle.battery.voltage))
print("Battery current: " + str(vehicle.battery.current))

print("Mode: " + str(vehicle.mode.name))    # settable
print("Armed: " + str(vehicle.armed))    # settable

print("Is Armable?: " + str(vehicle.is_armable))

print("System status: " + str(vehicle.system_status.state))

# print "Groundspeed: %s" % vehicle.groundspeed
# print "Airspeed: %s" % vehicle.airspeed
# print "Gimbal status: %s" % vehicle.gimbal
# print "Battery: %s" % vehicle.battery
# print "EKF OK?: %s" % vehicle.ekf_ok
# print "Last Heartbeat: %s" % vehicle.last_heartbeat
# print "Rangefinder: %s" % vehicle.rangefinder
# print "Rangefinder distance: %s" % vehicle.rangefinder.distance
# print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
# print "Heading: %s" % vehicle.heading
# print "Is Armable?: %s" % vehicle.is_armable
# print "System status: %s" % vehicle.system_status.state
# print "Mode: %s" % vehicle.mode.name    # settable
# print "Armed: %s" % vehicle.armed    # settable