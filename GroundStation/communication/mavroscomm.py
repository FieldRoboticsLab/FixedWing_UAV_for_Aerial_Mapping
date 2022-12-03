import rospy
import time
import datetime

from std_msgs.msg import String
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import *


import redis


class MavrosComm():
    def __init__(self):

        self.rate = rospy.Rate(5)
        self.redis_pub = redis.Redis()

        self.state = ''
        self.gps = ''
        self.battery = ''
        self.altitude = ''
        self.velocity = ''
        self.data = ''
        self.heading = ''
        self.wind_vel = ''
        self.circle_gps = ''

        rospy.Subscriber("/mavros/state", State, self.callback_mavros_state)
        rospy.Subscriber("/mavros/global_position/raw/fix",
                        NavSatFix, self.callback_mavros_gps)
        rospy.Subscriber("/mavros/local_position/velocity_body",
                        TwistStamped, self.callback_mavros_vel)
        rospy.Subscriber("/mavros/battery", BatteryState, self.callback_mavros_battery)
        rospy.Subscriber("/mavros/local_position/pose",
                        PoseStamped, self.callback_mavros_altitude)

        rospy.Subscriber("/mavros/imu/data", Imu, self.callback_mavros_data)

        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.callback_mavros_heading)

        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.callback_wind)



    def callback_circle_gps(self, msg):
        self.circle_gps = 'circle_lat: ' + str(msg.latitude) + ' circle_lon: ' + str(msg.longitude)
    
    def callback_wind(self, msg):
        self.wind_vel = 'wind_vel: ' + str(msg.airspeed)

    def callback_mavros_heading(self, msg):
        self.heading = 'heading: ' + str(msg.data)
    
    def callback_mavros_data(self, msg):
        self.data = 'roll: '+ str(msg.angular_velocity.x) + ' pitch: ' + str(msg.angular_velocity.y) + ' yaw: ' + str(msg.angular_velocity.z)

    def callback_mavros_state(self, msg):
        self.state = 'connected:' + str(msg.connected)[0] + ' armed:' + str(
        msg.armed)[0] + ' guided:' + str(msg.guided)[0] + ' mode:' + str(msg.mode) 
    
    def callback_mavros_gps(self, msg):
        self.gps = 'lat: ' + str(msg.latitude) + ' lon: ' + str(msg.longitude)
    
    def callback_mavros_vel(self, msg):
        if msg.twist.linear.x and msg.twist.linear.y and msg.twist.linear.z is not None:
            self.velocity = 'x: ' + str(round(msg.twist.linear.x, 2)) + ' y: ' + str(
		                    round(msg.twist.linear.y, 2)) + ' z: ' + str(round(msg.twist.linear.z, 2))
        else:
            self.velocity = 'x: ' + "0.0" + ' y: ' + "0.0" + ' z: ' + "0.0"
    
    def callback_mavros_battery(self, msg):
        if msg.percentage is not None:
            self.battery = "{0:.2f}".format(msg.percentage)
        else:
            self.battery = "0.0"
    
    def callback_mavros_altitude(self, msg):
        if msg.pose.position.z is not None:
            self.altitude = "{0:.2f}".format(msg.pose.position.z)
        else:
            self.altitude = "0.0"
    
    def send_status(self):        
        while not rospy.is_shutdown():
            msg_status = self.state + ';' + self.gps + ';' + self.velocity + ';' + self.altitude + ';' + self.battery + ';' + self.data + ';' + self.heading + ';' + self.wind_vel + ';' + self.circle_gps
            ts = time.time()
            time_curr= datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S')
            self.redis_pub.rpush('incoming_messages', time_curr +" " + msg_status + "\n")
            print('incoming_messages', time_curr +" " + msg_status + "\n")
            self.rate.sleep()
            time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('mavroscomm', anonymous=True)
    mavroscomm = MavrosComm()
    mavroscomm.send_status()
