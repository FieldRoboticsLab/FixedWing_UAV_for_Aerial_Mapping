#!/usr/bin/env python

#####################################
#Created by TEAM IMU GROUND STATION #
#-----umutdumandag61@gmail.com------#
#####################################


import rospy
import mavros
from geometry_msgs.msg import Point, PoseStamped, Pose, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *
from mavros import command

#Author: Bugra Buyukarslan

class Modes:
    def __init__(self):
        pass

    def setArm(self):
        print("\n----------armingCall----------")
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def auto_set_mode(self):
        print("\n----------autoMode----------")
        rospy.wait_for_service('mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def wpPush(self,index,wps):
        print("\n----------pushingWaypoints----------")
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)		# start_index = the index at which we want the mission to start
            print "Waypoint Pushed"
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e
    
    def wpPull(self,wps):
        print("\n----------pullingWaypoints----------")
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print wpPullService().wp_received

            print "Waypoint Pulled"
        except rospy.ServiceException, e:
            print "Service Puling call failed: %s"%e


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        
    def stateCb(self, msg):
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame 
        self.wp.command = command
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue  
        self.wp.param1=param1 
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt 

        return self.wp


def main():

    rospy.init_node('waypointMission', anonymous=True)
    stateMt = stateMoniter()

    md = Modes()

    rate = rospy.Rate(20.0)
    md.setArm()
    rate.sleep()
    md.auto_set_mode()
    rate.sleep()

        
    wayp0 = wpMissionCnt()
        
    wayp1 = wpMissionCnt()
        
    wayp2 = wpMissionCnt()
        
    wayp3 = wpMissionCnt()
        
    wayp4 = wpMissionCnt()
        
    wayp5 = wpMissionCnt()
        
    wayp6 = wpMissionCnt()
        
    wayp7 = wpMissionCnt()
        
    wayp8 = wpMissionCnt()
        
    wayp9 = wpMissionCnt()
        
    wayp10 = wpMissionCnt()
        
    wayp11 = wpMissionCnt()
        
    wayp12 = wpMissionCnt()
        
    wayp13 = wpMissionCnt()
        
    wps = [] #List to story waypoints
        
    w = wayp0.setWaypoints(3,22,True,True,0.0,0.0,0.0,float('nan'),41.100528635451,28.551149368286136,20)
    wps.append(w)
        
    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.099962697501105,28.55046272277832,30)
    wps.append(w)
        
    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.099445264250875,28.551385402679447,30)
    wps.append(w)
        
    w = wayp3.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10133710977726,28.553445339202884,30)
    wps.append(w)
        
    w = wayp4.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.101773681775335,28.55279088020325,30)
    wps.append(w)
        
    w = wayp5.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10039927863525,28.550902605056766,30)
    wps.append(w)
        
    w = wayp6.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.09980100004828,28.55180382728577,30)
    wps.append(w)
        
    w = wayp7.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10133710977726,28.55350971221924,30)
    wps.append(w)
        
    w = wayp8.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10180602032639,28.552919626235965,30)
    wps.append(w)
        
    w = wayp9.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10044778747101,28.55102062225342,30)
    wps.append(w)
        
    w = wayp10.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.099817169811494,28.55209350585938,30)
    wps.append(w)
        
    w = wayp11.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10128860159835,28.55332732200623,30)
    wps.append(w)
        
    w = wayp12.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),41.10161198878109,28.552747964859012,15)
    wps.append(w)
        
    w = wayp13.setWaypoints(3,21,False,True,0.0,0.0,0.0,float('nan'),41.10066607678859,28.551417589187622,0)
    wps.append(w)
        



    print wps
    md.wpPush(0,wps)
    md.wpPull(0)

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


        