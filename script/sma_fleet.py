#!/usr/bin/env python2
import rospy
import numpy as np
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as q2e

class SMA():

    def __init__(self, name):
        self.name = name
        self.max_lin = 0.22
        self.max_ang = 2.84
        self.pose = Pose()
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(1)
        self.pub = rospy.Publisher(self.name + "/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(self.name + "/odom", Odometry, self.track)

    def track(self, msg):
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = q2e([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw
    
    
    def move(self, msg):
        self.cmd_vel = msg
        self.pub.publish(self.cmd_vel)
        #self.status()
    
    
    def status(self):
        print(self.name + ':')
   
    
    def run(self):
        rospy.spin()


class Fleet():

    def __init__(self, leader, followers=[]):
        rospy.init_node("Fleet", anonymous=False)
        self.Fleet = {}
        self.leader = SMA(leader)
        self.addFollowers(followers)
        self.machinist = rospy.Rate(10)
        self.min_radius = 0.5
        rospy.Subscriber("/cmd_vel", Twist, self.teleop)
    
    
    def teleop(self, msg):
        self.leader.move(msg)

    
    def addFollowers(self, robots):
        new_bots = list(set(robots) - set(self.Fleet.keys()))
        for robot in new_bots:
            print('adding turtlebot \"' + robot + '\"')
            self.Fleet[robot] = SMA(robot)
    
    
    def relPos(self, front_turtlebot, back_turtlebot):
        ref = front_turtlebot.pose
        tb_pose = back_turtlebot.pose
        module = np.sqrt((ref.x - tb_pose.x)**2 + (ref.y - tb_pose.y)**2)
        angle = np.arctan2(ref.y - tb_pose.y,  ref.x - tb_pose.x) - tb_pose.theta
        return (module, angle)

    
    def control(self, vector, kp=(1.5, 6)):
        u = Twist()
        flag = (vector[0] > self.min_radius)
        
        u.linear.x  = kp[0] * vector[0] * flag
        u.linear.y = 0
        u.linear.z = 0
        
        u.angular.z = kp[1] * vector[1] * flag
        u.angular.x = 0
        u.angular.y = 0
        return u
    

    def sat(self, u, turtlebot):

        if abs(u.linear.x) > turtlebot.max_lin:
            u.linear.x = turtlebot.max_lin * np.sign(u.linear.x)

        if abs(u.angular.z) > turtlebot.max_ang:
            u.angular.z = turtlebot.max_ang * np.sign(u.angular.z)
        return u


    def run(self):
        while not rospy.is_shutdown():
            front_turtlebot = self.leader

            for robot in self.Fleet:
                

                turtlebot = self.Fleet[robot]
                (dist, ang) = self.relPos(front_turtlebot, turtlebot)

                cmd = self.control((dist, ang))
                cmd = self.sat(cmd, turtlebot)
                turtlebot.move(cmd)

                front_turtlebot = turtlebot
            
            self.machinist.sleep()

if __name__ == "__main__":
    try:
        tb3_fleet = Fleet(leader='tb3_0')
        tb3_fleet.addFollowers(robots=['tb3_1', 'tb3_2'])
        tb3_fleet.run()
    except rospy.ROSInterruptException:
        pass
