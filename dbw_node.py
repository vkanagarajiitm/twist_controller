#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.linear_target = None
        self.angular_target = None
        self.linear_current = None

        # debug for veer issue
        self.last_required_vel_angular = 0.0
        self.count_required_vel_angular = 0

        # default to drive-by-wire not enabled - will pick this up from the topic...
        self.dbw_enabled = False
        self.sampling_rate = 50.0 # 50Hz

        self.controller = Controller(sampling_rate=self.sampling_rate,
                                     decel_limit=rospy.get_param('~decel_limit', -5),
                                     accel_limit=rospy.get_param('~accel_limit', 1.),
                                     brake_deadband=rospy.get_param('~brake_deadband', .1),
                                     vehicle_mass=rospy.get_param('~vehicle_mass', 1736.35),
                                     fuel_capacity=rospy.get_param('~fuel_capacity', 13.5),
                                     wheel_radius=rospy.get_param('~wheel_radius', 0.2413),
                                     wheel_base=rospy.get_param('~wheel_base', 2.8498),
                                     steer_ratio=rospy.get_param('~steer_ratio', 14.8),
                                     max_lat_accel=rospy.get_param('~max_lat_accel', 3.),
                                     max_steer_angle=rospy.get_param('~max_steer_angle', 8.))

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        #Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.callback_dbw_enabled)
        # /current_velocity topic gives the current speed of the vehicle
        rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)
        # /twist_cmd topic is the output from the vehicles waypoint controller
        # As implemented in waypoint_follower / pure_pursuit given code
        rospy.Subscriber('/twist_cmd', TwistStamped, self.callback_twist_cmd)

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.sampling_rate) # Was 50Hz
        while not rospy.is_shutdown():
            if self.linear_current is None or self.linear_target is None \
            or not self.dbw_enabled:
                continue

            throttle, brake, steer = self.controller.control(
                self.linear_target,
                self.angular_target,
                self.linear_current)

            self.publish(throttle, brake, steer)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        #rospy.loginfo('TwistController: Steering = ' + str(steer))
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def callback_dbw_enabled(self, msg):
        # check if drive-by-wire is enabled (i.e. the car is not in manual mode)
        self.dbw_enabled = msg.data
        rospy.loginfo('TwistController: dbw_enabled = ' + str(self.dbw_enabled))

        if not self.dbw_enabled:
            self.controller.reset()

    def callback_current_velocity(self, msg):
        # store the current velocity TwistStamped message
        self.linear_current = msg.twist.linear.x

    def callback_twist_cmd(self, msg):
        # store the received TwistStamped message from the waypoint follower node
        self.linear_target = msg.twist.linear.x
        self.angular_target = msg.twist.angular.z

        # debugging veering issue
        #if self.angular_target <> self.last_required_vel_angular:
       #     rospy.loginfo('Veer: received new angular velocity required : ' + str(self.angular_target) + ', count = ' + str(self.count_required_vel_angular))
        #    self.count_required_vel_angular = 0
       # self.last_required_vel_angular = self.angular_target
       # self.count_required_vel_angular += 1

if __name__ == '__main__':
    DBWNode()
