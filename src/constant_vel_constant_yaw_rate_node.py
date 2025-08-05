#!/usr/bin/env python
import rospy
# from clean_desired_state import setpoint_publisher_time_dependent
from constant_vel_constant_yaw_rate import send_desired_vel
# from get_setpoints import setpoint_publisher
rospy.init_node("RPY_publishing_node",anonymous=False)
vel_publisher = send_desired_vel()

# waypoint_publisher = setpoint_publisher()
loop_rate = rospy.Rate(100)

while not rospy.is_shutdown():
    vel_publisher.spin()
    loop_rate.sleep()
