#!/usr/bin/env python
import rospy
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from geometry_msgs.msg import PoseStamped, Twist
from reef_msgs.msg import DesiredState

def trajectory_gen():
    rospy.init_node("trajectory_gen_node")
    control_mode = rospy.get_param("reef_teleop/control_mode")
    setpoint_raw_local_pub = rospy.Publisher("setpoint_raw/local", PositionTarget, queue_size=1)
    teleop_pub = rospy.Publisher("desired_state", DesiredState, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg=PositionTarget()
        if control_mode == "Position_trajectory":
            setpoint_raw_local_pub.Publish(msg)
        elif control_mode == "Velocity_trajectory":
            setpoint_raw_local_pub.publish(msg)


