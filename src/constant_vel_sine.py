#!/usr/bin/env python
import rospy
import reef_msgs
import math
import csv
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist,TransformStamped
from std_msgs.msg import Float32MultiArray
from reef_msgs.msg import DesiredState
from mavros_msgs.msg import PositionTarget, RCIn, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class send_desired_vel:
    def __init__(self):

        # Publishers
        self._RPY_pub = rospy.Publisher("RPY",Float32MultiArray, queue_size=1)
        self._error_2d_pub = rospy.Publisher("error_2D",Float32MultiArray, queue_size=1)
        self._error_3d_pub = rospy.Publisher("error_3D",Float32MultiArray, queue_size=1)
        self._error_r_pub = rospy.Publisher("error_r",Float32MultiArray, queue_size=1)
        self._target_pub = rospy.Publisher("target",Float32MultiArray, queue_size=1)
        self._Vout_2d_pub = rospy.Publisher("Vout_2d",Float32MultiArray, queue_size=1)
        self._desired_state = rospy.Publisher("desired_state", DesiredState,queue_size=1)
        self._setpoint_raw_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        
        # self.dt=.01
        # rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        
        # getting things from a yaml file
        self.waypoint_list = rospy.get_param("~waypoint_list")
        self.go2points=self.get_list_of_list_from_csv(self.waypoint_list)
        self.test_var = rospy.get_param("~test_var")

        # initial set up variables
        self.R=0
        self.Rd=0
        self.P=0
        self.Pd=0
        self.Y=0
        self.Yd=0
        self.I=0
        self.tx=float(self.go2points[self.I][0])
        self.ty=float(self.go2points[self.I][1])
        self.tz=float(self.go2points[self.I][2])
        self.tyawD=float(self.go2points[self.I][3])
        self.eyawD=0
        self.eyawR=0
        self.ex=0
        self.ey=0
        self.ez=0
        self.en=np.array([0,0,0])
        self.en_2d=np.array([0,0])
        self.V=1 #m/s
        self.Vout=np.array([0,0,0])
        self.Vout_2d=np.array([0,0])
        self.kp_yaw=.1 #rads/s
        self.yaw_rate=0
        self.er=100
        self.thresh_hold=.25
        self.thresh_hold_yaw=.1
        self.flag=0
        self.active = False
        self.TRAVELING = 0
        self.AT_WALL = 0
        self.error_theta=0
        self.L=5.5

        # Subscriber
        self._mocap_sub = rospy.Subscriber("/mavros/vision_pose/pose",PoseStamped,self.mocap_cb)
        self._RCIn_sub = rospy.Subscriber("/mavros/rc/in",RCIn,self.RCIn_cb)
        self._state_sub = rospy.Subscriber("/mavros/state",State, self.State_cb)

    def State_cb(self,State):
        active_param=(State.mode=="OFFBOARD")
        if not self.active and active_param:
            self.active = True
            print("im active")
        elif self.active and not active_param:
            self.active = False
            self.I=0
            print("im inactive")

        

    def RCIn_cb(self,RCIn_state):
        self.state=RCIn_state.channels[4]
        if self.state >= 1900:
            self.flag=1
        else:
            self.flag=0
            
        

    def mocap_cb(self,mocap_state):
        
        #Getting the euler angels
        self.R,self.P,self.Y=self.euler_from_quaternion(mocap_state.pose.orientation.x,mocap_state.pose.orientation.y,mocap_state.pose.orientation.z,mocap_state.pose.orientation.w)
        self.Rd=np.rad2deg(self.R)
        self.Pd=np.rad2deg(self.P)
        self.Yd=np.rad2deg(self.Y)
        # print(self.Y,self.Yd)
        #error from target to current
        self.ex=float(self.tx-mocap_state.pose.position.x)
        self.ey=float(self.ty-mocap_state.pose.position.y)
        self.ez=float(self.tz-mocap_state.pose.position.z)
        self.eyawD=float(self.tyawD-self.Yd)
        self.tyawR=np.deg2rad(self.tyawD)
        self.eyawR=float(self.tyawR-self.Y)
        
        #norm of the error 3d and 3d
        self.en = np.array([self.ex,self.ey,self.ez]) / np.linalg.norm(np.array([self.ex,self.ey,self.ez]))
        self.en_2d = np.array([self.ex,self.ey]) / np.linalg.norm(np.array([self.ex,self.ey]))

        #Unyawing the error
        Cyaw_2d=self.get_Cyaw_2d(self.Y)
        # ebln_2d=np.matmul(Cyaw_2d,self.en_2d)
        # self.Vout_2d=self.V*self.en_2d
        # print(self.Vout_2d)

        #Yaw rate
        self.yaw_rate=self.kp_yaw*self.eyawR
        # Print(RC call back)
        # print(self.flag)
        if self.active:
            # # Update point
            # self.er=np.sqrt(self.ex**2+self.ey**2+self.ez**2)
            # if  mocap_state.pose.position.x>=3.55 or mocap_state.pose.position.x <= -3.55 or mocap_state.pose.position.y>=1.1 or mocap_state.pose.position.y<=-1.7 or (self.er <= self.thresh_hold and self.eyawR <= np.abs(self.thresh_hold_yaw)):
            #     print("hit target or wall")
            #     self.AT_WALL = 1
            #     if  self.TRAVELING==1 :
            #         self.I=self.I+1
            #         if self.I >= len(self.go2points):
            #             self.tx=float(self.go2points[-1][0])
            #             self.ty=float(self.go2points[-1][1])
            #             self.tz=float(self.go2points[-1][2])
            #             self.tyaw=float(self.go2points[-1][3])
            #             self.Vout_2d[0]=0.0
            #             self.Vout_2d[1]=0.0
            #             self.yaw_rate=0.0
            #             print("all done")
            #         else:
            #             self.tx=float(self.go2points[self.I][0])
            #             self.ty=float(self.go2points[self.I][1])
            #             self.tz=float(self.go2points[self.I][2])
            #             self.tyaw=float(self.go2points[self.I][3])
            #             self.error_theta=np.arctan2(self.ty-mocap_state.pose.position.y,self.tx-mocap_state.pose.position.x)
            #             print(self.error_theta)
                        
            #         self.TRAVELING = 0
            # else:
            #     self.AT_WALL = 0 
            #     self.TRAVELING = 1
            # print("I:",self.I)
            # print("tx,ty:",self.tx,self.ty)
            # self.Vout_2d=self.V*self.en_2d
            # # if self.I == 0:
            # #     self.Vout_2d=self.V*self.en_2d
            # # else:
            # #     seconds = rospy.get_time()
            # #     Vtotal=np.matrix([[self.V*np.sin(seconds)],[np.abs(self.V*np.cos(seconds))]])
            # #     theta = self.error_theta
            # #     c, s = np.cos(theta), np.sin(theta)
            # #     R = np.array(((c, -s), (s, c)))
            # #     O=np.array(R*Vtotal)
            # #     XR=O[0][0]
            # #     YR=O[1][0]
            # #     self.Vout_2d=[XR,YR]
            # #     print("Vx,Vy: ",XR,YR)

        # else:
            switch_2_trav=False
            self.er=np.sqrt(self.ex**2+self.ey**2+self.ez**2)
            if self.I % 2==0:
                outside=mocap_state.pose.position.x>=3.55
            else:
                outside=mocap_state.pose.position.x<=-3.55
            # outside=mocap_state.pose.position.x>=3.55 or mocap_state.pose.position.x <= -3.55 or mocap_state.pose.position.y>=1.1 or mocap_state.pose.position.y<=-1.7 
            if self.TRAVELING == 0 and outside:
                self.TRAVELING=1
                switch_2_trav=True
                print("swithed to traveling")
            elif self.TRAVELING == 1 and not outside:
                self.TRAVELING=0
                print("switched to not traveling")

            if switch_2_trav or (self.er <= self.thresh_hold and self.eyawR <= np.abs(self.thresh_hold_yaw)):
                if self.I < len(self.go2points)-1:
                    #Increment waypoint
                    self.I=self.I+1
                    self.tx=float(self.go2points[self.I][0])
                    self.ty=float(self.go2points[self.I][1])
                    self.tz=float(self.go2points[self.I][2])
                    self.tyaw=float(self.go2points[self.I][3])
                    self.error_theta=np.arctan2(self.ty-mocap_state.pose.position.y,self.tx-mocap_state.pose.position.x)
                    self.L=self.L-.5
                    print("I: ",self.I)
                    print("error_theta: ",self.error_theta)
                    print("L: ",self.L)
                elif self.I < len(self.go2points):
                    #We're done - load final waypoint
                    self.I=self.I+1
                    self.tx=float(self.go2points[-1][0])
                    self.ty=float(self.go2points[-1][1])
                    self.tz=float(self.go2points[-1][2])
                    self.tyaw=float(self.go2points[-1][3])
                    # self.Vout_2d[0]=0.0
                    # self.Vout_2d[1]=0.0
                    # self.yaw_rate=0.0
                    print("all done")

            # print("I:",self.I)
                # print("tx,ty:",self.tx,self.ty)
            # print("error_theta: ",self.error_theta)

                # print("I:",self.I)
                # print("tx,ty:",self.tx,self.ty)
            # self.Vout_2d=self.V*self.en_2d
            if self.I == 0:
                self.Vout_2d=self.V*self.en_2d
            else:
                # print("im in")
                # self.L=self.L-.5
                seconds = rospy.get_time()
                Vtotal=np.matrix([[np.abs(self.V*np.cos((self.L)*seconds))],[self.V*np.sin((self.L)*seconds)]])
                theta = self.error_theta
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))
                O=np.array(R*Vtotal)
                XR=O[0][0]
                YR=O[1][0]
                self.Vout_2d=[XR,YR]
            # print("L:",self.L)
                # print("Vx,Vy: ",XR,YR)
                
            





            
            

    def spin(self):
        # publish Roll Pitch Yaw
        RPY_msg=Float32MultiArray()
        RPY_msg.data=[self.Rd,self.Pd,self.Yd]
        self._RPY_pub.publish(RPY_msg)
        # publish error 2d (ex,ey)
        error_2d_msg=Float32MultiArray()
        error_2d_msg.data=[self.ex,self.ey]
        self._error_2d_pub.publish(error_2d_msg)
        # publish error 3d (ex,ey,ez)
        error_3d_msg=Float32MultiArray()
        error_3d_msg.data=[self.ex,self.ey,self.ez]
        self._error_3d_pub.publish(error_3d_msg)
        # publish Vout 2d (Vx,Vy)
        Vout_2d_msg=Float32MultiArray()
        Vout_2d_msg.data=self.Vout_2d
        self._Vout_2d_pub.publish(Vout_2d_msg)
        # Publish Desired state
        desired_state_msg=DesiredState()
        # Velocity states
        desired_state_msg.velocity_valid = True
        desired_state_msg.velocity.x = self.Vout_2d[0]
        desired_state_msg.velocity.y = self.Vout_2d[1]
        desired_state_msg.velocity.yaw = self.yaw_rate
        # Position states
        desired_state_msg.pose.z = self.tz
        self._desired_state.publish(desired_state_msg)
        # Publish pose error yaw error
        er_msg=Float32MultiArray()
        er_msg.data=[self.er,self.eyawD]
        self._error_r_pub.publish(er_msg)
        # Publish pose target yaw target
        target_msg=Float32MultiArray()
        target_msg.data=[self.tx,self.ty,self.tz,self.tyawD]
        self._target_pub.publish(target_msg)
        #publish to the drone via /mavros/setpoint_raw/local
        setpoint_raw_local_pub_msg=PositionTarget()
        setpoint_raw_local_pub_msg.coordinate_frame = setpoint_raw_local_pub_msg.FRAME_LOCAL_NED
        setpoint_raw_local_pub_msg.header.stamp = rospy.get_rostime()
        setpoint_raw_local_pub_msg.type_mask = PositionTarget.IGNORE_PX + \
                        PositionTarget.IGNORE_PY + \
                        PositionTarget.IGNORE_VZ + \
                        PositionTarget.IGNORE_AFX + \
                        PositionTarget.IGNORE_AFY + \
                        PositionTarget.IGNORE_AFZ + \
                        PositionTarget.IGNORE_YAW 

        setpoint_raw_local_pub_msg.velocity.x = self.Vout_2d[0]
        setpoint_raw_local_pub_msg.velocity.y = self.Vout_2d[1]
        setpoint_raw_local_pub_msg.position.z = self.tz
        setpoint_raw_local_pub_msg.yaw_rate = self.yaw_rate
        self._setpoint_raw_local_pub.publish(setpoint_raw_local_pub_msg)

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_Cyaw(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , 1*np.sin(Y), 0]
                        ,[-1*np.sin(Y) , np.cos(Y) , 0]
                        ,[0 , 0 , 1]])
        return Cyaw

    def get_Cyaw_2d(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , np.sin(Y)]
                        ,[-1*np.sin(Y) , np.cos(Y)]])
        return Cyaw

    def get_list_of_list_from_csv(self,csv_path):
        csv_file = csv_path

        data = []

        with open(csv_file, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Skip the first row
            for row in csv_reader:
                data.append(row)

        return data

        # # Print the list of lists
        # for row in data:
        #     print(row)
