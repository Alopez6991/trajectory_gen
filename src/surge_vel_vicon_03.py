#!/usr/bin/env python
import rospy
import csv
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget, RCIn
from mavros_msgs.srv import SetMode, SetModeRequest
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import tf.transformations
import os
import datetime
from std_msgs.msg import Float32

# For odor detection - same as before
try:
    from plume_sim_fast import CosmosFast
    import pandas as pd
except ImportError:
    rospy.logwarn("Could not import CosmosFast, will use dummy odor detection")


class OdorTrackerNode:
    def __init__(self):
        rospy.init_node("odor_tracking_controller")

        # -----------------------------------------------------
        # 1) STATE-VARIABLES
        # -----------------------------------------------------
        self.active = False  # We'll set this True if in OFFBOARD mode
        self.start_tracking = False  # Will be set to True when RC switch is flipped
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.prev_heading = 0.0

        # RC control values
        self.rc_values = []
        self.rc_tracking_channel = 4  # Typically channel 5 (0-indexed as 4)
        self.state = 0  # Current RC value on tracking channel
        self.flag = 0   # Flag set when RC value is high

        # Surging/casting logic variables
        self.reached_start_position = False
        self.start_x = 4.0
        self.start_y = -0.5
        self.altitude = 2.0
        self.source_x = -2.0  # If x <= this, we land
        self.current_odor = 0.0
        self.last_odor = 0.0
        self.odor_threshold = 4.5
        self.hit_occurred = False
        self.last_hit_time = rospy.Time.now().to_sec()

        self.target_pos = np.array([-2.0, 0.0])
        self.closest_to_source = 0.1
        self.is_surging = False
        self.is_casting = True
        self.surge_speed = 1
        self.base_speed = 0.5
        self.surge_duration = 1.0
        self.surge_end_time = 0

        self.cast_base_freq = 0.5
        self.cast_growth_rate = 0.5
        self.max_cast_amplitude = 1.5

        # -----------------------------------------------------
        # 2) LOGGING SETUP
        # -----------------------------------------------------
        self.data_log = []
        self.log_columns = [
            "timestamp","x","y","z","yaw","odor_concentration",
            "is_surging","is_casting","vx","vy","vz","heading","angular_velocity"
        ]
        self.log_dir = os.path.expanduser("/home/flybot2/catkin_ws/src/trajectory_gen/odor_sim_assets/")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(self.log_dir, f"odor_tracking_vel_{timestamp_str}.csv")
        with open(self.log_filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(self.log_columns)
        rospy.on_shutdown(self.save_log)

        self.load_odor_predictor()

        # -----------------------------------------------------
        # 3) SUBSCRIBERS / PUBLISHERS
        # -----------------------------------------------------
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.rc_sub = rospy.Subscriber("mavros/rc/in", RCIn, self.rc_callback)

        self.setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        from geometry_msgs.msg import TwistStamped
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.drone_marker_pub = rospy.Publisher("drone_position_marker", Marker, queue_size=10)
        self.odor_pub = rospy.Publisher("odor_concentration", Float32, queue_size=10)

        self.dt = 0.05
        self.rate = rospy.Rate(1.0 / self.dt)

        self.setup_markers()

        rospy.loginfo("Odor tracking controller initialized (waiting for OFFBOARD mode).")

        # -----------------------------------------------------
        # 4) NEUTRAL (HOVER) SETPOINT
        # -----------------------------------------------------
        # We'll publish this even if not in OFFBOARD. That way PX4 will see a valid
        # OFFBOARD setpoint stream and allow you to switch to OFFBOARD.
        self.neutral_pose = PositionTarget()
        self.neutral_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.neutral_pose.type_mask = (
            PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW_RATE
        )
        # Hover at local position (0, 0, 4). You can tweak if desired:
        self.neutral_pose.position.x = 0.0
        self.neutral_pose.position.y = 0.0
        self.neutral_pose.position.z = 1.5
        self.neutral_pose.yaw = 0.0

    def load_odor_predictor(self):
        """Load odor prediction model if available."""
        self.predictor = None
        try:
            rospy.loginfo("Attempting to load odor model...")
            dirname = rospy.get_param(
                '~odor_model_path',
                '/home/flybot2/catkin_ws/src/trajectory_gen/odor_sim_assets/'
            )
            hmap_data = np.load(str(dirname) + "hmap.npz")
            fdf = pd.read_hdf(str(dirname) + 'whiff_rescaled.h5')
            fdf_nowhiff = pd.read_hdf(str(dirname) + 'nowhiff_rescaled.h5')

            from plume_sim_fast import CosmosFast
            self.predictor = CosmosFast(
                fitted_p_heatmap=hmap_data['fitted_heatmap']*0.1,
                xedges=hmap_data['xedges'],
                yedges=hmap_data['yedges'],
                fdf=fdf,
                fdf_nowhiff=fdf_nowhiff
            )
            rospy.loginfo("CosmosFast predictor initialized successfully")
        except Exception as e:
            rospy.logwarn(f"Could not load odor model: {e}")
            rospy.logwarn("Will use a simple simulated model instead")
            self.predictor = self.DummyPredictor()

    class DummyPredictor:
        """Simple dummy odor predictor for testing"""
        def __init__(self):
            self.source_pos = np.array([0.0, 0.0])
            self.plume_width = 3.0
            self.max_distance = 20.0
        def step_update(self, x, y, dt=0.05):
            distance = np.linalg.norm(np.array([x,y]) - self.source_pos)
            if distance > self.max_distance:
                return 0.0
            if x > 0 and abs(y) < self.plume_width:
                concentration = 10.0 * np.exp(-0.1 * distance)
                concentration *= (1.0 + 0.2 * np.random.randn())
                return max(0.0, concentration)
            else:
                return 0.0

    def setup_markers(self):
        self.markers = MarkerArray()
        import threading
        marker_thread = threading.Thread(target=self.publish_markers)
        marker_thread.daemon = True
        marker_thread.start()

        drone_marker_thread = threading.Thread(target=self.publish_drone_marker)
        drone_marker_thread.daemon = True
        drone_marker_thread.start()

    def publish_markers(self):
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.markers)
            if hasattr(self, 'whiff_markers') and len(self.whiff_markers.markers) > 0:
                self.marker_pub.publish(self.whiff_markers)
            rospy.sleep(0.5)

    def publish_drone_marker(self):
        while not rospy.is_shutdown():
            from visualization_msgs.msg import Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "drone"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            pos = self.current_position
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            if self.is_surging:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            self.drone_marker_pub.publish(marker)
            rospy.sleep(0.1)

    def add_whiff_marker(self, x, y, z):
        from visualization_msgs.msg import Marker, MarkerArray
        if not hasattr(self, 'whiff_markers'):
            self.whiff_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "whiffs"
        marker.id = len(self.whiff_markers.markers)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.whiff_markers.markers.append(marker)
        self.marker_pub.publish(self.whiff_markers)

    # -----------------------------------------------------
    # STATE AND RC CALLBACKS
    # -----------------------------------------------------
    def state_callback(self, msg):
        """Handle OFFBOARD mode detection"""
        active_param = (msg.mode == "OFFBOARD")
        if not self.active and active_param:
            self.active = True
            rospy.loginfo("OFFBOARD mode activated - im active")
        elif self.active and not active_param:
            self.active = False
            self.start_tracking = False
            self.reached_start_position = False
            rospy.loginfo("Exited OFFBOARD mode - im inactive")

    def rc_callback(self, msg):
        """Handle RC input for tracking activation"""
        if len(msg.channels) <= self.rc_tracking_channel:
            # Not enough channels
            return
            
        # Check RC switch for tracking control (channel 5)
        self.state = msg.channels[self.rc_tracking_channel]
        if self.state >= 1900:
            self.flag = 1
            # If we've reached the start position, start tracking when switch flips
            if self.reached_start_position and not self.start_tracking:
                self.start_tracking = True
                rospy.loginfo("RC switch activated - beginning odor tracking")
                self.last_hit_time = rospy.Time.now().to_sec()
        else:
            self.flag = 0
            
        # For Gazebo testing - also allow keyboard command to start tracking
        if self.reached_start_position and not self.start_tracking:
            # Set up a non-blocking thread to wait for user input for testing
            if not hasattr(self, 'input_prompt_shown'):
                rospy.loginfo("Reached starting position. Hovering and waiting for RC switch or press Enter.")
                self.input_prompt_shown = True
                
                # Start a non-blocking thread to wait for user input
                import threading
                def wait_for_input():
                    input("Press Enter to begin odor tracking...")
                    rospy.loginfo("User input received. Beginning odor tracking.")
                    self.start_tracking = True
                    self.last_hit_time = rospy.Time.now().to_sec()
                
                input_thread = threading.Thread(target=wait_for_input)
                input_thread.daemon = True
                input_thread.start()

    def pose_callback(self, msg):
        self.current_pose = msg
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        # Only run the tracking step if we are actively controlling
        if self.active:
            self.tracking_step()

    # -----------------------------------------------------
    # MAIN TRACKING STEP
    # -----------------------------------------------------
    def tracking_step(self):
        # If we haven't reached the "start position" yet, move there:
        if not self.reached_start_position:
            current_pos = self.current_position
            dist_to_start = math.sqrt(
                (current_pos[0] - self.start_x)**2 + (current_pos[1] - self.start_y)**2
            )
            if dist_to_start < 0.01:
                # We are close enough to start position, mark as reached
                if not hasattr(self, 'start_reached_notified'):
                    rospy.loginfo("Reached starting position. Hovering. Waiting for RC switch or Enter.")
                    self.reached_start_position = True
                    self.start_reached_notified = True

                # Keep publishing the hover setpoint at start location
                hover_pose = PositionTarget()
                hover_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                hover_pose.type_mask = (
                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                    PositionTarget.IGNORE_YAW_RATE
                )
                hover_pose.position.x = self.start_x
                hover_pose.position.y = self.start_y
                hover_pose.position.z = self.altitude
                hover_pose.yaw = math.pi
                self.setpoint_pub.publish(hover_pose)
                return
            else:
                # Move toward start position
                move_pose = PositionTarget()
                move_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                move_pose.type_mask = (
                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                    PositionTarget.IGNORE_YAW_RATE
                )
                move_pose.position.x = self.start_x
                move_pose.position.y = self.start_y
                move_pose.position.z = self.altitude
                move_pose.yaw = math.pi
                self.setpoint_pub.publish(move_pose)
                return

        # If we're at start but haven't gotten tracking signal, keep hovering
        if not self.start_tracking:
            hover_pose = PositionTarget()
            hover_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            hover_pose.type_mask = (
                PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                PositionTarget.IGNORE_YAW_RATE
            )
            hover_pose.position.x = self.start_x
            hover_pose.position.y = self.start_y
            hover_pose.position.z = self.altitude
            hover_pose.yaw = math.pi
            self.setpoint_pub.publish(hover_pose)
            return

        # If we're here, we've reached the start position & received command to track
        current_time = rospy.Time.now().to_sec()
        x, y = self.current_position[0], self.current_position[1]

        # Land if crossing source threshold in x
        if x <= self.source_x:
            rospy.loginfo("Crossed source in x-direction, initiating AUTO.LAND (optional).")
            try:
                land_mode_req = SetModeRequest()
                land_mode_req.custom_mode = "AUTO.LAND"
                response = rospy.ServiceProxy("/mavros/set_mode", SetMode)(land_mode_req)
                if response.mode_sent:
                    rospy.loginfo("AUTO.LAND mode set successfully.")
                else:
                    rospy.logwarn("Failed to set AUTO.LAND mode.")
            except rospy.ServiceException:
                rospy.logwarn("Could not call set_mode to land. Exiting.")
            rospy.signal_shutdown("Source crossed; landing now.")
            return

        # Odor concentration update
        if self.predictor:
            self.current_odor = self.predictor.step_update(x, y, dt=self.dt)
        else:
            self.current_odor = 0.0
        self.odor_pub.publish(Float32(self.current_odor))

        # Check odor
        currently_in_odor = (self.current_odor >= self.odor_threshold)
        if currently_in_odor:
            rospy.loginfo(f"Odor peak detected at concentration {self.current_odor:.2f}")
            self.add_whiff_marker(x, y, self.current_position[2])
            self.hit_occurred = True
            self.last_hit_time = current_time
            self.is_surging = True
            self.is_casting = False
            self.surge_end_time = current_time + self.surge_duration
        else:
            self.hit_occurred = False

        # End surge if needed
        if current_time > self.surge_end_time and self.is_surging and not currently_in_odor:
            rospy.loginfo("Surge time over, switching to casting")
            self.is_surging = False
            self.is_casting = True

        # Compute velocity commands
        vx, vy, vz = 0.0, 0.0, 0.0
        if self.is_surging:
            vx = -self.surge_speed
            vy = 0.0
            rospy.loginfo_throttle(1.0, "SURGING")
        else:
            # time_since_hit = current_time - self.last_hit_time
            # cast_amp = min(self.max_cast_amplitude, self.cast_growth_rate * time_since_hit)
            # freq_factor = 1.0 / (1.0 + 0.09 * time_since_hit)
            # current_cast_freq = self.cast_base_freq * freq_factor
            # cast_phase = math.sin(2.0 * math.pi * current_cast_freq * time_since_hit)
            # vy = cast_amp * cast_phase
            # vx = -self.base_speed
            # rospy.loginfo_throttle(1.0, f"CASTING with amplitude {cast_amp:.2f}")
            # Casting
            time_since_hit = current_time - self.last_hit_time
            cast_amp = min(self.max_cast_amplitude, self.cast_growth_rate * time_since_hit)
            rospy.loginfo_throttle(
                0.5,
                f"time={time_since_hit:.2f}s, amp={cast_amp:.2f},"
                f" sin_val={math.sin(2.0 * math.pi * self.cast_base_freq * time_since_hit):.2f}"
            )

            # freq_factor = 1.0 / (1.0 + 0.009 * time_since_hit)
            # current_cast_freq = self.cast_base_freq * freq_factor
            
            # cast_phase = math.sin(2.0 * math.pi * current_cast_freq * time_since_hit)
            if self.use_normalized_frequency:
                phase_angle = (current_time * self.cast_base_freq) % 1.0  # Normalized to [0,1]
                cast_phase = math.sin(2.0 * math.pi * phase_angle)
            else:
                # Original approach (frequency affects amplitude indirectly)
                cast_phase = math.sin(2.0 * math.pi * self.cast_base_freq * time_since_hit)
            # cast_phase = math.sin(2.0 * math.pi * self.cast_base_freq * frequency_time)

            vy = cast_amp * cast_phase
            vx = -self.base_speed
            rospy.loginfo_throttle(1.0, f"CASTING with amplitude {cast_amp:.2f}")

        # Altitude hold
        error_z = self.altitude - self.current_position[2]
        vz = 0.5 * error_z

        # Yaw from velocity
        heading = math.atan2(vy, vx)
        angle_diff = math.atan2(
            math.sin(heading - self.prev_heading),
            math.cos(heading - self.prev_heading)
        )
        angular_velocity = angle_diff / self.dt
        self.prev_heading = heading

        # Check if we have a known source position
        dist_to_source = np.linalg.norm(self.target_pos - self.current_position[:2])
        if dist_to_source < self.closest_to_source:
            rospy.loginfo(f"Source reached at {self.current_position[:2]}. Shutting down.")
            vx, vy, vz = 0.0, 0.0, 0.0
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            rospy.signal_shutdown("Source reached successfully")
            return

        # Publish & log
        self.log_data(vx, vy, vz, heading, angular_velocity)
        self.publish_velocity(vx, vy, vz, heading)

    def publish_velocity(self, vx, vy, vz, heading):
        from geometry_msgs.msg import TwistStamped
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.header.frame_id = "base_link"
        vel_cmd.twist.linear.x = vx
        vel_cmd.twist.linear.y = vy
        vel_cmd.twist.linear.z = vz
        self.velocity_pub.publish(vel_cmd)

        pt = PositionTarget()
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = (
            PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW
        )
        pt.velocity.x = vx
        pt.velocity.y = vy
        pt.velocity.z = vz
        speed_xy = math.sqrt(vx*vx + vy*vy)
        if speed_xy > 0.1:
            desired_yaw = math.atan2(vy, vx)
            pt.yaw_rate = 0.5 * self.angle_difference(desired_yaw, self.current_yaw)
        else:
            pt.yaw_rate = 0.0

        self.setpoint_pub.publish(pt)

    @staticmethod
    def angle_difference(a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff

    def log_data(self, vx, vy, vz, heading, angular_velocity):
        timestamp = rospy.Time.now().to_sec()
        log_entry = [
            timestamp,
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            self.current_yaw,
            self.current_odor,
            self.is_surging,
            self.is_casting,
            vx, vy, vz,
            heading,
            angular_velocity
        ]
        self.data_log.append(log_entry)
        if len(self.data_log) >= 50:
            with open(self.log_filename, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)
            self.data_log = []

    def save_log(self):
        if self.data_log:
            rospy.loginfo(f"Saving {len(self.data_log)} log entries to {self.log_filename}")
            with open(self.log_filename, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)

    # -----------------------------------------------------
    # run(): ALWAYS PUBLISH NEUTRAL POSE
    # -----------------------------------------------------
    def run(self):
        # Status reporting
        status_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if not self.active:
                # Publish neutral pose if we are NOT in OFFBOARD mode
                self.setpoint_pub.publish(self.neutral_pose)
            # else, if we *are* active, "tracking_step()" does the real commands in pose_callback
            
            # Print status periodically
            if (rospy.Time.now() - status_time) > rospy.Duration(5.0):
                status_msg = f"Status: OFFBOARD={self.active}, "
                status_msg += f"ReachedStart={self.reached_start_position}, "
                status_msg += f"Tracking={self.start_tracking}, "
                status_msg += f"RC_Flag={self.flag}"
                
                if self.active:
                    pos = self.current_position
                    status_msg += f", Pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
                    
                    if self.start_tracking:
                        status_msg += f", Odor={self.current_odor:.2f}"
                        if self.is_surging:
                            status_msg += " (SURGING)"
                        elif self.is_casting:
                            status_msg += " (CASTING)"
                
                rospy.loginfo(status_msg)
                status_time = rospy.Time.now()
            
            self.rate.sleep()

# -----------------------------------------------------
# MAIN
# -----------------------------------------------------
if __name__ == "__main__":
    try:
        args = rospy.myargv(argv=sys.argv)
        if len(args) >= 3:
            start_x = float(args[1])
            start_y = float(args[2])
        else:
            rospy.logwarn("Start location not provided. Using default (40.0, 10.0).")
            start_x = 1.0
            start_y = 0.0

        controller = OdorTrackerNode()
        controller.start_x = start_x
        controller.start_y = start_y
        controller.run()
    except rospy.ROSInterruptException:
        pass