#!/usr/bin/env python
import rospy
import csv
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import tf.transformations
import os
import datetime  # For timestamped log filename

# For odor detection - assuming a similar implementation to the example
try:
    from plume_sim_fast import CosmosFast
    import pandas as pd
except ImportError:
    rospy.logwarn("Could not import CosmosFast, will use dummy odor detection")


class OdorTrackerNode:
    def __init__(self):
        # Initialize node
        rospy.init_node("odor_tracking_controller")

        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.prev_heading = 0.0
        self.reached_start_position = False  # Flag to track if we've reached the starting position

        # Starting position (well inside the plume)
        self.start_x = 40.0
        self.start_y = 10.0

        # We'll define a threshold x-coordinate for the source
        # When x <= source_x, we will command a landing
        self.source_x = 0.0  # adjust to your desired threshold

        # Odor detection variables
        self.current_odor = 0.0
        self.last_odor = 0.0
        self.odor_threshold = 4.5  # From example code
        self.hit_occurred = False
        self.last_hit_time = rospy.Time.now().to_sec()

        # Behavior parameters
        self.target_pos = np.array([0.0, 0.0])  # Odor source location (if known)
        self.closest_to_source = 0.5  # Distance to consider source reached
        self.is_surging = False
        self.is_casting = True  # Start with casting behavior
        self.surge_speed = 1.2  # m/s during surge
        self.base_speed = 0.5   # m/s during normal flight
        self.surge_duration = 2.0  # seconds to continue surge after losing odor
        self.surge_end_time = 0

        # Casting parameters
        self.cast_base_freq = 0.5   # Base frequency for sine wave (Hz)
        self.cast_growth_rate = 0.5  # Rate at which amplitude grows per second out of odor
        self.max_cast_amplitude = 12.0  # Maximum crosswind amplitude

        # Altitude to maintain
        self.altitude = 1.2  # meters

        # ------------- Logging Setup -------------
        self.data_log = []
        self.log_columns = [
            "timestamp", "x", "y", "z", "yaw", "odor_concentration",
            "is_surging", "is_casting", "vx", "vy", "vz", "heading", "angular_velocity"
        ]
        self.log_dir = os.path.expanduser("../odor_sim_assets/")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(self.log_dir, f"odor_tracking_vel_{timestamp_str}.csv")
        with open(self.log_filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(self.log_columns)
        rospy.loginfo(f"Logging data to: {self.log_filename}")
        rospy.on_shutdown(self.save_log)
        # -----------------------------------------

        # Load odor predictor if available
        self.load_odor_predictor()

        # ROS subscribers
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)

        # ROS publishers
        self.setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        from geometry_msgs.msg import TwistStamped
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.drone_marker_pub = rospy.Publisher("drone_position_marker", Marker, queue_size=10)

        # Set up service clients
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Control rate (20Hz)
        self.dt = 0.005  # 20 Hz
        self.rate = rospy.Rate(1.0 / self.dt)

        # Setup visualization markers
        self.setup_markers()

        rospy.loginfo("Odor tracking controller initialized")

    def load_odor_predictor(self):
        """Load odor prediction model if available"""
        self.predictor = None
        try:
            rospy.loginfo("Attempting to load odor model...")
            dirname = rospy.get_param(
                '~odor_model_path',
                '/home/flybot2/catkin_ws/src/trajectory_gen/odor_sim_assets/'
            )
            hmap_data = np.load(str(dirname) + "hmap.npz")
            fdf = pd.read_hdf(str(dirname) + 'whiff.h5')
            fdf_nowhiff = pd.read_hdf(str(dirname) + 'nowhiff.h5')

            from plume_sim_fast import CosmosFast
            self.predictor = CosmosFast(
                fitted_p_heatmap=hmap_data['fitted_heatmap']*1,
                xedges=hmap_data['xedges'],
                yedges=hmap_data['yedges'],
                fdf=fdf,
                fdf_nowhiff=fdf_nowhiff
            )
            rospy.loginfo("CosmosFast predictor initialized successfully")
        except Exception as e:
            rospy.logwarn(f"Could not load odor model: {e}")
            rospy.logwarn("Will use a simple simulated model instead")
            self.predictor = self.DummyPredictor()  # Fallback to a simple model

    class DummyPredictor:
        """Simple dummy odor predictor for testing"""
        def __init__(self):
            self.source_pos = np.array([0.0, 0.0])
            self.plume_width = 3.0
            self.max_distance = 20.0

        def step_update(self, x, y, dt=0.05):
            pos = np.array([x, y])
            distance = np.linalg.norm(pos - self.source_pos)
            if distance > self.max_distance:
                return 0.0
            if x > 0 and abs(y) < self.plume_width:
                concentration = 10.0 * np.exp(-0.1 * distance)
                concentration *= (1.0 + 0.2 * np.random.randn())
                return max(0.0, concentration)
            else:
                return 0.0

    def setup_markers(self):
        """Setup visualization markers for RViz"""
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
        """Add a marker for an odor detection point"""
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

    def state_callback(self, msg):
        self.current_state = msg

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

        # Run tracking step only if in OFFBOARD and armed
        if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
            self.tracking_step()

    def tracking_step(self):
        """Main tracking algorithm step"""
        # If we haven't reached our designated start position yet, move there first
        if not self.reached_start_position:
            current_pos = self.current_position
            dist_to_start = math.sqrt(
                (current_pos[0] - self.start_x) ** 2 + (current_pos[1] - self.start_y) ** 2
            )
            # If close to the start position, enable odor tracking
            if dist_to_start < 3.0:
                rospy.loginfo("Reached starting position. Beginning odor tracking.")
                self.reached_start_position = True
                self.last_hit_time = rospy.Time.now().to_sec()
            else:
                takeoff_pose = PositionTarget()
                takeoff_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                takeoff_pose.type_mask = (
                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                    PositionTarget.IGNORE_YAW_RATE
                )
                takeoff_pose.position.x = self.start_x
                takeoff_pose.position.y = self.start_y
                takeoff_pose.position.z = self.altitude
                takeoff_pose.yaw = math.pi
                self.setpoint_pub.publish(takeoff_pose)
            return

        # Once start position is reached, start odor tracking
        current_time = rospy.Time.now().to_sec()
        x, y = self.current_position[0], self.current_position[1]

        # ----------------------------------------------------------------
        # NEW CODE: Land if crossing the source in the x-direction
        if x <= self.source_x:
            rospy.loginfo("Crossed source in x-direction, initiating AUTO.LAND")
            land_mode_req = SetModeRequest()
            land_mode_req.custom_mode = "AUTO.LAND"
            response = self.set_mode_client.call(land_mode_req)
            if response.mode_sent:
                rospy.loginfo("AUTO.LAND mode set successfully.")
            else:
                rospy.logwarn("Failed to set AUTO.LAND mode.")
            rospy.signal_shutdown("Source crossed; landing now.")
            return
        # ----------------------------------------------------------------

        # Update odor concentration from the predictor
        if self.predictor:
            self.current_odor = self.predictor.step_update(x, y, dt=self.dt)
        else:
            self.current_odor = 0.0

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

        # After surge time ends (and we're not currently in odor), resume casting
        if current_time > self.surge_end_time and self.is_surging and not currently_in_odor:
            rospy.loginfo("Surge time over, switching to casting")
            self.is_surging = False
            self.is_casting = True

        # Compute velocity commands
        vx, vy, vz = 0.0, 0.0, 0.0
        if self.is_surging:
            # Fly straight upwind at surge speed
            vx = -self.surge_speed
            vy = 0.0
            rospy.loginfo_throttle(1.0, "SURGING")
        else:
            # Casting
            time_since_hit = current_time - self.last_hit_time
            cast_amp = min(self.max_cast_amplitude, self.cast_growth_rate * time_since_hit)
            rospy.loginfo_throttle(
                0.5,
                f"time={time_since_hit:.2f}s, amp={cast_amp:.2f},"
                f" sin_val={math.sin(2.0 * math.pi * self.cast_base_freq * time_since_hit):.2f}"
            )

            freq_factor = 1.0 / (1.0 + 0.009 * time_since_hit)
            current_cast_freq = self.cast_base_freq * freq_factor
            cast_phase = math.sin(2.0 * math.pi * current_cast_freq * time_since_hit)
            vy = cast_amp * cast_phase
            vx = -self.base_speed
            rospy.loginfo_throttle(1.0, f"CASTING with amplitude {cast_amp:.2f}")

        # Altitude hold
        error_z = self.altitude - self.current_position[2]
        vz = 0.5 * error_z

        # Determine heading from velocity
        heading = math.atan2(vy, vx)
        # Compute angular velocity as difference between headings (with angle wrapping)
        angle_diff = math.atan2(
            math.sin(heading - self.prev_heading),
            math.cos(heading - self.prev_heading)
        )
        angular_velocity = angle_diff / self.dt
        self.prev_heading = heading

        # If you have a known source position, check if we've reached it
        dist_to_source = np.linalg.norm(self.target_pos - self.current_position[:2])
        if dist_to_source < self.closest_to_source:
            rospy.loginfo(f"Source reached at {self.current_position[:2]}")
            vx, vy, vz = 0.0, 0.0, 0.0
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            rospy.signal_shutdown("Source reached successfully")
            return

        # Log and publish velocity
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
            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
            + PositionTarget.IGNORE_YAW
        )
        pt.velocity.x = vx
        pt.velocity.y = vy
        pt.velocity.z = vz

        speed_xy = math.sqrt(vx * vx + vy * vy)
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
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def log_data(self, vx, vy, vz, heading, angular_velocity):
        """Append current state data to the log and write to file periodically."""
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
        # Write to file every 50 entries
        if len(self.data_log) >= 50:
            with open(self.log_filename, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)
            self.data_log = []

    def save_log(self):
        """Write any remaining log entries to file on shutdown."""
        if self.data_log:
            rospy.loginfo(f"Saving {len(self.data_log)} log entries to {self.log_filename}")
            with open(self.log_filename, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)

    def takeoff_sequence(self):
        """Execute initial takeoff sequence"""
        rospy.loginfo("Beginning takeoff sequence")

        initial_takeoff = PositionTarget()
        initial_takeoff.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        initial_takeoff.type_mask = (
            PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW_RATE
        )
        current_x = self.current_position[0]
        current_y = self.current_position[1]
        if abs(current_x) < 0.1 and abs(current_y) < 0.1:
            current_x = 1.0
            current_y = 1.0
        initial_takeoff.position.x = current_x
        initial_takeoff.position.y = current_y
        initial_takeoff.position.z = 1  # Minimal altitude
        initial_takeoff.yaw = 0

        takeoff_pose = PositionTarget()
        takeoff_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        takeoff_pose.type_mask = (
            PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW_RATE
        )
        takeoff_pose.position.x = self.start_x
        takeoff_pose.position.y = self.start_y
        takeoff_pose.position.z = self.altitude
        takeoff_pose.yaw = math.pi

        # Send 100 setpoints before starting
        for _ in range(100):
            if rospy.is_shutdown():
                break
            self.setpoint_pub.publish(initial_takeoff)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()
        while not rospy.is_shutdown() and (not self.current_state.armed or self.current_state.mode != "OFFBOARD"):
            self.setpoint_pub.publish(initial_takeoff)
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                response = self.set_mode_client.call(offb_set_mode)
                if response.mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                else:
                    rospy.logwarn("Failed to set OFFBOARD mode")
                last_req = rospy.Time.now()
            if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                response = self.arming_client.call(arm_cmd)
                if response.success:
                    rospy.loginfo("Vehicle armed")
                else:
                    rospy.logwarn("Failed to arm vehicle")
                last_req = rospy.Time.now()
            self.rate.sleep()

        rospy.loginfo("Waiting to reach initial takeoff altitude...")
        takeoff_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_alt = self.current_position[2]
            error = abs(current_alt - 1.2)
            if error < 0.3:
                rospy.loginfo("Reached initial altitude of 1.2m")
                break
            if (rospy.Time.now() - takeoff_start_time) > rospy.Duration(20.0):
                rospy.logwarn("Timeout waiting for takeoff. Continuing anyway.")
                break
            self.setpoint_pub.publish(initial_takeoff)
            self.rate.sleep()

        rospy.loginfo(f"Moving to starting position ({self.start_x}, {self.start_y}, {self.altitude})")
        for _ in range(100):
            if rospy.is_shutdown():
                break
            self.setpoint_pub.publish(takeoff_pose)
            self.rate.sleep()

        rospy.loginfo("Starting position commands sent. Will begin tracking when we reach the vicinity.")
        rospy.loginfo("Takeoff complete, beginning odor tracking")

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo_throttle(1.0, "Waiting for FCU connection...")
            self.rate.sleep()

        rospy.loginfo("Connected to FCU")
        self.takeoff_sequence()
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    try:
        args = rospy.myargv(argv=sys.argv)
        if len(args) >= 3:
            start_x = float(args[1])
            start_y = float(args[2])
        else:
            rospy.logwarn("Start location not provided. Using default values.")
            start_x = 40.0
            start_y = 10.0

        controller = OdorTrackerNode()
        # Override the default start position with terminal inputs (if provided):
        controller.start_x = start_x
        controller.start_y = start_y
        controller.run()
    except rospy.ROSInterruptException:
        pass
