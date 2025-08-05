#!/usr/bin/env python3
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

import pandas as pd
try:
    from plume_sim_fast import CosmosFast
    import pandas as pd
except ImportError:
    rospy.logwarn("Could not import CosmosFast, will use dummy odor detection instead.")


class OdorTrackerNode:
    def __init__(self):
        rospy.init_node("odor_tracking_controller")

        # --- Internal state variables ---
        self.current_state = State()
        self.active = False  # Will become True when we enter OFFBOARD
        self.current_pose = PoseStamped()
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.prev_heading = 0.0

        # Starting position (example: well inside a plume)
        self.start_x = 40.0
        self.start_y = 10.0
        self.altitude = 1.2  # flight altitude

        # We'll define a threshold x-coordinate for the source
        # When x <= source_x, we will command a landing
        self.source_x = 0.0  # Adjust to your desired threshold

        # Odor detection variables
        self.current_odor = 0.0
        self.odor_threshold = 4.5
        self.last_hit_time = rospy.Time.now().to_sec()
        self.hit_occurred = False

        # Behavior parameters
        self.target_pos = np.array([0.0, 0.0])  # If you know the actual source location
        self.closest_to_source = 0.5
        self.is_surging = False
        self.is_casting = True
        self.surge_speed = 1.2    # m/s
        self.base_speed = 0.5     # m/s
        self.surge_duration = 2.0 # seconds
        self.surge_end_time = 0

        # Casting parameters
        self.cast_base_freq = 0.5
        self.cast_growth_rate = 0.5
        self.max_cast_amplitude = 12.0

        # --- Logging Setup ---
        self.data_log = []
        self.log_columns = [
            "timestamp", "x", "y", "z", "yaw", "odor_concentration",
            "is_surging", "is_casting", "vx", "vy", "vz", "heading", "angular_velocity"
        ]
        self.log_dir = os.path.expanduser(
            "/home/flybot2/catkin_ws/src/trajectory_gen/odor_sim_assets/"
        )
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(
            self.log_dir, f"odor_tracking_vel_{timestamp_str}.csv"
        )
        with open(self.log_filename, "w") as f:
            writer = csv.writer(f)
            writer.writerow(self.log_columns)
        rospy.loginfo(f"Logging data to: {self.log_filename}")
        rospy.on_shutdown(self.save_log)

        # Load odor predictor if available
        self.load_odor_predictor()

        # --- ROS Subscribers ---
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)

        # --- ROS Publishers ---
        # For controlling velocity / setpoint
        self.setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        from geometry_msgs.msg import TwistStamped
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        # --- Services (arming / set_mode) ---
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Main loop rate (20Hz)
        self.dt = 0.05  # 20Hz => 0.05s
        self.rate = rospy.Rate(1.0 / self.dt)

        rospy.loginfo("Odor tracking controller initialized (no markers).")

    # -------------------- Odor Predictor --------------------
    def load_odor_predictor(self):
        """Load odor prediction model if available, else use a dummy."""
        self.predictor = None
        try:
            rospy.loginfo("Attempting to load odor model...")
            dirname = rospy.get_param(
                "~odor_model_path",
                "/home/flybot2/catkin_ws/src/trajectory_gen/odor_sim_assets/"
            )
            hmap_data = np.load(str(dirname) + "rescaled_hmap.npz")
            fdf = pd.read_hdf(str(dirname) + 'whiff_rescaled.h5')
            fdf_nowhiff = pd.read_hdf(str(dirname) + 'nowhiff_rescaled.h5')

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
            self.predictor = self.DummyPredictor()

    class DummyPredictor:
        """Simple dummy odor predictor for testing."""
        def __init__(self):
            self.source_pos = np.array([0.0, 0.0])
            self.plume_width = 3.0
            self.max_distance = 20.0

        def step_update(self, x, y, dt=0.05):
            pos = np.array([x, y])
            distance = np.linalg.norm(pos - self.source_pos)
            if distance > self.max_distance:
                return 0.0
            # If x>0 and abs(y)<plume_width, produce an exponential odor
            if x > 0 and abs(y) < self.plume_width:
                concentration = 10.0 * np.exp(-0.1 * distance)
                concentration *= (1.0 + 0.2 * np.random.randn())
                return max(0.0, concentration)
            else:
                return 0.0

    # -------------------- ROS Callbacks --------------------
    def state_callback(self, msg):
        """Keep track of current FCU state and set `active` if OFFBOARD."""
        self.current_state = msg
        if msg.mode == "OFFBOARD":
            if not self.active:
                rospy.loginfo("Switched to OFFBOARD mode => activating tracking.")
            self.active = True
        else:
            if self.active:
                rospy.logwarn("Left OFFBOARD mode => deactivating tracking.")
            self.active = False

    def pose_callback(self, msg):
        """Update current pose and run tracking step if active & armed."""
        self.current_pose = msg
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        # Convert quaternion to yaw
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        # Only do tracking if we are in OFFBOARD and armed
        if self.active and self.current_state.armed:
            self.tracking_step()

    # -------------------- Main Tracking Logic --------------------
    def tracking_step(self):
        """Odor-tracking logic, runs only if OFFBOARD & armed."""
        x, y, z = self.current_position
        current_time = rospy.Time.now().to_sec()

        # Land if crossing the source in the x-direction
        if x <= self.source_x:
            rospy.loginfo("Crossed source x-threshold => initiating AUTO.LAND")
            land_mode_req = SetModeRequest()
            land_mode_req.custom_mode = "AUTO.LAND"
            response = self.set_mode_client.call(land_mode_req)
            if response.mode_sent:
                rospy.loginfo("AUTO.LAND mode set successfully.")
            else:
                rospy.logwarn("Failed to set AUTO.LAND mode.")
            rospy.signal_shutdown("Source crossed; landing now.")
            return

        # Update odor concentration
        if self.predictor:
            self.current_odor = self.predictor.step_update(x, y, dt=self.dt)
        else:
            self.current_odor = 0.0

        # Check if we are currently in odor
        in_odor = (self.current_odor >= self.odor_threshold)
        if in_odor:
            rospy.loginfo(f"Odor peak detected at {self.current_odor:.2f}")
            self.hit_occurred = True
            self.last_hit_time = current_time
            self.is_surging = True
            self.is_casting = False
            self.surge_end_time = current_time + self.surge_duration
        else:
            self.hit_occurred = False

        # If surge time is over (and not in odor), switch back to casting
        if current_time > self.surge_end_time and self.is_surging and not in_odor:
            rospy.loginfo("Surge time over => switching to casting.")
            self.is_surging = False
            self.is_casting = True

        # Compute velocity commands
        vx, vy, vz = 0.0, 0.0, 0.0
        if self.is_surging:
            # Go straight upwind (negative x)
            vx = -self.surge_speed
            vy = 0.0
            rospy.loginfo_throttle(1.0, "SURGING")
        else:
            # Casting: sin wave in y, slow speed in x
            time_since_hit = current_time - self.last_hit_time
            cast_amp = min(self.max_cast_amplitude, self.cast_growth_rate * time_since_hit)
            freq_factor = 1.0 / (1.0 + 0.009 * time_since_hit)
            current_cast_freq = self.cast_base_freq * freq_factor
            cast_phase = math.sin(2.0 * math.pi * current_cast_freq * time_since_hit)
            vy = cast_amp * cast_phase
            vx = -self.base_speed
            rospy.loginfo_throttle(1.0, f"CASTING with amplitude {cast_amp:.2f}")

        # Simple altitude hold
        alt_error = self.altitude - z
        vz = 0.5 * alt_error

        # Heading control => face the velocity direction
        heading = math.atan2(vy, vx)
        angle_diff = self.angle_difference(heading, self.prev_heading)
        angular_velocity = angle_diff / self.dt
        self.prev_heading = heading

        # If we have a known source location, see if we got close enough
        dist_to_source = np.linalg.norm(self.target_pos - self.current_position[:2])
        if dist_to_source < self.closest_to_source:
            rospy.loginfo("Reached the odor source => shutting down.")
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            rospy.signal_shutdown("Source reached.")
            return

        # Log and publish
        self.log_data(vx, vy, vz, heading, angular_velocity)
        self.publish_velocity(vx, vy, vz, heading)

    # -------------------- Velocity Commands --------------------
    def publish_velocity(self, vx, vy, vz, heading):
        """Publish velocity in both /cmd_vel and /setpoint_raw/local."""
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
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    # -------------------- Data Logging --------------------
    def log_data(self, vx, vy, vz, heading, angular_velocity):
        """Append current state data to the log and flush to CSV periodically."""
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

        # Write out every 50 entries
        if len(self.data_log) >= 50:
            with open(self.log_filename, "a") as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)
            self.data_log = []

    def save_log(self):
        """Write any remaining log entries to file on shutdown."""
        if self.data_log:
            rospy.loginfo(f"Saving {len(self.data_log)} log entries to {self.log_filename}")
            with open(self.log_filename, "a") as f:
                writer = csv.writer(f)
                writer.writerows(self.data_log)

    # -------------------- Main Loop --------------------
    def run(self):
        while not rospy.is_shutdown():
            # Wait for FCU connection
            if not self.current_state.connected:
                rospy.loginfo_throttle(1.0, "Waiting for FCU connection...")
            # Spin at 20Hz
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
        controller.start_x = start_x
        controller.start_y = start_y
        controller.run()

    except rospy.ROSInterruptException:
        pass
