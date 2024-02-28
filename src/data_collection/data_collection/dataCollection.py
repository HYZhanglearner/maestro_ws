import rclpy
from rclpy.node import Node
import h5py
import numpy as np
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf2_ros
import robosuite.utils.transform_utils as robosuite_transform_utils
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import datetime  # For naming groups with the current date and time

class HDF5DataCollector(Node):
    def __init__(self):
        super().__init__("hdf5_data_collector")
        
        # Define thresholds
        self.translation_threshold = 0.2  # Adjust as needed
        self.rotation_threshold = 0.2  # Adjust as needed

        # Subscribe to several topics
        self.subscriber_spaceMouse = self.create_subscription(
            Joy, "spaceMouseMotion", self.spaceMouse_callback, 10
        )
        self.subscriber_optimoJoints = self.create_subscription(
            JointState, "/joint_states", self.opti_joint_state_callback, qos_profile_sensor_data
        )

        """     
            self.subscriber_platoJoints = self.create_subscription(
            JointState,"/plate",self.plato_joint_state_callback,10
        ) 
        """
        NUM_AXES = 6  # Six axes x, y, z, roll, yaw, pitch
        NUM_POSE = 6  # Six axes x, y, z, axis 0, axis 1, axis 2,
        NUM_OPTIMO_JOINTS = 7  # 7 joints for the optimo arm
        NUM_PLATO_JOINTS = 9 # 9 joints for plato hand joint spaces

        # Set up HDF5 file
        current_time = datetime.datetime.now().strftime("%H-%M-%S")
        self.current_group_name = f"trajectory_{current_time}"
        self.hdf5_file = h5py.File(f"Trajectries_{self.current_group_name}.hdf5", "w")
        self.hdf5_group = self.hdf5_file.create_group("data")

        # Initialize datasets for axes and buttons with fixed sizes
        self.spaceMouse_storage = self.hdf5_group.create_dataset(
            "actions_spaceMouse",
            (100, NUM_AXES),  # Initial size based on expected joystick input
            maxshape=(None, NUM_AXES),  # Allows unlimited resizing along the first axis
            dtype="f",
        )

        """         self.pose_storage = self.hdf5_group.create_dataset(
                    "current_eef_pose",
                    (100, NUM_POSE),  # 
                    maxshape=(None, NUM_POSE),  # Allows unlimited resizing
                    dtype="f",
                ) 
        """

        self.optimo_joint_storage = self.hdf5_group.create_dataset(
            "current_optimo_joint_states",
            (100, NUM_OPTIMO_JOINTS),  # Adjust based on the number of joints
            maxshape=(None, NUM_OPTIMO_JOINTS),
            dtype="f",
        )

        """         
            self.plato_joint_storage = self.hdf5_group.create_dataset(
            "current_plato_joint_states",
            (100, NUM_PLATO_JOINTS),  # Adjust based on the number of joints
            maxshape=(None, NUM_PLATO_JOINTS),
            dtype="f",
        ) """

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.end_effector_frame = "ee"  # Change to your end effector's frame name
        self.base_frame = "world"  # Change to your robot's base frame name

        # Variable to keep track of data index
        self.data_index = 0

        # Temporary storage for the latest messages
        self.latest_space_mouse_msg = None
        self.latest_opti_joint_state_msg = None
        self.latest_plato_joint_state_msg = None
        
        # updating all robot states in 20 Hz
        self.pose_timer = self.create_timer(0.05, self.updateAllDataTohdf)

    def get_end_effector_pose(self):
        try:
            # Lookup the transformation between base frame and end effector frame
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_frame, rclpy.time.Time()
            )

            # Convert the transformation's quaternion to Axis angle
            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            )

            axis_angle = robosuite_transform_utils.quat2axisangle(np.array(quaternion))

            # Create a custom pose array including both position and orientation in Euler angles
            pose_array = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                axis_angle[0],  # Axis angle 0
                axis_angle[1],  # Axis angle 1
                axis_angle[2],  # Axis angle 2
            ]

            return pose_array
        except Exception as e:
            self.get_logger().error(
                "Could not transform from {} to {}: {}".format(
                    self.base_frame, self.end_effector_frame, e
                )
            )
            return None

    # Save all the group data to hdf5 all in once.
    def updateAllDataTohdf(self):

        """ pose = self.get_end_effector_pose()
        if pose is not None:
            # Ensure that there is space in the datasets
            if self.data_index >= self.pose_storage.shape[0]:
                new_size = self.data_index + 100  # Increase by 100
                # Resize all relevant datasets
                self.pose_storage.resize((new_size, 6)) # 6 the number of pose
                self.spaceMouse_storage.resize((new_size, self.spaceMouse_storage.shape[1]))
                self.optimo_joint_storage.resize((new_size, self.optimo_joint_storage.shape[1]))
                self.plato_joint_storage.resize((new_size, self.plato_joint_storage.shape[1]))

            # Save the pose to pose storage
            self.pose_storage[self.data_index] = pose
        """
        # print("updateAllDataTohdf(self)")
        if self.data_index >= self.spaceMouse_storage.shape[0]:
                new_size = self.data_index + 100  # Increase by 100
                # Resize all relevant datasets
                
                self.spaceMouse_storage.resize((new_size, self.spaceMouse_storage.shape[1]))
                self.optimo_joint_storage.resize((new_size, self.optimo_joint_storage.shape[1]))
        
        # Save the latest other messages if available
        if self.latest_space_mouse_msg is not None:
            # print("self.latest_space_mouse_msg is not None")
            self.spaceMouse_storage[self.data_index] = np.array(
                self.latest_space_mouse_msg.axes
            )
            self.latest_space_mouse_msg = None  # Reset after saving

        if self.latest_opti_joint_state_msg is not None:
            # print("self.latest_opti_joint_state_msg is not None")
            self.optimo_joint_storage[self.data_index] = np.array(
                self.latest_opti_joint_state_msg.position
            )

            # print(self.optimo_joint_storage[self.data_index])
            
            self.latest_opti_joint_state_msg = None  # Reset after saving

        # if self.latest_plato_joint_state_msg is not None:
        #     self.plato_joint_storage[self.data_index] = np.array(
        #         self.latest_plato_joint_state_msg.position
        #     )
        #     self.latest_plato_joint_state_msg = None  # Reset after saving


        self.data_index += 1
    
        # Log for every 100 data points received
        if self.data_index % 20 == 0:
            self.get_logger().info(
                f"Received {self.data_index} joint state data points so far."
            )

        """     def plato_joint_state_callback(self, msg):
        # Just update the latest message
        self.latest_opti_joint_state_msg = msg """

    def opti_joint_state_callback(self, msg):
        # Just update the latest message
        # print(msg.position)
        self.latest_opti_joint_state_msg = msg

    def spaceMouse_callback(self, msg):
        # print("spaceMouse_callback is called!")
        translation_magnitude = self.calculate_translation_magnitude(msg.axes[:3])
        rotation_magnitude = self.calculate_rotation_magnitude(msg.axes[3:])

        filtered_axes = [0.0] * 6  # Default to all zeros

        if translation_magnitude > self.translation_threshold and translation_magnitude > rotation_magnitude:
            filtered_axes[:3] = msg.axes[:3]  # Keep translation, zero rotation
        elif rotation_magnitude > self.rotation_threshold:
            filtered_axes[3:] = msg.axes[3:]  # Keep rotation, zero translation

        msg.axes = filtered_axes
        # Just update the latest message
        self.latest_space_mouse_msg = msg

    def save_and_close(self):
        # Make sure to close the HDF5 file when you're done to avoid corruption
        self.hdf5_file.close()

    def calculate_translation_magnitude(self, translation_axes):
        return np.linalg.norm(translation_axes)

    def calculate_rotation_magnitude(self, rotation_axes):
        return np.linalg.norm(rotation_axes)

def main(args=None):
    rclpy.init(args=args)
    hdf5_data_collector = HDF5DataCollector()
    try:
        rclpy.spin(hdf5_data_collector)
    except KeyboardInterrupt:
        pass
    finally:
        # Save and close the HDF5 file properly
        hdf5_data_collector.save_and_close()
        rclpy.shutdown()
        print("closed properly!")


if __name__ == "__main__":
    main()
