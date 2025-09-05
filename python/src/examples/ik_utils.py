import time
import os
import numpy as np
import pink
import pinocchio as pin
from fourier_aurora_client import AuroraClient


def print_joints(joints):
    """Print joint positions in a readable format"""
    print(f"\n=== Joint Positions ===")
    for joint, pos in joints.items():
        formatted = "[" + ", ".join(f"{v:8.2f}" for v in pos) + "]"
        print(f"{joint:20}: {formatted}")
    print("=" * 50)

def print_poses(poses):
    """Pretty-print SE3 poses with position and RPY rotation in degrees."""
    print("\n=== End effector poses ===")
    for name, se3 in poses.items():
        pos = se3.translation
        roll, pitch, yaw = np.degrees(pin.rpy.matrixToRpy(se3.rotation))
        print(f"{name:20}:")
        print(f"{'':20}  Position: [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]")
        print(f"{'':20}  Rotation (deg): rx={roll:8.2f}, ry={pitch:8.2f}, rz={yaw:8.2f}")
    print("=" * 50)


def interpolate_position(init_pos, target_pos, step, total_steps):
    """Linear interpolation between initial and target positions"""
    return [i + (t - i) * step / total_steps for i, t in zip(init_pos, target_pos)]


def joints_to_q(joints_dict, q0):
    """
    Convert joint dictionary to full configuration vector q
    
    Args:
        joints_dict (dict): Dictionary with joint groups and their positions
        q0 (np.array): Base configuration vector
    
    Returns:
        np.array: Full configuration vector q
    """
    q = q0.copy()
    
    # Map joint groups to their indices in the configuration vector
    if "left_manipulator" in joints_dict:
        q[15:22] = joints_dict["left_manipulator"]
    if "right_manipulator" in joints_dict:
        q[22:29] = joints_dict["right_manipulator"]
    if "left_hand" in joints_dict:
        q[29:35] = joints_dict["left_hand"]
    if "right_hand" in joints_dict:
        q[35:41] = joints_dict["right_hand"]
    
    return q


def q_to_joints(q, joint_structure):
    """
    Convert full configuration vector q to joint dictionary
    
    Args:
        q (np.array): Full configuration vector
        joint_structure (dict): Template joint structure
    
    Returns:
        dict: Joint dictionary with extracted values
    """
    joints = joint_structure.copy()
    
    # Extract joint values from configuration vector
    joints["left_manipulator"] = q[15:22].copy()
    joints["right_manipulator"] = q[22:29].copy()
    joints["left_hand"] = q[29:35].copy()
    joints["right_hand"] = q[35:41].copy()
    
    return joints


def setup_robot_model(urdf_path=None):
    """Setup and load robot model from URDF"""
    if urdf_path is None:
        urdf_path = os.path.join(os.path.dirname(__file__), "robot.urdf")
    
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=[os.path.dirname(__file__)],
        root_joint=None,
    )
    
    robot.data = pin.Data(robot.model)
    robot.q0 = pin.neutral(robot.model)
    
    print(f"Robot model loaded successfully!")
    print(f"Number of joints: {robot.model.nq}")
    print(f"Number of frames: {robot.model.nframes}")
    
    return robot


class HumanoidPlanner:
    """A class for planning and controlling humanoid robot movements using IK"""
    
    def __init__(self, robot_name="gr2t2v2", is_simulation=True, frequency=225, urdf_path=None):
        """
        Initialize the HumanoidPlanner
        
        Args:
            robot_name (str): Name of the robot
            is_simulation (bool): Whether running in simulation mode
            frequency (int): Control frequency in Hz
            urdf_path (str): Path to URDF file (optional)
        """
        self.is_simulation = is_simulation
        self.frequency = frequency
        
        # Initialize client
        self.client = AuroraClient.get_instance(123, robot_name=robot_name, serial_number=None)
        time.sleep(1)
        
        # Setup robot model and IK tasks
        self.robot = setup_robot_model(urdf_path)
        self.build_ik_tasks()
        
        # Set robot to appropriate mode
        fsm_state = 2 if is_simulation else 11
        self.client.set_fsm_state(fsm_state)
        
        # Initialize joint structure
        self.joint_structure = {
            "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
            "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        }
        if not self.is_simulation:
            self.joint_structure["left_hand"] = [0, 0, 0, 0, 0, 0]
            self.joint_structure["right_hand"] = [0, 0, 0, 0, 0, 0]
        
        # Store joint group keys for easy access
        self.joint_groups = list(self.joint_structure.keys())

    def build_ik_tasks(self):
        """Build IK tasks for left and right end effectors"""
        left_task = pink.tasks.RelativeFrameTask(
            "left_end_effector_link",
            "base_link",
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        right_task = pink.tasks.RelativeFrameTask(
            "right_end_effector_link",
            "base_link",
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )
        
        posture_task = pink.tasks.PostureTask(cost=1e-3)
        
        self.ik_tasks = {
            "left_end_effector_link": left_task,
            "right_end_effector_link": right_task,
            "posture_task": posture_task,
        }
        
        # Initialize configuration and set posture target
        self.configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0)
        self.set_posture_target_from_current_configuration()

    def set_posture_target_from_current_configuration(self):
        """Set the posture task target from the current configuration"""
        self.ik_tasks["posture_task"].set_target_from_configuration(self.configuration)

    def set_poses(self, target_poses, duration=3):
        """
        Set end effector poses using IK and execute the movement
        
        Args:
            target_poses (dict): Dictionary with 'left_end_effector' and 'right_end_effector' 
                                containing pin.SE3 objects
            duration (float): Movement duration in seconds
        """
        dt = 1e-2
        stop_threshold = 1e-6
        max_iterations = 1000
        
        # Set target poses (already pin.SE3 objects)
        for frame_name, task in self.ik_tasks.items():
            if frame_name in target_poses:
                task.set_target(target_poses[frame_name])
        # Solve IK
        print("Solving IK for dual-arm robot...")
        nb_steps = 0
        
        while nb_steps < max_iterations:
            # Compute errors
            total_error = 0
            for task in self.ik_tasks.values():
                total_error += np.linalg.norm(task.compute_error(self.configuration))
            
            if total_error < stop_threshold:
                print(f"Converged! Total error: {total_error:.6f}")
                break
                
            # Solve IK with all tasks
            dv = pink.solve_ik(
                self.configuration,
                tasks=list(self.ik_tasks.values()),
                dt=dt,
                damping=1e-8,
                solver="quadprog",
            )
            
            # Integrate joint velocities
            self.configuration.integrate_inplace(dv, dt)
            nb_steps += 1
        
        if nb_steps >= max_iterations:
            print(f"Warning: Reached maximum iterations ({max_iterations})")
        
        # Convert IK solution to joint format
        ik_solution = q_to_joints(self.configuration.q, self.joint_structure)
        
        # Execute the movement using set_joints
        self.set_joints(ik_solution, duration)
    
    def get_joints(self):
        """
        Get current joint positions for all groups
        
        Returns:
            dict: Current joint positions for all groups
        """
        pos = self.joint_structure.copy()

        # Get all joint group positions
        for joint_group in self.joint_groups:
            pos[joint_group] = self.client.get_group_state(joint_group, "position")
        
        return pos
    
    def get_poses(self):
        """
        Get current end effector poses as pin.SE3 objects
        
        Returns:
            dict: Current end effector poses as pin.SE3 objects
        """
        # Get current joint positions
        current_joints = self.get_joints()
        
        # Convert to full configuration vector
        q = joints_to_q(current_joints, self.robot.q0)
        
        # Update robot data with current configuration
        pin.computeJointJacobians(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        
        # Get end effector poses
        poses = {}
        for ee_name in ["left_end_effector_link", "right_end_effector_link"]:
            ee_frame_id = self.robot.model.getFrameId(ee_name)
            poses[ee_name] = self.robot.data.oMf[ee_frame_id]
        
        return poses
    
    def set_joints(self, target_positions, duration=3):
        """
        Set joint positions with smooth interpolation
        
        Args:
            target_positions (dict): Target joint positions
            duration (float): Movement duration in seconds
        """
        init_pos = self.get_joints()
        total_steps = self.frequency * duration
        
        for step in range(total_steps + 1):
            positions = {}
            
            # Interpolate all joint group positions
            for joint_group in self.joint_groups:
                positions[joint_group] = interpolate_position(
                    init_pos[joint_group], 
                    target_positions[joint_group], 
                    step, total_steps
                )
            
            self.client.set_joint_positions(positions)
            time.sleep(1 / self.frequency)
    
