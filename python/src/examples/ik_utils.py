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


def convert_to_pose_format(q_list, joint_structure):
    """Convert IK solution to pose format for manipulators"""
    pos = joint_structure.copy()

    # Extract manipulator joint values from IK solution
    left_manipulator = q_list[15:22]
    right_manipulator = q_list[22:]
    
    pos["left_manipulator"] = left_manipulator.copy()
    pos["right_manipulator"] = right_manipulator.copy()

    return pos


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


def create_ik_tasks():
    """Create IK tasks for left and right end effectors"""
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
    
    return left_task, right_task


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
        self.left_task, self.right_task = create_ik_tasks()
        
        # Set robot to appropriate mode
        fsm_state = 2 if is_simulation else 11
        self.client.set_fsm_state(fsm_state)
        
        # Initialize joint structure
        self.joint_structure = {
            "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
            "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
            "left_hand": [0, 0, 0, 0, 0, 0],
            "right_hand": [0, 0, 0, 0, 0, 0],
        }
    
    def solve_ik(self, target_poses):
        """
        Solve inverse kinematics for dual-arm robot
        
        Args:
            target_poses (dict): Dictionary with 'left_end_effector' and 'right_end_effector' 
                                containing pin.SE3 objects
            
        Returns:
            dict: Joint positions for manipulators
        """
        dt = 1e-2
        stop_threshold = 1e-6
        max_iterations = 1000
        
        # Set target poses (already pin.SE3 objects)
        left_target = target_poses["left_end_effector"]
        right_target = target_poses["right_end_effector"]
        
        self.left_task.set_target(left_target)
        self.right_task.set_target(right_target)
        
        print(f"Target left EE position: {left_target.translation}")
        print(f"Target right EE position: {right_target.translation}")
        
        # Initialize configuration
        configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0)
        
        # Solve IK
        print("Solving IK for dual-arm robot...")
        nb_steps = 0
        
        while nb_steps < max_iterations:
            # Compute errors
            left_error = np.linalg.norm(self.left_task.compute_error(configuration))
            right_error = np.linalg.norm(self.right_task.compute_error(configuration))
            total_error = left_error + right_error
            
            if total_error < stop_threshold:
                print(f"Converged! Total error: {total_error:.6f}")
                break
                
            # Solve IK with both tasks
            dv = pink.solve_ik(
                configuration,
                tasks=[self.left_task, self.right_task],
                dt=dt,
                damping=1e-8,
                solver="quadprog",
            )
            
            # Integrate joint velocities
            q_out = pin.integrate(self.robot.model, configuration.q, dv * dt)
            configuration = pink.Configuration(self.robot.model, self.robot.data, q_out)
            nb_steps += 1
        
        if nb_steps >= max_iterations:
            print(f"Warning: Reached maximum iterations ({max_iterations})")
        
        return convert_to_pose_format(q_out, self.joint_structure)
    
    def get_joints(self):
        """
        Get current joint positions for all groups
        
        Returns:
            dict: Current joint positions for all groups
        """
        pos = self.joint_structure.copy()

        pos["left_manipulator"] = self.client.get_group_state("left_manipulator", "position")
        pos["right_manipulator"] = self.client.get_group_state("right_manipulator", "position")
        
        # Only read hand data if not in simulation mode
        if not self.is_simulation:
            pos["left_hand"] = self.client.get_group_state("left_hand", "position")
            pos["right_hand"] = self.client.get_group_state("right_hand", "position")
        
        return pos
    
    def get_poses(self):
        """
        Get current end effector poses as pin.SE3 objects
        
        Returns:
            dict: Current end effector poses as pin.SE3 objects
        """
        # Get current joint positions
        current_joints = self.get_joints()
        
        # Create full joint configuration vector
        q = self.robot.q0.copy()
        
        # Update with current manipulator positions
        left_manip = current_joints["left_manipulator"]
        right_manip = current_joints["right_manipulator"]
        
        # Assuming left manipulator joints start at index 15, right at index 22
        q[15:22] = left_manip
        q[22:29] = right_manip
        
        # Update robot data with current configuration
        pin.computeJointJacobians(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        
        # Get end effector poses
        left_ee_frame_id = self.robot.model.getFrameId("left_end_effector_link")
        right_ee_frame_id = self.robot.model.getFrameId("right_end_effector_link")
        
        left_ee_pose = self.robot.data.oMf[left_ee_frame_id]
        right_ee_pose = self.robot.data.oMf[right_ee_frame_id]
        
        return {
            "left_end_effector": left_ee_pose,
            "right_end_effector": right_ee_pose
        }
    
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
            positions = {
                "left_manipulator": interpolate_position(
                    init_pos["left_manipulator"], 
                    target_positions["left_manipulator"], 
                    step, total_steps
                ),
                "right_manipulator": interpolate_position(
                    init_pos["right_manipulator"], 
                    target_positions["right_manipulator"], 
                    step, total_steps
                ),
            }
            
            # Only include hand data if not in simulation mode
            if not self.is_simulation:
                positions["left_hand"] = interpolate_position(
                    init_pos["left_hand"], 
                    target_positions["left_hand"], 
                    step, total_steps
                )
                positions["right_hand"] = interpolate_position(
                    init_pos["right_hand"], 
                    target_positions["right_hand"], 
                    step, total_steps
                )
            
            self.client.set_joint_positions(positions)
            time.sleep(1 / self.frequency)
    
    def set_poses(self, target_poses, duration=3):
        """
        Set end effector poses using IK
        
        Args:
            target_poses (dict): Dictionary with 'left_end_effector' and 'right_end_effector' 
                                containing pin.SE3 objects
            duration (float): Movement duration in seconds
        """
        # Solve IK to get joint positions
        ik_solution = self.solve_ik(target_poses)
        
        # Move to the IK solution
        self.set_joints(ik_solution, duration)