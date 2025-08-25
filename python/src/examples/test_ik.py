import time
import os
import numpy as np
import pink
import pinocchio as pin
from fourier_aurora_client import AuroraClient


def interpolate_position(init_pos, target_pos, step, total_steps):
    """Linear interpolation between initial and target positions"""
    return [i + (t - i) * step / total_steps for i, t in zip(init_pos, target_pos)]


def move_joints(client, init_pos, target_pos, frequency, duration):
    """Move joints from initial to target position with linear interpolation"""
    total_steps = frequency * duration
    for step in range(total_steps + 1):
        positions = {
            "left_manipulator": interpolate_position(init_pos["left_manipulator"], target_pos["left_manipulator"], step, total_steps),
            "right_manipulator": interpolate_position(init_pos["right_manipulator"], target_pos["right_manipulator"], step, total_steps),
            "left_hand": interpolate_position(init_pos["left_hand"], target_pos["left_hand"], step, total_steps),
            "right_hand": interpolate_position(init_pos["right_hand"], target_pos["right_hand"], step, total_steps),
        }
        client.set_joint_positions(positions)
        time.sleep(1 / frequency)


def get_current_pose(client):
    """Get current joint positions for manipulators and hands"""
    pos = {
        "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "left_hand": [0, 0, 0, 0, 0, 0],
        "right_hand": [0, 0, 0, 0, 0, 0],
    }

    pos["left_manipulator"] = client.get_group_state("left_manipulator", "position")
    pos["right_manipulator"] = client.get_group_state("right_manipulator", "position")
    pos["left_hand"] = client.get_group_state("left_hand", "position")
    pos["right_hand"] = client.get_group_state("right_hand", "position")
    return pos


def convert_to_pose_format(q_list):
    """Convert IK solution to pose format for manipulators"""
    pos = {
        "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "left_hand": [0, 0, 0, 0, 0, 0],
        "right_hand": [0, 0, 0, 0, 0, 0],
    }

    # Extract manipulator joint values from IK solution
    left_manipulator = q_list[15:22]
    right_manipulator = q_list[22:]
    
    pos["left_manipulator"] = left_manipulator.copy()
    pos["right_manipulator"] = right_manipulator.copy()
    
    print(f"Left manipulator: {left_manipulator}")
    print(f"Right manipulator: {right_manipulator}")
    return pos


def ortho6d_to_so3(ortho6d):
    """Convert from continuous 6D rotation representation to SO(3)"""
    x_raw = ortho6d[:3]
    y_raw = ortho6d[3:6]

    x = x_raw / np.linalg.norm(x_raw)
    z = np.cross(x, y_raw)
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def xyzortho6d_to_se3(xyzortho6d):
    """Convert continuous 6D rotation representation to SE(3)"""
    xyz = xyzortho6d[:3]
    ortho6d = xyzortho6d[3:]
    so3 = ortho6d_to_so3(ortho6d)
    se3 = np.eye(4)
    se3[:3, :3] = so3
    se3[:3, 3] = xyz
    return se3


def setup_robot_model():
    """Setup and load robot model from URDF"""
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


def solve_ik(robot, left_task, right_task, target_poses):
    """Solve inverse kinematics for dual-arm robot"""
    dt = 1e-2
    stop_threshold = 1e-6
    max_iterations = 1000
    
    # Set target poses
    left_target = pin.SE3(xyzortho6d_to_se3(target_poses[0:9]))
    right_target = pin.SE3(xyzortho6d_to_se3(target_poses[9:18]))
    
    left_task.set_target(left_target)
    right_task.set_target(right_target)
    
    print(f"Target left EE position: {left_target.translation}")
    print(f"Target right EE position: {right_target.translation}")
    
    # Initialize configuration
    configuration = pink.Configuration(robot.model, robot.data, robot.q0)
    
    # Solve IK
    print("Solving IK for dual-arm robot...")
    nb_steps = 0
    
    while nb_steps < max_iterations:
        # Compute errors
        left_error = np.linalg.norm(left_task.compute_error(configuration))
        right_error = np.linalg.norm(right_task.compute_error(configuration))
        total_error = left_error + right_error
        
        if total_error < stop_threshold:
            print(f"Converged! Total error: {total_error:.6f}")
            break
            
        # Solve IK with both tasks
        dv = pink.solve_ik(
            configuration,
            tasks=[left_task, right_task],
            dt=dt,
            damping=1e-8,
            solver="quadprog",
        )
        
        # Integrate joint velocities
        q_out = pin.integrate(robot.model, configuration.q, dv * dt)
        configuration = pink.Configuration(robot.model, robot.data, q_out)
        nb_steps += 1
    
    if nb_steps >= max_iterations:
        print(f"Warning: Reached maximum iterations ({max_iterations})")
    
    return q_out


def main():
    """Main function to demonstrate IK and joint movement"""
    # Initialize client
    client = AuroraClient.get_instance(123, robot_name="gr2t2v2", serial_number=None)
    time.sleep(1)
    
    # Configuration
    FREQUENCY = 225
    DURATION = 3
    
    # Initial position (all zeros)
    INIT_POS = {
        "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "left_hand": [0, 0, 0, 0, 0, 0],
        "right_hand": [0, 0, 0, 0, 0, 0],
    }
    
    # Target poses for IK (6D rotation representation)
    target_poses = [
        0.3090981, 0.23688767, 0.2431964, -0.2142459, -0.01407145, 0.9766784,
        0.43532259, 0.89372831, 0.10836949, 0.29423556, -0.2041497, 0.26918888,
        -0.33982287, 0.23328049, 0.91109859, -0.25946691, 0.90789846, -0.32923746,
        0.001, 0.0, 0.535705, 0.9484644, -0.13843021, -0.285048,
        0.1444218, 0.98951622, 0.0
    ]
    
    # Setup robot model and IK tasks
    robot = setup_robot_model()
    left_task, right_task = create_ik_tasks()
    
    # Solve IK
    q_out = solve_ik(robot, left_task, right_task, target_poses)
    print(f"IK solution: {q_out}")
    
    # Convert IK solution to pose format
    ik_pose = convert_to_pose_format(q_out)
    
    # Set robot to task mode
    client.set_fsm_state(11)
    
    # Move to initial position
    input("Press Enter to move to initial pose...")
    move_joints(client, get_current_pose(client), INIT_POS, FREQUENCY, DURATION)
    
    # Move to IK solution
    input("Press Enter to move to IK solution...")
    move_joints(client, INIT_POS, ik_pose, FREQUENCY, DURATION)
    
    # Read final joint positions
    input("Press Enter to read final joint positions...")
    print("Left manipulator position:", client.get_group_state("left_manipulator", "position"))
    print("Right manipulator position:", client.get_group_state("right_manipulator", "position"))


if __name__ == "__main__":
    main()


