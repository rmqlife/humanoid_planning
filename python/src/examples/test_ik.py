"""
Example script demonstrating the HumanoidPlanner class for IK-based robot control.
This script shows how to use the HumanoidPlanner to perform inverse kinematics
and control a humanoid robot's manipulators.
"""

from ik_utils import HumanoidPlanner, print_joints, print_poses
import pinocchio as pin
import numpy as np
import time


# Initial position (all zeros)
INIT_JOINTS = {
    "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
    "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
    "left_hand": [0, 0, 0, 0, 0, 0],
    "right_hand": [0, 0, 0, 0, 0, 0],
}
    
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
    """Convert continuous 6D rotation representation to pin.SE3"""
    xyz = np.array(xyzortho6d[:3])  # Convert to numpy array
    ortho6d = xyzortho6d[3:]
    so3 = ortho6d_to_so3(ortho6d)
    return pin.SE3(so3, xyz)




def main():
    """Main function to demonstrate the HumanoidPlanner class"""
    # Initialize planner
    planner = HumanoidPlanner(is_simulation=True, frequency=1000)
    

    # Original target poses in 6D rotation representation
    # Left end effector: [x, y, z, rx1, ry1, rz1, rx2, ry2, rz2]
    left_6d = [0.3090981, 0.23688767, 0.2431964, -0.2142459, -0.01407145, 0.9766784, 0.43532259, 0.89372831, 0.10836949]
    
    # Right end effector: [x, y, z, rx1, ry1, rz1, rx2, ry2, rz2]
    right_6d = [0.29423556, -0.2041497, 0.26918888, -0.33982287, 0.23328049, 0.91109859, -0.25946691, 0.90789846, -0.32923746]
    
    # Convert to pinocchio SE3 using original values
    target_poses = {
        "left_end_effector": xyzortho6d_to_se3(left_6d),
        "right_end_effector": xyzortho6d_to_se3(right_6d)
    }
    
    # Example usage of the HumanoidPlanner class
    
    # Get and print current joint positions
    print_joints(planner.get_joints())
    
    # Print target poses for verification
    print_poses(target_poses)
    
    # Move to initial position
    input("\nPress Enter to move to initial pose...")
    planner.set_joints(INIT_JOINTS, duration=3)
    
    # Move to IK solution using poses
    input("Press Enter to move to IK solution using poses...")
    planner.set_poses(target_poses, duration=3)
    


    modified_target_poses = target_poses.copy()
    for _ in range(3):
        time.sleep(1)

        modified_target_poses["left_end_effector"].translation[1] -= 0.05
        # modified_target_poses["right_end_effector"].translation[2] += 0.05
        planner.set_poses(modified_target_poses, duration=3)
        print_poses(planner.get_poses())

    # Read and print final joint positions
    input("Press Enter to read final joint positions...")
    print_joints(planner.get_joints())


if __name__ == "__main__":
    main()


