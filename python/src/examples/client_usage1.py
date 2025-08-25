import time 
from fourier_aurora_client import AuroraClient
# import h5py


import numpy as np
import pink
import pinocchio as pin




def interpolate_position(init_pos, target_pos, step, total_steps):
    """Linear interpolation between initial and target positions"""
    return [i + (t - i) * step / total_steps for i, t in zip(init_pos, target_pos)]

def move_joints(client, init_pos, target_pos, frequency, duration):
    """Move joints from initial to target position with linear interpolation"""
    total_steps = frequency * duration
    for step in range(total_steps + 1):
        positions = {
            # "left_leg": interpolate_position(init_pos["left_leg"], target_pos["left_leg"], step, total_steps),
            # "right_leg": interpolate_position(init_pos["right_leg"], target_pos["right_leg"], step, total_steps),
            # "waist": interpolate_position(init_pos["waist"], target_pos["waist"], step, total_steps),
            # "head": interpolate_position(init_pos["head"], target_pos["head"], step, total_steps),
            "left_manipulator": interpolate_position(init_pos["left_manipulator"], target_pos["left_manipulator"], step, total_steps),
            "right_manipulator": interpolate_position(init_pos["right_manipulator"], target_pos["right_manipulator"], step, total_steps),
            "left_hand": interpolate_position(init_pos["left_hand"], target_pos["left_hand"], step, total_steps),
            "right_hand": interpolate_position(init_pos["right_hand"], target_pos["right_hand"], step, total_steps),
        }
        client.set_joint_positions(positions)
        time.sleep(1 / frequency)


# 4-17

def get_current_pose(client):
    POS = {
        "left_leg": [0, 0, 0, 0, 0, 0],
        "right_leg": [0, 0, 0, 0, 0, 0],
        "waist": [0],
        "head": [0, 0],
        "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "left_hand": [0, 0, 0, 0, 0, 0],
        "right_hand": [0, 0, 0, 0, 0, 0],
    }

    POS["left_manipulator"] = client.get_group_state("left_manipulator", "position")
    POS["right_manipulator"] = client.get_group_state("right_manipulator", "position")
    POS["left_hand"] = client.get_group_state("left_hand", "position")
    POS["right_hand"] = client.get_group_state("right_hand", "position")
    return POS


if __name__ == "__main__":

    # Initialize client
    client = AuroraClient.get_instance(123, robot_name="gr2t2v2", serial_number=None)
    time.sleep(1)

    # Configuration
    FREQUENCY = 225
    DURATION = 3

    # Position definitions
    INIT_POS = {
        "left_leg": [0, 0, 0, 0, 0, 0],
        "right_leg": [0, 0, 0, 0, 0, 0],
        "waist": [0],
        "head": [0, 0],
        "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
        "left_hand": [0, 0, 0, 0, 0, 0],
        "right_hand": [0, 0, 0, 0, 0, 0],
    }

    TARGET_POS = {
        "left_leg": [0, 0, 0, 0, 0.7, 0],
        "right_leg": [0, 0, 0, 0, 0.7, 0],
        "waist": [0.0], 
        "head": [0.1, 0.1],
        "left_manipulator": [1, 0, 0, -0.7, 0, 0, 0],
        "right_manipulator": [0, 0, 0, -0.7, 0, 0, 0],
        "left_hand": [3.0, 3.0, 1.5, 1.5, 0.5, 0.5],
        "right_hand": [3.0, 1.5, 1.5, 1.5, 0.5, 0.5],
    }

    kp_config = {
        "left_leg": [200.0, 200.0, 120.0, 200.0, 90.0, 120.0],
        "right_leg": [200.0, 200.0, 120.0, 200.0, 90.0, 120.0],
        "waist": [150],
        "head": [30, 30],
        "left_manipulator": [90, 90, 90, 90, 10, 10, 10],
        "right_manipulator": [90, 90, 90, 90, 10, 10, 10]
    }

    kd_config = {
        "left_leg": [20, 20, 12, 20, 9, 12],
        "right_leg": [20, 20, 12, 20, 9, 12],
        "waist": [15],
        "head": [ 2.4,  2.4],
        "left_manipulator": [ 9, 9, 9, 9, 0.1, 0.1, 0.1],
        "right_manipulator": [ 9, 9, 9, 9, 0.1, 0.1, 0.1],
    }
#---------------------------------------------- read from hdf5----------------------------------------------------------------
    def convert2poseformat(qlist):
        "only for left hand and right hand"
        POS = {
            "left_leg": [0, 0, 0, 0, 0, 0],
            "right_leg": [0, 0, 0, 0, 0, 0],
            "waist": [0],
            "head": [0, 0],
            "left_manipulator": [0, 0, 0, 0, 0, 0, 0],
            "right_manipulator": [0, 0, 0, 0, 0, 0, 0],
            "left_hand": [0, 0, 0, 0, 0, 0],
            "right_hand": [0, 0, 0, 0, 0, 0],
        } 

        print(len(qlist))
        left_hand = qlist[15:22]
        right_hand = qlist[22:]
        POS["left_manipulator"] = left_hand.copy()
        POS["right_manipulator"] = right_hand.copy()
        print(left_hand)
        print(right_hand)
        return POS



    '''
    STATE_JOINT_NAMES = [
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_hip_pitch_joint",
        "left_knee_pitch_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_hip_pitch_joint",
        "right_knee_pitch_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_pitch_joint",
        "waist_roll_joint",
        "head_yaw_joint",
        "head_roll_joint",
        "head_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_wrist_yaw_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_yaw_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
    ]
'''
#---------------------------------------------- test ik----------------------------------------------------------------
    def ortho6d_to_so3(ortho6d):
        """
        Convert from continuous 6D rotation representation to SO(3), adapted from
        On the Continuity of Rotation Representations in Neural Networks
        https://arxiv.org/pdf/1812.07035.pdf
        https://github.com/papagina/RotationContinuity/blob/master/sanity_test/code/tools.py
        """
        x_raw = ortho6d[:3]
        y_raw = ortho6d[3:6]

        x = x_raw / np.linalg.norm(x_raw)
        z = np.cross(x, y_raw)
        z = z / np.linalg.norm(z)
        y = np.cross(z, x)
        return np.column_stack((x, y, z))

    def xyzortho6d_to_se3(xyzortho6d):
        """
        Convert continuous 6D rotation representation to SE(3).
        """
        xyz = xyzortho6d[:3]
        ortho6d = xyzortho6d[3:]
        so3 = ortho6d_to_so3(ortho6d)
        se3 = np.eye(4)
        se3[:3, :3] = so3
        se3[:3, 3] = xyz
        return se3

    dt = 1e-2
    stop_thres = 1e-6
    max_iterations = 1000

    import os
    urdf_path = os.path.join(os.path.dirname(__file__), "robot.urdf")
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=[os.path.dirname(__file__)],
        root_joint=None,
    )
    print(f"URDF description successfully loaded in {robot}")
    print("Robot model loaded successfully!")
    print(f"Number of joints: {robot.model.nq}")
    print(f"Number of frames: {robot.model.nframes}")

    # Initialize robot data
    robot.data = pin.Data(robot.model)
    
    # Get joint limits
    low = robot.model.lowerPositionLimit
    high = robot.model.upperPositionLimit
    robot.q0 = pin.neutral(robot.model)
    
    print(f"Joint limits - Low: {low}")
    print(f"Joint limits - High: {high}")
    
    # Frame details - find end effector frames
    joint_names = robot.model.names
    print("\nJoint names:")
    for i, name in enumerate(joint_names):
        print(f"  {i}: {name}")
    
    # Find end effector frames
    frame_names = []
    for i in range(robot.model.nframes):
        frame_name = robot.model.frames[i].name
        frame_names.append(frame_name)
        if "end_effector" in frame_name:
            print(f"Found end effector frame: {frame_name} (index: {i})")
    
    # Create tasks for left and right end effectors
    left_ee_frame = "left_end_effector_link"
    right_ee_frame = "right_end_effector_link"
    
    # Check if frames exist
    left_frame_id = None
    right_frame_id = None
    base_link_id = None
    
    for i, frame_name in enumerate(frame_names):
        if frame_name == left_ee_frame:
            left_frame_id = i
        elif frame_name == right_ee_frame:
            right_frame_id = i
    
        elif frame_name == "base_link":
            base_link_id = i
    
    
    print(f"Left EE frame ID: {left_frame_id}")
    print(f"Right EE frame ID: {right_frame_id}")
    print
    
    # Create tasks for both end effectors


    # left_task = FrameTask(left_ee_frame, [1.0, 1.0, 1.0], [1.0, 1.0, 1.0])
    # right_task = FrameTask(right_ee_frame, [1.0, 1.0, 1.0], [1.0, 1.0, 1.0])
    

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


    # Set initial configuration
    # q_init = robot.q0.copy()

    # print(q_init)
    # input()

    configuration = pink.Configuration(robot.model, robot.data, robot.q0) #initial configuration

    
    # Set target poses (example: move both hands to different positions)
    # Left hand target

    poses = [ 0.3090981,  0.23688767,  0.2431964 , -0.2142459 , -0.01407145  ,0.9766784,
              0.43532259,  0.89372831,  0.10836949,  0.29423556, -0.2041497,   0.26918888,
              -0.33982287,  0.23328049,  0.91109859, -0.25946691,  0.90789846, -0.32923746,
              0.001,       0.        ,  0.535705,    0.9484644,  -0.13843021, -0.285048,
              0.1444218,   0.98951622,  0.        ] # target poses length 27



    left_target = pin.SE3(xyzortho6d_to_se3(poses[0:9]))
    left_task.set_target(left_target)
    
    # Right hand target  
    right_target = pin.SE3(xyzortho6d_to_se3(poses[9:18]))
    right_task.set_target(right_target)
    
    print(f"\nTarget left EE pose:")
    print(f"  Position: {left_target.translation}")
    
    print(f"\nTarget right EE pose:")
    print(f"  Position: {right_target.translation}")
    
    # Solve IK for both arms
    print("\nSolving IK for dual-arm robot...")
    
    nb_steps = 0
    while nb_steps < max_iterations:
        # Compute errors
        left_error = np.linalg.norm(left_task.compute_error(configuration))
        right_error = np.linalg.norm(right_task.compute_error(configuration))
        total_error = left_error + right_error
        
        # print(f"Step {nb_steps}: Left error = {left_error:.6f}, Right error = {right_error:.6f}, Total = {total_error:.6f}")
        
        if total_error < stop_thres:
            print(f"Converged! Total error: {total_error:.6f}")
            break
            
        # Solve IK with both tasks
        dv = pink.solve_ik(
            configuration,
            tasks=[left_task, right_task],
            dt=dt,
            damping=1e-8,
            solver = "quadprog",
        )
        
        # print(dv)



        # Integrate joint velocities
        q_out = pin.integrate(robot.model, configuration.q, dv * dt)
        configuration = pink.Configuration(robot.model, robot.data, q_out)
    
        
        nb_steps += 1
    
    if nb_steps >= max_iterations:
        print(f"Warning: Reached maximum iterations ({max_iterations})")
    
    # Final results
    print(q_out)


    print(configuration.get_transform("left_end_effector_link", "base_link"))


#----------------------------------------------move joint example----------------------------------------------------------------
    print("\nTesting move joint function..")
    # state = input("switch to task\nPlease desired state number: ")
    # use pd stand fot upper joint control
    client.set_fsm_state(11)

    # use usercontrollerA fot upper joint control
    # client.set_state(3)
    # client.set_upper_state(2)

    qout = convert2poseformat(q_out)

    input("move to init pose")

    move_joints(client, get_current_pose(client), INIT_POS, FREQUENCY, DURATION)



    input("set robot joints to desired position\nPress Enter to continue...")



    move_joints(client, INIT_POS, qout, FREQUENCY, DURATION)

    input("read joints")
    print("Left manipulator position:", client.get_group_state("left_manipulator", "position"))
    print("Right manipulator position:", client.get_group_state("right_manipulator", "position"))


#     # Move back to initial position
#     input("set robot joints to home position\nPress Enter to continue...")
#     move_joints(client, TARGET_POS, INIT_POS, FREQUENCY, DURATION)

# #----------------------------------------------set velocity example----------------------------------------------------------------
#     print("\nTesting set velocity function..")
#     input("please make sure your aurora have install locomotion task in UserControllerA\nPress Enter to continue...")
#     client.set_state(3)     # set state to UserControllerA
#     client.set_upper_state(1)   # set upper state to Act State (Arm Swing)
#     client.set_velocity_source(2)   # set velocity source to Navigation

#     while True:
#     # move forward

#         input("send move forward command\nPress Enter to continue...")
#         client.set_velocity(1, 0, 0)
#         # move left
#         input("send move left command\nPress Enter to continue...")
#         client.set_velocity(0, 1, 0)

#         # rotate clockwise
#         input("send rotate clockwise command\nPress Enter to continue...")
#         client.set_velocity(0, 0, 1)
#         # stop
#         input("send stop command\nPress Enter to continue...")
#         client.set_velocity(0, 0, 0)
    # client.set_velocity_source(0)

#----------------------------------------------set stand pose example----------------------------------------------------------------
    # client.set_state(2)
    # client.set_velocity_source(2)
    
    # input("start sending pose cmd\nPress Enter to continue...")
    # client.set_stand_pose(-0.2, 0, 0)
    # input("start sending pose cmd\nPress Enter to continue...")
    # client.set_stand_pose(-0.2, 0.2, 0)
    # input("start sending pose cmd\nPress Enter to continue...")
    # client.set_stand_pose(-0.2, 0.2, 0.2)


    # input("stop sending pose cmd\nPress Enter to continue...")    
    # client.set_stand_pose(0, 0, 0)
    
    # input("start sending back pose cmd\nPress Enter to continue...")
    # client.set_stand_pose(1, 1, 1)
    # input("stop sending back pose cmd\nPress Enter to continue...")
    # client.set_stand_pose(0, 0, 0)

#----------------------------------------------subscriber example----------------------------------------------------------------
    # while True:
    #     try:
    #         print(f"state: {client.get_state()}")
    #         print(f"upper_state: {client.get_upper_state()}")
    #         print(f"velocity_command_source: {client.get_velocity_source()}")
    #         print(f"stand_pose: {client.get_stand_pose()}")
    #         print(f"left_leg: {client.get_group_state('left_leg')}")
    #         print(f"left_manipulator: {client.get_cartesian_state('left_manipulator')}")
    #         print(f"base: rpy: {client.get_base_data('rpy')}")
    #         print(f"true: vel_B: {client.get_true_data('vel_B')}")
    #         print(f"contact_prob: {client.get_contact_data('contact_prob')}")
            
    #     except KeyError as e:
    #         print(f"Error: {e}")
            
    #     time.sleep(0.1)
    

    # client.close()
    # print("User command test completed successfully.")


