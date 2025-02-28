from robotiq_socket import RobotiqCModelURCap
import time
import numpy as np


# # Check different target positions
# print("\nChecking different target positions")
# for k in range(0,N_CYCLES):
#     print(f"\nCycle {k}/{N_CYCLES}")
#     if not gripper.is_moving():
#         goal = max(int(np.random.rand()*gripper._max_position),0)
#         print(f"Position goal: {goal}")
#         gripper.move_and_wait_for_pos(goal, FIXED_VEL, FIXED_FORCE)
#         while gripper._get_var(gripper.PRE) != goal:
#             print("Waiting for command acknoledgement...")
#             time.sleep(0.001)
    
#     print(f"Attained position: {gripper.get_current_position()}")
#     time.sleep(1)

# # Check different target speeds
# print("\n\nChecking different target speeds")
# for k in range(0,N_CYCLES):
#     print(f"\nCycle {k}/{N_CYCLES}")
#     if not gripper.is_moving():
#         pos_goal = max(int(np.random.rand()*gripper._max_position),0)
#         speed_goal = max(int(np.random.rand()*gripper._max_speed),1)
#         print(f"Position goal: {pos_goal} | Speed goal: {speed_goal}")
#         gripper.move_and_wait_for_pos(pos_goal, speed_goal, FIXED_FORCE)
#         while gripper._get_var(gripper.PRE) != pos_goal:
#             print("Waiting for command acknoledgement...")
#             time.sleep(0.001)
    
#     print(f"Attained position: {gripper.get_current_position()} | Attained speed: {gripper.get_current_velocity()}")
#     time.sleep(1)

# # Check different target forces
# print("\n\nChecking different target forces")
# for k in range(0,N_CYCLES):
#     print(f"\nCycle {k}/{N_CYCLES}")
#     if not gripper.is_moving():
#         pos_goal = max(int(np.random.rand()*gripper._max_position),0)
#         force_goal = max(int(np.random.rand()*gripper._max_force),1)
#         print(f"Position goal: {pos_goal} | Force goal: {force_goal}")
#         gripper.move_and_wait_for_pos(pos_goal, FIXED_VEL, force_goal)
#         while gripper._get_var(gripper.PRE) != pos_goal:
#             print("Waiting for command acknoledgement...")
#             time.sleep(0.001)
    
#     print(f"Attained position: {gripper.get_current_position()} | Attained force: {gripper.get_current_effort()}")
#     time.sleep(1)



def check_gripper_targets(target_type, n_cycles, gripper, fixed_vel, fixed_force):
    print(f"\n\n### Checking different target {target_type}s ###")
    for k in range(0, n_cycles):
        print(f"\nCycle {k+1}/{n_cycles}")
        if not gripper.is_moving():
            pos_goal = max(int(np.random.rand() * gripper._max_position), 0)
            if target_type == 'position':
                print(f"Position goal: {pos_goal}")
                gripper.move_and_wait_for_pos(pos_goal, fixed_vel, fixed_force)
                while gripper._get_var(gripper.PRE) != pos_goal:
                    print("Waiting for command acknowledgement...")
                    time.sleep(0.001)
                print(f"Attained position: {gripper.get_current_position()}")

            elif target_type == 'speed':
                speed_goal = max(int(np.random.rand() * gripper._max_speed), 1)
                print(f"Position goal: {pos_goal} | Speed goal: {speed_goal}")
                gripper.move_and_wait_for_pos(pos_goal, speed_goal, fixed_force)
                while gripper._get_var(gripper.PRE) != pos_goal:
                    print("Waiting for command acknowledgement...")
                    time.sleep(0.001)
                print(f"Attained position: {gripper.get_current_position()} | Attained speed: {gripper.get_current_velocity()}")
            
            elif target_type == 'force':
                force_goal = max(int(np.random.rand() * gripper._max_force), 1)
                print(f"Position goal: {pos_goal} | Force goal: {force_goal}")
                gripper.move_and_wait_for_pos(pos_goal, fixed_vel, force_goal)
                while gripper._get_var(gripper.PRE) != pos_goal:
                    print("Waiting for command acknowledgement...")
                    time.sleep(0.001)
                print(f"Attained position: {gripper.get_current_position()} | Attained force: {gripper.get_current_effort()}")
            
            else:
                raise ValueError("Invalid target type")
        time.sleep(1)


# Constants
IP_ADDRESS = '192.168.10.2'
RUN_AUTO_CALIBRATION = True
N_CYCLES = 10
FIXED_VEL = 100
FIXED_FORCE = 50

# Create gripper object
gripper = RobotiqCModelURCap(IP_ADDRESS)
gripper.activate(RUN_AUTO_CALIBRATION)

# Check different target positions, speeds and forces
check_gripper_targets('position', N_CYCLES, gripper, FIXED_VEL, FIXED_FORCE)
check_gripper_targets('speed', N_CYCLES, gripper, FIXED_VEL, FIXED_FORCE)
check_gripper_targets('force', N_CYCLES, gripper, FIXED_VEL, FIXED_FORCE)