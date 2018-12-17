#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from ur_msgs.msg import AngleToolMoveCmd
import utils.compute_ik as ik
from utils import angle_math
from gym.envs.robotics import rotations
import time
#import cv2
import os

def move_rule(act, q_init):
    assert act.shape == (7,)

    target_pos = act[:3]

    target_rot = rotations.quat2mat(act[3:])

    arm_action = ik.choose_ik(
        *ik.calculate_inverse_kinematics(target_pos, target_rot, ik.UR10_RG6_Para), q_init=q_init)

    return arm_action


def basic_insert_rule(obj_pos, obj_euler_z, shbox_pos, shbox_euler_z, obj_xy_offset=np.zeros(2), obj_euler_z_offset=0.0):
    init = np.asarray([-0.19408803, -1.07770249, 1.80195949, -2.29505332, -1.57079633, 2.94750463])
    euler_center = np.array([0.0, -np.pi / 2, np.pi])
    tgt_pos = obj_pos + np.array([0.0, 0.0, 0.25])
    tgt_quat = rotations.quat_mul(rotations.euler2quat(np.array([0.0, 0.0, obj_euler_z])), rotations.euler2quat(euler_center))
    tgt = np.concatenate([tgt_pos, tgt_quat])
    arm_joint_action = move_rule(tgt, init)
    gripper_action = np.array([0.03])
    result = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
 


    tgt_pos = obj_pos + np.array([0.0, 0.0, 0.02])

    tgt = np.concatenate([tgt_pos, tgt_quat])
    arm_joint_action = move_rule(tgt, init)
    gripper_action = np.array([0.03])
    action = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
    result = np.concatenate([result, action], axis=0)

    gripper_action = np.array([0.01])
    action = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
    result = np.concatenate([result, action], axis=0)


    tgt_pos = obj_pos + np.array([0.0, 0.0, 0.25])
    tgt = np.concatenate([tgt_pos, tgt_quat])
    arm_joint_action = move_rule(tgt, init)
    action = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
    result = np.concatenate([result, action], axis=0)



    rel_pos = rotations.quat_rot_vec(rotations.euler2quat(np.array([0.0, 0.0,shbox_euler_z ])), np.concatenate([obj_xy_offset, np.array([0.15])]))   

    tgt_pos = shbox_pos + rel_pos
    tgt_quat = rotations.quat_mul(rotations.euler2quat(rotations.normalize_angles(np.array([0.0, 0.0, shbox_euler_z + obj_euler_z_offset]))), rotations.euler2quat(euler_center))
    tgt = np.concatenate([tgt_pos, tgt_quat])
    arm_joint_action = move_rule(tgt, init)
    action = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
    result = np.concatenate([result, action], axis=0)

    gripper_action = np.array([0.01])
    action = np.concatenate([arm_joint_action, gripper_action]).reshape(1, -1)
    result = np.concatenate([result, action], axis=0)
   

     
    return result


def main():


    circle_pos = np.array([0.8506,0.2717, 0.775])
    circle_euler_z = 0.0

    shbox_pos = np.array([0.9250,-0.0251, 0.815])
    shbox_euler_z = 0.02826

    #circle_offset = np.array([-0.03, 0.03])
    circle_offset = np.array([0, 0])

    circle_actions = basic_insert_rule(circle_pos, circle_euler_z, shbox_pos, shbox_euler_z,  obj_xy_offset=circle_offset)

    actions = circle_actions
    


    rospy.init_node("calibration", anonymous=True)
    pub = rospy.Publisher("/ur_driver/angle_tool_move_cmd",
                          AngleToolMoveCmd, queue_size=1)
    rate = rospy.Rate(5)
    #======================================================================================
    # joint_state = np.asarray(rospy.wait_for_message("/joint_states", JointState).position)
    # transform = ik.forward(joint_state)
    # pos = transform[0:3, 3]
    # quat = rotations.mat2quat(transform[0:3, 0:3])
    # print("initial pos quat:", pos, quat)

   

    act_cnt = 0
    start_time = time.time()

   
    

    while not rospy.is_shutdown():

        joint_state = np.asarray(rospy.wait_for_message("/joint_states", JointState).position)
        tgt_joint_qpos = actions[act_cnt][0:6]
        width =  actions[act_cnt][6] * 2000
        cur_time = time.time()

        if np.linalg.norm(joint_state-tgt_joint_qpos) < 1e-4 and cur_time - start_time > 0.5:
            transform = ik.forward(joint_state)
            pos = transform[0:3, 3]
            quat = rotations.euler2quat(rotations.mat2euler(transform[0:3, 0:3]))
            print("-------------------")
            print("Current joint pos", joint_state)
            print("forward pos quat:", pos, quat)
            # print("Reach the target point: {}".format(tgt_pos_quat))
            # print("Reach the target joint: {}".format(tgt_joint_qpos))
            act_cnt += 1
            start_time = time.time()
            # time.sleep(3)




        
        angle_tool_move_cmd = AngleToolMoveCmd()
        angle_tool_move_cmd.position = tgt_joint_qpos
        angle_tool_move_cmd.max_speed = 0.02
        angle_tool_move_cmd.tool = width
        pub.publish(angle_tool_move_cmd)
        # print(act_cnt)


        if act_cnt == actions.shape[0]:
            break

        rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass