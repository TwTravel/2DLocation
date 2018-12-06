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


INIT_JOINT_QPOS =np.asarray([-0.19408803, -1.07770249, 1.80195949, -
                       2.29505332, -1.57079633, 2.94750463])


# TARGET_POS_QUAT = np.asarray([[0.85, 0.0, 0.95, 0.0, -1.0, 0.0, 0.0],
#                             [0.85, 0.0, 0.8, 0.0, -1.0, 0.0, 0.0],
#                             [0.95, 0.0, 0.8, 0.0, -1.0, 0.0, 0.0]])

TARGET_POS_QUAT = np.asarray([[0.75, 0.0, 0.95, 0.0, 1.0, 0.0, -1],
                              [0.75, 0.0, 0.95, 0.0, 1.0, 0.0, -1]])

# TARGET_POS_QUAT = np.asarray([[0.75, 0.0, 0.95, 0.0, 1.0, 0.0, -1],
#                               [0.9, 0.25, 0.8, 0.0, 1.0, 0.0, -1],
#                               [0.6, 0.25, 0.8, 0.0, 1.0, 0.0, -1],
#                               [0.6, -0.25, 0.8, 0.0, 1.0, 0.0, -1],
#                               [0.9, -0.25, 0.8, 0.0, 1.0, 0.0, -1],
#                               [0.9, 0.25, 1.1, 0.0, 1.0, 0.0, -1],
#                               [0.6, 0.25, 1.1, 0.0, 1.0, 0.0, -1],
#                               [0.6, -0.25, 1.1, 0.0, 1.0, 0.0, -1],
#                               [0.9, -0.25, 1.1, 0.0, 1.0, 0.0, -1]])
WIDTH = 0.0

def move_rule(pos_quat, joint_qpos_init):
    assert pos_quat.shape == (7,)

    pos = pos_quat[:3]
    quat = pos_quat[3:]
    rmat = rotations.quat2mat(quat)
    joint_qpos = ik.choose_ik(*ik.calculate_inverse_kinematics(pos, rmat, ik.UR10_RG6_Para), q_init=joint_qpos_init)
    # joint_qpos += np.asarray([0, -np.pi/2.0] * 3)
    return joint_qpos






def main():
    rospy.init_node("calibration", anonymous=True)
    pub = rospy.Publisher("/ur_driver/angle_tool_move_cmd",
                          AngleToolMoveCmd, queue_size=1)
    rate = rospy.Rate(5)
    # target = np.asarray([0.75, 0.0, 0.95, 0.0, 1.0, 0.0, -1.0])
    # joint_qpos = move_rule(target, INIT_JOINT_QPOS)
    # print(joint_qpos)
    tgt_cnt = 0
    # last_joint_qpos = INIT_JOINT_QPOS

    while not rospy.is_shutdown():
        tgt_pos_quat = TARGET_POS_QUAT[tgt_cnt]
        tgt_joint_qpos = move_rule(tgt_pos_quat, INIT_JOINT_QPOS)
        joint_state = np.asarray(rospy.wait_for_message("/joint_states", JointState).position)
        transform = ik.forward(joint_state)
        pos = transform[0:3, 3]
        quat = rotations.mat2quat(transform[0:3, 0:3])

        if np.linalg.norm(joint_state-tgt_joint_qpos) < 1e-4:

            transform = ik.forward(joint_state)
            pos = transform[0:3, 3]
            quat = rotations.euler2quat(rotations.mat2euler(transform[0:3, 0:3]))
            print("Current joint pos", joint_state)
            print("forward pos quat:", pos, quat)
            print("Reach the target point: {}".format(tgt_pos_quat))
            print("Reach the target joint: {}".format(tgt_joint_qpos))
            tgt_cnt += 1
            time.sleep(3)
        if tgt_cnt == len(TARGET_POS_QUAT):
            print("Reach all points!")
            break
            # last_joint_qpos = joint_state

        angle_tool_move_cmd = AngleToolMoveCmd()
        angle_tool_move_cmd.position = tgt_joint_qpos
        angle_tool_move_cmd.max_speed = 0.1
        # angle_tool_move_cmd.tool = WIDTH
        pub.publish(angle_tool_move_cmd)


        rate.sleep()
        

        




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass