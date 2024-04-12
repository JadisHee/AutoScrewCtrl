import robotcontrol as RC
import time
import math
import numpy as np

import ControlTools as ct

# from transforms3d import quaternions
# 系统初始化
RC.Auboi5Robot.initialize()
robot = RC.Auboi5Robot()
# pos_data = tuple[0,1]
# pos_data = (1,1,12,2,23,3,4)


def ConnectToAubo(ipAddress, port):
    RC.logger_init()
    RC.logger.info("{0} connecting...".format(RC.Auboi5Robot.get_local_time()))
    handle = robot.create_context()
    RC.logger.info("robot.rsh={0}".format(handle))
    
    queue = RC.Queue()

    p = RC.Process(target=RC.runWaypoint, args=(queue,))
    p.start()
    RC.time.sleep(5)
    print("process started.")

    
    result = robot.connect(ipAddress, port)

    if result != RC.RobotErrorType.RobotError_SUCC:
        RC.logger.info("connect server{0}:{1} failed.".format(ipAddress, port))
    else:
        RC.logger.info("connect server{0}:{1} successed.".format(ipAddress, port))


def MoveAubo(target_pose,target_rpy, joint_velocity,joint_acceleration):
    robot.enable_robot_event()
    robot.init_profile()
    SetVelocity(joint_velocity,joint_acceleration)
    robot.move_to_target_in_cartesian(target_pose,target_rpy)

    # 当法兰的实际位置与目标位置的距离小于0.5mm时，才跳出循环进入下一阶段
    while(1):
        current_data = robot.get_current_waypoint()
        current_pos = current_data['pos']
        dist = math.sqrt((current_pos[0] - target_pose[0])**2 + (current_pos[1] - target_pose[1])**2 + (current_pos[2] - target_pose[2])**2)
        if dist <= 0.0005:
            break

def SetVelocity(joint_velocity,joint_acceleration):
    joint_maxvelc = (joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity, joint_velocity, joint_velocity)
  
    joint_maxacc = (joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration)
  
    robot.set_joint_maxacc(joint_maxacc)
    robot.set_joint_maxvelc(joint_maxvelc)
    robot.set_arrival_ahead_blend(0.05)


if __name__ == '__main__':

    #--------------------------------------------------------------------

    #------------------------设置协作臂相关参数----------------------------
    # 设置关节速度，单位：rad/s
    joint_velocity = 0.08
    # 设置关节加速度，单位：rad/s²
    joint_acceleration = 0.17

    # 设置Aubo的ip和port
    AuboIpAddress = '192.168.1.107'
    AuboPort = 8899

    # 拍照的示教位置
    origin_pos = (0.382987,0.045358,0.6)
    origin_rpy = (180,0,180)

    
    #--------------------------------------------------------------------

    # 连接至Aubo
    ConnectToAubo(AuboIpAddress,AuboPort)
    


    # 回到拍照的位置
    MoveAubo(origin_pos,origin_rpy,joint_velocity,joint_acceleration)
