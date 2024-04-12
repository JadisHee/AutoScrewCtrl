import robotcontrol as RC
import time
import math
import numpy as np
import socket
import ControlTools as ct

# from transforms3d import quaternions
# 系统初始化
RC.Auboi5Robot.initialize()
robot = RC.Auboi5Robot()
# pos_data = tuple[0,1]
# pos_data = (1,1,12,2,23,3,4)


def ConnectToAubo(ipAddress, port):
    '''
    param:
        ipAddress: Aubo机器人的ip地址
        port: Aubo机器人的端口
    '''
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

def InitialAll():

    ClawCtrl(1)
    VacuumCtrl(0)
    DriverCtrl(0)

    time.sleep(3)
    # 待传感器就位后，将时延替换为缩回限位的判断

    robot.set_board_io_status(5,"U_DO_03",0)
    robot.set_board_io_status(5,"U_DO_04",0)

    robot.set_board_io_status(5,"U_DO_05",0)

    robot.set_board_io_status(5,"U_DO_07",0)
    robot.set_board_io_status(5,"U_DO_10",0)


def ClawCtrl(target_status):
    '''
    0:闭合夹爪
    1:张开夹爪
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_03",0)
        robot.set_board_io_status(5,"U_DO_04",1)
        # robot.set_board_io_status(5,"U_DO_04",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_04",0)
        robot.set_board_io_status(5,"U_DO_03",1)
        # robot.set_board_io_status(5,"U_DO_03",0)


def VacuumCtrl(target_status):
    '''
    0:关闭真空阀
    1:打开真空阀
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_05",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_05",1)

def DriverCtrl(target_status):
    '''
    0:模组收回
    1:模组伸出
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_10",0)
        robot.set_board_io_status(5,"U_DO_07",1)
        # robot.set_board_io_status(5,"U_DO_04",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_07",0)
        robot.set_board_io_status(5,"U_DO_10",1)

def IsVacuumPressOk():
    '''
    判断吸顶气压是否足够：
    return:
        True: 满足
        False: 不满足
    '''
    IsPressOk = robot.get_board_io_status(4,"U_DI_02")
    if IsPressOk == 0:
        return True
    else:
        return False

def IsMouldDownOk():
    '''
    判断模组是否下降到位：
    return:
        True: 到位
        False: 未到位
    '''
    IsDownOk = robot.get_board_io_status(4,"U_DI_03")
    if IsDownOk == 1:
        return True
    else:
        return False
    
def IsMouldUpOk():
    '''
    判断模组是否上升到位：
    return:
        True: 到位
        False: 未到位
    '''
    IsUpOk = robot.get_board_io_status(4,"U_DI_04")
    if IsUpOk == 1:
        return True
    else:
        return False

def ScrewMotorCtrl(ip, port):

    MotorStart_Hex = "020000000A573033303130313D313B03"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # 连接到目标设备
        s.connect((ip, port))
        
        # 将十六进制字符串转换为字节串
        byte_data = bytes.fromhex(MotorStart_Hex)
        
        # 发送数据
        s.sendall(byte_data)
        
        print("\n电批启动成功！")
        
    except Exception as e:
        print(f"\n电批启动时发生错误：{e}")
        
    finally:
        # 关闭连接
        s.close()

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

def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw

# 获取协作臂末端当前位姿            
def GetCurrentPos():
    current_data = robot.get_current_waypoint()
    current_pos = current_data['pos']
    current_ori = current_data['ori']
    r,p,y = quart_to_rpy(current_ori[1],current_ori[2],current_ori[3],current_ori[0])
    # current_rpy =  RC.Auboi5Robot.quaternion_to_rpy(RC.Auboi5Robot,current_rpy_ori)
    current_all = [current_pos[0],current_pos[1],current_pos[2],
                   r,p,y]

    return current_all

def SetVelocity(joint_velocity,joint_acceleration):
    joint_maxvelc = (joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity, joint_velocity, joint_velocity)
  
    joint_maxacc = (joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration)
  
    robot.set_joint_maxacc(joint_maxacc)
    robot.set_joint_maxvelc(joint_maxvelc)
    robot.set_arrival_ahead_blend(0.05)

def Target(Width,Height,DisCamX,DisCamY,RotationEndToCam,dRuler,PosNow,CirclePos):
    """
    根据相机提供的螺钉位置，控制机械臂移动
    params:
    Width: 图像的宽度
    Height: 图像的高度
    DisCamX: 电批相对相机偏移, x
    DisCamY: 电批相对相机偏移, y
    RotationEndToCam: 末端到相机旋转矩阵
    dRuler: 比例尺
    PosNow: 机械臂末端当前位姿
    CirclePos: 孔在图像中的位置
    """
    # 计算旋转矩阵
    RotVec = np.array([PosNow[3], PosNow[4], PosNow[5]])
    RotBaseToEnd = ct.getRotVec2RotMAT(RotVec)

    MoveX = (CirclePos[0] - Width / 2) * dRuler + DisCamX
    MoveY = (CirclePos[1] - Height / 2) * dRuler + DisCamY

    Move1 = np.array([[MoveX], [MoveY], [0]])
    Move1 = Move1 / 1000

    MoveCam = RotBaseToEnd @ RotationEndToCam @ Move1

    px = PosNow[0] + MoveCam[0]
    py = PosNow[1] + MoveCam[1]
    pz = PosNow[2]

    return [px[0],py[0],pz]


    
if __name__ == '__main__':
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIpAddress = "192.168.1.101"
    HikPort = 8192
    # 通讯触发信号
    msgStart = '123'
    # 前期标定特征尺寸
    # 控制盒实际宽度，单位：mm
    dReal = 66.11
    # 相机参数
    # 海康相机图像尺寸
    Width = 2368
    Height = 1760
    # 末端到相机旋转矩阵
    RotEndToCamera = np.array([[-1, 0, 0],
                               [ 0,-1, 0],
                               [ 0, 0, 1]])
    
    # 示教参数-电批相对相机偏移，相机坐标系下，单位：mm
    dx = 297
    dy = -3.59
    
    # dx = -3.59
    # dy = 297
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
    # 
    # 料盘上钉子的位置
    screw_pos = [[0.219820,-0.269595,0.377820],
                 [0.319820,-0.269595,0.377820],
                 [0.319820,-0.369595,0.377820],
                 [0.219820,-0.369595,0.377820]]
    screw_rpy = [[180,0,180],
                 [180,0,180],
                 [180,0,180],
                 [180,0,180]]
    
    deepth = 0.01
    #--------------------------------------------------------------------

    # 连接至Aubo
    ConnectToAubo(AuboIpAddress,AuboPort)
    


    # 回到拍照的位置
    MoveAubo(origin_pos,origin_rpy,joint_velocity,joint_acceleration)

    # 获取末端相对于基座的当前位置
    PosNow = GetCurrentPos()
#     print["当前的位姿为：",PosNow]

    # 给海康相机发出切换指令
    ct.getHikSwitchPlanByTCP(HikIpAddress,HikPort,'switch','CircleDetect1')

    # 发出检测信号并记录数据
    DetectedData = ct.getDataFromHikSmartCameraByTCP(HikIpAddress,HikPort,msgStart,9)
    
    # 控制盒宽度，单位：像素
    dPixel = DetectedData[0,0]

    # 待测孔的个数
    numTotal = 4

    # 孔的位置
    CirclePos = np.zeros((numTotal, 2))
    for i in range(numTotal):
        CirclePos[i, 0] = DetectedData[2 * i + 1, 0]
        CirclePos[i, 1] = DetectedData[2 * i + 2, 0]
    
    # 比例尺
    Ruler = dReal / dPixel

    
    for j in range(numTotal):
        # 来到螺钉上方
        screw_up = [screw_pos[j][0], screw_pos[j][1], PosNow[2]]
        MoveAubo(screw_up,screw_rpy[j],joint_velocity,joint_acceleration)
        time.sleep(3)

        # 初始化电批：
        # 夹爪张开，吸钉关闭，模组收回
        InitialAll()

        # 向下取螺钉
        MoveAubo(screw_pos[j],screw_rpy[j],joint_velocity,joint_acceleration)
        # 此处添加吸顶气阀触发信号
        time.sleep(1)

        # 开启吸钉
        VacuumCtrl(1)
        time.sleep(1)
        if IsVacuumPressOk() == 0:
            print("\n请检查真空阀气压！！！")
        # 闭合夹爪
        ClawCtrl(0)


        # 回到螺钉上方
        # screw_up = (screw_pos(j)(0), screw_pos(j)(1), PosNow[2])
        MoveAubo(screw_up,screw_rpy[j],joint_velocity,joint_acceleration)
        time.sleep(3)

        # 计算当前螺纹孔上方的位置
        circle_target = Target(Width,Height,dx,dy,RotEndToCamera,Ruler,PosNow,CirclePos[j])
        # time.sleep(3)

        # 来到螺纹孔上方
        MoveAubo(circle_target,origin_rpy,joint_velocity,joint_acceleration)
        time.sleep(3)

        # 模组伸出
        DriverCtrl(1)
        time.sleep(2)
        # 伸出是否到位
        Down_1 = IsMouldDownOk()
        if Down_1 == 0:
            print("\n请检查模组是否下降到位")
        

        # 降至螺纹孔
        end_pos = (circle_target[0],circle_target[1],circle_target[2] - deepth)
        MoveAubo(end_pos,origin_rpy,joint_velocity,joint_acceleration)
        # 此处添加电批触发信号
        time.sleep(1)
        
        # 模组伸出
        # DriverCtrl(1)
        # time.sleep(1)
        # 伸出是否被抵回
        # if Down_1 == 1 & IsMouldDownOk() == 0:
        # 启动电批（上机应用时启动以上判断语句）    
        ClawCtrl(1)
        ScrewMotorCtrl("192.168.1.10",5000)
        time.sleep(5)
        # 添加判断是否拧紧

        # 模组回到初始状态
        InitialAll()

        # 回到螺纹孔上方
        MoveAubo(circle_target,origin_rpy,joint_velocity,joint_acceleration)
        time.sleep(3)

        # 回到拍照的位置
        MoveAubo(origin_pos,origin_rpy,joint_velocity,joint_acceleration)
        time.sleep(3)

    print("运行结束！！")
    


    # current_data = robot.get_current_waypoint()
    # print(robot.get_current_waypoint())

    # MoveAubo(pos_1,rpy_1,joint_velocity,joint_acceleration)
    

    # MoveAubo(pos_2,rpy_2,joint_velocity,joint_acceleration)

    

    # MoveAubo(pos_3,rpy_3,joint_velocity,joint_acceleration)

    

    # MoveAubo(origin_pos,origin_rpy,joint_velocity,joint_acceleration)

    

    # MoveAubo(base_pos,base_rpy,joint_velocity,joint_acceleration)

    


    
