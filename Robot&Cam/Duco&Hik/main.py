import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess

def main():
    
    set_xml = XmlData()
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIp = "192.168.1.101"
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
    RotEndToCamera = np.array([[ 0, 1, 0],
                               [ 1, 0, 0],
                               [ 0, 0, 1]])
    
    # 示教参数-电批相对相机偏移，相机坐标系下，单位：mm
    dx = 297
    dy = -3.59

    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7998

    # 机械臂末端的速度
    vel = 0.1
    acc = 0.2

    # 起始拍照的位姿
    origin_photo_pos = [0.382987,0.045358,0.6, math.pi, 0, math.pi]

    # 料盘上钉子的位置
    screw_pos = [[0.219820,-0.269595,0.377820, math.pi, 0, math.pi],
                 [0.319820,-0.269595,0.377820, math.pi, 0, math.pi],
                 [0.319820,-0.369595,0.377820, math.pi, 0, math.pi],
                 [0.219820,-0.369595,0.377820, math.pi, 0, math.pi]]
    
    # 拍照位置到螺纹孔的下降高度，单位m
    DeepIn = 0.1

    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.103'
    DanikorPort = 8888


    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    set_xml.TypeData = 1
    set_xml.SystemStatusData = True

    # 机械臂回到拍照位置
    duco.DucoMove(origin_photo_pos,vel,acc)
    set_xml.StageData = '协作臂正在前往拍照位置'
    set_xml.StageNumData = 1

    # 获取机械臂拍完照后末端位姿
    PhotoPos =  duco.GetDucoPos()

    # 给海康相机发出切换方案指令
    hik.SetHikSwitchPlan('switch','CircleDetect1')

    # 发出检测信号并记录数据
    DetectedData = hik.GetDataFromHik(msgStart,9)
    set_xml.StageNumData = 2
    set_xml.StageData = '相机正在检测螺纹孔'
    if DetectedData != 0:
        set_xml.CooperativeArmData = 1
    else:
        set_xml.CooperativeArmData = 0
        set_xml.SystemStatusData = False
        set_xml.ErrorData = '相机识别失败'
        return

    # 检测控制盒的宽度，用于计算比例尺，单位: 像素
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
        
        StageFront = ''
        if i == 0:
            StageFront = '拧钉进程: 1/4'
        elif i == 1:
            StageFront = '拧钉进程: 2/4'
        elif i == 2:
            StageFront = '拧钉进程: 3/4'
        elif i == 3:
            StageFront = '拧钉进程: 4/4'

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在前往螺钉上方'
        # 螺钉正上方与机械臂拍照位等高的位置
        screw_up = [screw_pos[j][0], screw_pos[j][1], PhotoPos[2], math.pi, 0, math.pi]
        # 控制机械臂移动至上方
        duco.DucoMove(screw_up,vel,acc)
        time.sleep(1)

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在初始化电批模组'
        # 初始化电批模组
        # 夹爪张开，吸钉关闭，模组收回
        danikor.InitialAllMould()

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在移动至吸钉位置'
        # 控制机械臂向下移动至吸钉位置
        duco.DucoMove(screw_pos[j],vel,acc)

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '开启真空阀'
        # 开启真空阀
        danikor.VacuumCtrl(1)
        set_xml.VacuumStateData = 1
        time.sleep(1)

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '闭合夹爪'
        # 闭合夹爪
        danikor.ClawCtrl(0)
        time.sleep(1)


        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在前往螺纹孔上方'
        # 控制机械臂回到该螺钉上方
        duco.DucoMove(screw_up,vel,acc)
        time.sleep(1)

        # 计算当前目标螺纹孔的位置
        circle_target = hik.GetTargetPos(Width,Height,dx,dy,RotEndToCamera,Ruler,PhotoPos,CirclePos[j])

        # 控制机械臂来到当前目标螺纹孔上方
        duco.DucoMove(circle_target,vel,acc)

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '电批模组准备中'
        # 控制电批模组伸出
        IsOk =  danikor.DriverCtrl(1)

        # 当电批模组伸出到位时继续执行
        if IsOk == 2:
            return 
            
        # 最终拧钉位置
        target_pos = [circle_target[0], circle_target[1], circle_target[2] - DeepIn, circle_target[3], circle_target[4], circle_target[5]]
        
        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在对准孔位'
        # 控制机械臂下降至螺纹孔上方
        duco.DucoMove(target_pos,vel,acc)
        time.sleep(1)

        '''
        在此处添加是否可以开始拧钉的依据
        '''

        set_xml.StageNumData = set_xml.StageNumData + 1
        set_xml.StageData = StageFront + ',' + '正在拧钉'
        # 松开夹爪
        danikor.ClawCtrl(1)
        
        # 开启电批进行拧紧
        danikor.ScrewMotorCtrl()

        '''


        在此处添加判断是否拧紧的依据
        此时暂时以延时替代
        '''
        time.sleep(5)

        # 初始化模组
        danikor.InitialAllMould()
        time.sleep(1)

        # 控制机械臂螺纹孔上方与拍照位等高位置
        duco.DucoMove(circle_target,vel,acc)

    return

def MainTest():
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIp = "192.168.1.101"
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
    RotEndToCamera = np.array([[ 0, 1, 0],
                               [ -1, 0, 0],
                               [ 0, 0, 1]])
    
    # 示教参数-电批相对相机偏移，相机坐标系下，单位：mm
    # dx = -297
    # dy = 3.59

    dx = -296.20
    dy = 4.97

    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7003

    # 机械臂运动速度
    vel_move = 0.4
    acc_move = 0.4
    # 末端接近速度
    vel_end = 0.02
    acc_end = 0.2

    # 轴动角速度
    vel_joint = 0.2617993
    acc_joint = 0.2617993

    # 起始拍照位的安全高度位置
    PhotoSaftyPos = [0.6575234532356262, 1.1132798194885254, 0.8394834399223328, 3.141319990158081, -0.00022508660913445055, 0.06932102143764496]
    PhotoSaftyQNear = [0.906836986541748, 0.42894959449768066, 1.2343454360961914, -0.08622220158576965, -1.570582628250122, -0.7268856167793274]

    # 起始拍照的位姿
    PhotoPos = [0.6574873328208923, 1.1132745742797852, 0.41623711585998535, 3.1413567066192627, -0.0002582818269729614, 0.06934092938899994]
    PhotoQNear = [0.9066452383995056, 0.589190661907196, 1.555666446685791, -0.567784309387207, -1.5708582401275635, -0.7265980243682861]

    # 拍照位收回（可进行一轴旋转）
    PhotoRotatePos = [0.0792575255036354, 0.500156044960022, 0.839511513710022, 3.141336441040039, -0.00023485528072342277, 0.06932101398706436]
    PhotoRotateQNear = [1.0751793384552002, -0.526780366897583, 2.139753580093384, -0.03582853823900223, -1.5701391696929932, -0.5586271286010742]

    # 吸钉位置
    ScrewTakePos = [-0.40543806552886963, -0.8993127942085266, 0.36524279618263245, -3.141566038131714, 4.22334005634184e-06, 3.1415865421295166]
    ScrewTakeQNear = [4.117183208465576, 0.3478044271469116, 2.0381991863250732, -0.8089668154716492, -1.570582628250122, -0.5880364179611206]

    # 吸顶减速位置
    ScrewSlowDownPos = [-0.4054396450519562, -0.8993082046508789, 0.3994438350200653, -3.141578197479248, 8.45219983602874e-06, 3.1415657997131348]
    ScrewSlowDownQNear = [4.117183208465576, 0.31774798035621643, 2.023770332336426, -0.7644334435462952, -1.570522665977478, -0.5880483984947205]

    # 吸钉位置安全高度位置
    ScrewSaftyPos = [-0.4054396450519562, -0.8993082046508789, 0.8394834399223328, -3.141578197479248, 8.45219983602874e-06, 3.1415657997131348]
    ScrewSaftyQNear = [4.117183208465576, 0.31774798035621643, 2.023770332336426, -0.7644334435462952, -1.570522665977478, -0.5880483984947205]

    # 吸钉收回（可进行一轴旋转）
    ScrewRotatePos = [-0.1136971265077591, -0.4934713840484619, 0.8395147323608398, 3.1413629055023193, -0.00023751870321575552, -3.1415765285491943]
    ScrewRotateQNear = [4.147467136383057, -0.5267564058303833, 2.139753580093384, -0.03585250675678253, -1.5701152086257935, -0.5586271286010742]

    # 下降高度
    DownDeepth= 0.0621

    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.11'
    DanikorPort = 8888


    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)

    # 机械臂回到拍照位置安全高度
    duco.DucoMoveL(PhotoSaftyPos,vel_move,acc_move,PhotoSaftyQNear)
    
    # 机械臂来到拍照位置
    duco.DucoMoveL(PhotoPos,vel_move,vel_move,PhotoQNear)
    time.sleep(1)

    # 给海康相机发出切换方案指令
    hik.SetHikSwitchPlan('switch','CircleDetect1')

    # 发出检测信号并记录数据
    DetectedData = hik.GetDataFromHik(msgStart,9)

    # if DetectedData != 0:
    #     print("相机检测成功！！！")
    # else:
    #     print("相机检测失败！！！")
    #     return
    
    # 检测控制盒的宽度，用于计算比例尺，单位: 像素
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

    CircleTargetPos = hik.GetTargetPos(Width,Height,dx,dy,RotEndToCamera,Ruler,PhotoPos,CirclePos[0])
    CircleTargetQNear = PhotoQNear

    # 机械臂回到拍照安全位置
    duco.DucoMoveL(PhotoSaftyPos,vel_move,acc_move,PhotoSaftyQNear)
    
    # 机械臂来到拍照可进行第一轴旋转的位置
    duco.DucoMoveL(PhotoRotatePos,vel_move,acc_move,PhotoRotateQNear)

    # 机械臂第一轴运动至吸钉方向
    duco.DucoMoveJ(ScrewRotateQNear,vel_joint*2,acc_joint)

    # 机械臂运动至吸钉安全高度
    duco.DucoMoveL(ScrewSaftyPos,vel_move,acc_move,ScrewSaftyQNear)

    # 电批模组初始化
    danikor.InitialAllMould()

    # 机械臂下降至吸钉减速位
    duco.DucoMoveL(ScrewSlowDownPos,vel_move,acc_move,ScrewSlowDownQNear)

    danikor.VacuumCtrl(1)
    time.sleep(1)

    # 机械臂慢速运动至吸钉位置
    duco.DucoMoveL(ScrewTakePos,vel_end,acc_end,ScrewTakeQNear)
    time.sleep(5)

    # 夹爪闭合
    danikor.ClawCtrl(0)
    time.sleep(1)

    # 控制机械臂移动至吸钉安全高度
    duco.DucoMoveL(ScrewSaftyPos,vel_move,acc_move,ScrewSaftyQNear)

    # 控制机械臂吸钉方向可进行第一轴旋转的位置
    duco.DucoMoveL(ScrewRotatePos,vel_move,acc_move,ScrewRotateQNear)

    # 控制机械臂旋转至拍照方向
    duco.DucoMoveJ(PhotoRotateQNear,vel_joint*2,acc_joint)

    CircleTargetSaftyPos = [CircleTargetPos[0],CircleTargetPos[1],PhotoSaftyPos[2],CircleTargetPos[3],CircleTargetPos[4],CircleTargetPos[5]]
    CircleTargetSaftyQNear = PhotoSaftyQNear
    
    # 控制机械臂移动至螺纹孔上方安全位置
    duco.DucoMoveL(CircleTargetSaftyPos,vel_move,acc_move,CircleTargetSaftyQNear)

    duco.DucoMoveL(CircleTargetPos,vel_move,acc_move,CircleTargetQNear)

    # 控制机械臂下降至螺纹孔
    DownPos = [CircleTargetPos[0],CircleTargetPos[1],CircleTargetPos[2]-DownDeepth,CircleTargetPos[3],CircleTargetPos[4],CircleTargetPos[5]]
    duco.DucoMoveL(DownPos,vel_end,acc_end,CircleTargetQNear)

    time.sleep(1)

    # 松开夹爪
    danikor.ClawCtrl(1)

    # 模组下降
    danikor.DriverCtrl(1)

    # 关闭真空阀
    danikor.VacuumCtrl(0)
    time.sleep(1)

    danikor.ScrewMotorCtrl()
    time.sleep(3)

    # 模组收回
    danikor.InitialAllMould()

    
    duco.DucoMoveL(CircleTargetPos,vel_end,acc_end,CircleTargetQNear)

    # 控制机械臂移动至螺纹孔上方安全位置
    duco.DucoMoveL(CircleTargetSaftyPos,vel_move,acc_move,CircleTargetSaftyQNear)

    # 机械臂来到拍照可进行第一轴旋转的位置
    duco.DucoMoveL(PhotoRotatePos,vel_move,acc_move,PhotoRotateQNear)

    # # 螺钉正上方安全位置 
    # ScrewUpPos = [ScrewPose_1[0],ScrewPose_1[1],PhotoSaftyPos[2],ScrewPose_1[3],ScrewPose_1[4],ScrewPose_1[5]]
    # ScrewUpQNear = ScrewQNear
    # # 控制机械臂移动至螺钉上方安全位置
    # duco.DucoMoveL(ScrewUpPos,vel_move,acc_move,ScrewUpQNear)
    # time.sleep(1)

    # # 电批模组初始化
    # danikor.InitialAllMould()

    

    # duco.DucoMove(ScrewPose_1,vel_move,acc_move,ScrewQNear)
    # time.sleep(1)

    # # 开启真空阀
    # danikor.VacuumCtrl(1)

    # duco.DucoMove(ScrewPose_2,vel_end,acc_end,ScrewQNear)
    # time.sleep(5)


    # danikor.ClawCtrl(0)
    # time.sleep(1)

    # # 控制机械臂移动至上方
    # duco.DucoMove(ScrewUpPos,vel_move,acc_move,ScrewUpQNear)
    # time.sleep(1)

    # # 控制机械臂来到螺纹孔上方
    # duco.DucoMove(CircleTargetPos,vel_move,acc_move,CircleTargetQNear)
    # time.sleep(1)

    # # 控制机械臂下降至螺纹孔
    # DownPos_1 = [CircleTargetPos[0],CircleTargetPos[1],CircleTargetPos[2]-DownDeepth_1,CircleTargetPos[3],CircleTargetPos[4],CircleTargetPos[5]]
    # duco.DucoMove(DownPos_1,vel_move,acc_move,CircleTargetQNear)

    # DownPos_2 = [CircleTargetPos[0],CircleTargetPos[1],CircleTargetPos[2]-DownDeepth_2,CircleTargetPos[3],CircleTargetPos[4],CircleTargetPos[5]]
    # duco.DucoMove(DownPos_2,vel_end,acc_end,CircleTargetQNear)

    # time.sleep(1)

    # # 松开夹爪
    # danikor.ClawCtrl(1)

    # # 模组下降
    # danikor.DriverCtrl(1)

    # # 关闭真空阀
    # danikor.VacuumCtrl(0)

    
    # danikor.ScrewMotorCtrl()
    # time.sleep(3)

    # # 控制机械臂来到螺纹孔上方
    # duco.DucoMove(CircleTargetPos,vel_move,acc_move,CircleTargetQNear)


def HikTest():
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIp = "192.168.1.101"
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
    RotEndToCamera = np.array([[ 0, 1, 0],
                               [ -1, 0, 0],
                               [ 0, 0, 1]])
    
    # 示教参数-电批相对相机偏移，相机坐标系下，单位：mm
    # dx = -297
    # dy = 3.59

    # dx = - 291.16
    # dy =  18.22

    dx = -296.20
    dy = 4.97
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7003

    # 机械臂末端的速度
    vel = 0.3
    acc = 0.15

    # 起始拍照位的安全高度位置
    PhotoSaftyPos = [0.6574935913085938, 1.1234687566757202, 0.8394834399223328, 3.141319990158081, -0.00022508660913445055, 0.06932102143764496]
    PhotoSaftyQNear = [0.906836986541748, 0.42894959449768066, 1.2343454360961914, -0.08622220158576965, -1.570582628250122, -0.7268856167793274]

    # 起始拍照的位姿
    PhotoPos = [0.6574935913085938, 1.1234687566757202, 0.41623711585998535, 3.1413567066192627, -0.0002582818269729614, 0.06934092938899994]
    PhotoQNear = [0.9115228056907654, 0.5976395606994629, 1.5408779382705688, -0.5614805817604065, -1.5708342790603638, -0.7217084169387817]

    ScrewPose = [0.3804687261581421, 0.6290889978408813, 0.5466667413711548, -3.1415727138519287, -4.217711193632567e-06, -1.028997667162912e-05]
    ScrewQNear = [0.7961986064910889, -0.07836660742759705, 2.245490312576294, -0.5900270342826843, -1.5702589750289917, -0.7675241231918335]
    
    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.103'
    DanikorPort = 8888


    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)

    # 机械臂来到拍照安全高度位置
    duco.DucoMoveL(PhotoSaftyPos,vel,acc,PhotoSaftyQNear)
    
    # 机械臂来到拍照位置
    duco.DucoMoveL(PhotoPos,vel,acc,PhotoQNear)
    # 给海康相机发出切换方案指令
    hik.SetHikSwitchPlan('switch','CircleDetect1')

    # 发出检测信号并记录数据
    DetectedData = hik.GetDataFromHik(msgStart,9)

    # if DetectedData != 0:
    #     print("相机检测成功！！！")
    # else:
    #     print("相机检测失败！！！")
    #     return
    
    # 检测控制盒的宽度，用于计算比例尺，单位: 像素
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
    
    CircleTargetPos = hik.GetTargetPos(Width,Height,dx,dy,RotEndToCamera,Ruler,PhotoPos,CirclePos[0])
    CircleTargetQNear = PhotoQNear

    CircleTargetSaftyPos = [CircleTargetPos[0],CircleTargetPos[1],PhotoSaftyPos[2],CircleTargetPos[3],CircleTargetPos[4],CircleTargetPos[5]]
    CircleTargetSaftyQNear = PhotoSaftyQNear

    duco.DucoMoveL(CircleTargetSaftyPos,vel,acc,CircleTargetSaftyQNear)

    duco.DucoMoveL(CircleTargetPos,vel,acc,CircleTargetQNear)
    
def OnlyMove_ClosePart_1():
    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    # 机械臂末端的速度
    vel_move = 0.5
    acc_move = 0.3

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993

    #------------------------示教位置---------------------------
    # 吸钉区域
    # 参考六轴
    QNearGetScrew = [-1.6864653825759888, 0.32770687341690063, 2.014782190322876, 0.7951695919036865, 1.4564528465270996, -1.575836181640625]
    
    # 吸钉位置
    PosGetScrew = [0.08444300293922424, -0.9816458821296692, 0.3237221694946289, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    
    # 吸钉减速位置
    PosGetScrewSlowDown = [0.08444300293922424, -0.9816458821296692, 0.3653124272823334, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    
    # 吸稳位置
    PosGetScrewDownAgain = [0.08444300293922424, -0.9816458821296692, 0.31965842843055725, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    
    # 吸钉退出位置
    PosGetScrewLeave = [0.055,-0.955,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测过渡位置
    PosGetScrewGoToConfirm = [0.055,-0.7258163094520569,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测位置:
    PosGetScrewConfirm = [0.28018778562545776, -0.7258163094520569, 0.3547946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉检测位置退出:
    PosGetScrewConfirmLeave = [0.28018778562545776, -0.7258163094520569, 0.4047946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉方向就绪位置
    PosGetScrewStandby = [0.055,-0.955,0.8128352165222168,3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    # PosGetScrewSafeArea = [0.14271675050258636, -1.0591011047363281, 0.7753543853759766, -1.5495918989181519, 1.5690542459487915, -3.1204283237457275]
    
    PosGetScrewMoveJ = [0.06975271552801132, -0.48145827651023865, 0.8128352165222168, 3.1363139152526855, 1.569033145904541, 1.5655303001403809]
    QNearGetScrewMoveJ = [-1.8543643951416016, -0.5473452806472778, 2.1644771099090576, 1.5211260318756104, 1.287714958190918, -1.5744819641113281]



    # 拧钉区域
    # 拧钉方向第一轴可动位置:
    PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
    QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]

    # 拧钉参考六轴
    QNearTwistScrew = [1.5505576133728027, 0.8574095964431763, 2.8965694904327393, -0.6185015439987183, 1.5518832206726074, -0.007149023003876209]
    # 拧钉左侧就绪位置:
    PosTwistScrewStandby = [-0.16429604589939117, 0.33377784490585327, 0.03015068918466568, 1.5707684755325317, 1.137180151999928e-05, -3.141587972640991]
    # 拧钉孔位减速位置:
    PosTwistScrewSlowDown = [-0.6097485733032227, 0.3337692320346832, 0.09015525877475739, 1.5708067417144775, 2.227753611805383e-05, 3.141575336456299]
    # 拧钉孔位进入位置:
    PosTwistScrew = [-0.6397485733032227, 0.3337847888469696, 0.09016480296850204, 1.5707998275756836, -5.1200686357333325e-06, 3.1415693759918213]



    # QNearTwistScrew = [2.2653207778930664, 0.7222155928611755, 2.1488735675811768, 0.25711384415626526, 2.2669219970703125, -0.013344867154955864]
    # PosTwistScrew = [-0.8056106567382812, 0.6195728778839111, -7.832080882508308e-06, 1.5707844495773315, -3.4144502478739014e-06, -3.1415765285491943]
    # PosTwistScrewSlowDown = [-0.7744351029396057, 0.6195857524871826, 3.364891017554328e-05, 1.5707625150680542, -3.788686444750056e-05, 3.1415858268737793]
    # PosTwistScrewSafeArea = [-0.17, 0.6195773482322693, 3.354552973178215e-05, 1.5707614421844482, -2.8890166504424997e-05, -3.1415791511535645]
    # QNearTwistScrewMoveJ_0 = [1.5743942260742188, -0.31151971220970154, 2.08732271194458, 1.359482765197754, 1.5750846862792969, -0.004919957369565964]
    # QNearTwistScrewMoveJ_1 = [1.574358344078064, -0.3114837408065796, 2.0872867107391357, 1.3595067262649536, 1.5751086473464966, -1.577489972114563]

    # 协作臂回到默认位置（吸钉区域安全位置）
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

    danikor.ClawCtrl(1)

    # 协作臂快速来到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,QNearGetScrew)

    # 协作臂慢速来到吸钉位置
    duco.DucoMoveL(PosGetScrew,vel_end,acc_end,QNearGetScrew)

    # 控制电批反转认帽
    danikor.ScrewMotorCtrl(2)
    time.sleep(1)

    # 第二次深入吸稳位置
    duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,QNearGetScrew)

    # 夹具夹紧
    danikor.ClawCtrl(0)

    # 协作臂慢速回到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,QNearGetScrew)

    # 协作臂快速来到吸钉退出位置
    duco.DucoMoveL(PosGetScrewLeave,vel_end,acc_end,QNearGetScrew)
    
    # 协作臂快速来到检测过渡位置
    duco.DucoMoveL(PosGetScrewGoToConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到吸钉检测位置
    duco.DucoMoveL(PosGetScrewConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到检测退出位置
    duco.DucoMoveL(PosGetScrewConfirmLeave,vel_move,acc_move,QNearGetScrew)

    # 协作臂快速来到吸钉方向第一轴可动位置
    duco.DucoMoveL(PosGetScrewMoveJ,vel_move,acc_move,QNearGetScrewMoveJ)
    
    #----------------------------------------------------------------------

    # 协作臂转动来到拧钉方向可旋转位置
    duco.DucoMoveJ(QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂快速来到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,QNearTwistScrew)

    # 协作臂慢速来到拧钉位置
    duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,QNearTwistScrew)

    # 夹具松开
    danikor.ClawCtrl(1)

    # 控制电批拧入螺钉
    danikor.ScrewMotorCtrl(1)
    time.sleep(10)

    # 协作臂慢速回到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,QNearTwistScrew)

    # 协作臂快速回到拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂快速轴动到轴动位置
    duco.DucoMoveL(PosTwistScrewMoveJ,vel_move,acc_move,QNearTwistScrewMoveJ)

    #---------------------------------------------------------------------------
    # 协作臂轴动转回至取钉轴动区域
    duco.DucoMoveJ(QNearGetScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到吸钉安全位置
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

def OnlyMove_ClosePart_2():
    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    # 机械臂末端的速度
    vel_move = 0.5
    acc_move = 0.3

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993

    #------------------------示教位置---------------------------
    # 吸钉区域
    # 参考六轴
    QNearGetScrew = [-1.6864653825759888, 0.32770687341690063, 2.014782190322876, 0.7951695919036865, 1.4564528465270996, -1.575836181640625]
    
    # 吸钉位置
    PosGetScrew = [0.03456265479326248, -0.9816403388977051, 0.3241221694946289, 3.134039878845215, 1.5690678358078003, 1.5632350444793701]
    
    # 吸钉减速位置
    PosGetScrewSlowDown = [0.03456265479326248, -0.9816403388977051, 0.3653124272823334, 3.134039878845215, 1.5690678358078003, 1.5632350444793701]
    
    # 吸稳位置
    PosGetScrewDownAgain = [0.03456265479326248, -0.9816403388977051, 0.31965842843055725, 3.134039878845215, 1.5690678358078003, 1.5632350444793701]
    
    # 吸钉退出位置
    PosGetScrewLeave = [0.055,-0.955,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测过渡位置
    PosGetScrewGoToConfirm = [0.055,-0.7258163094520569,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测位置:
    PosGetScrewConfirm = [0.28018778562545776, -0.7258163094520569, 0.3547946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉检测位置退出:
    PosGetScrewConfirmLeave = [0.28018778562545776, -0.7258163094520569, 0.4047946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉方向就绪位置
    PosGetScrewStandby = [0.055,-0.955,0.8128352165222168,3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    # PosGetScrewSafeArea = [0.14271675050258636, -1.0591011047363281, 0.7753543853759766, -1.5495918989181519, 1.5690542459487915, -3.1204283237457275]
    
    PosGetScrewMoveJ = [0.06975271552801132, -0.48145827651023865, 0.8128352165222168, 3.1363139152526855, 1.569033145904541, 1.5655303001403809]
    QNearGetScrewMoveJ = [-1.8543643951416016, -0.5473452806472778, 2.1644771099090576, 1.5211260318756104, 1.287714958190918, -1.5744819641113281]



    # 拧钉区域
    # 拧钉方向第一轴可动位置:
    PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
    QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]

    # 拧钉参考六轴
    QNearTwistScrew = [1.5505576133728027, 0.8574095964431763, 2.8965694904327393, -0.6185015439987183, 1.5518832206726074, -0.007149023003876209]
    # 拧钉左侧就绪位置:
    PosTwistScrewStandby = [-0.16429604589939117, 0.33377784490585327, 0.03015068918466568, 1.5707684755325317, 1.137180151999928e-05, -3.141587972640991]
    # 拧钉孔位减速位置:
    PosTwistScrewSlowDown = [-0.6097485733032227, 0.3330232799053192, -0.033347453248500824, 1.5713677406311035, 0.0009110721875913441, 3.1414732933044434]
    # 拧钉孔位进入位置:
    PosTwistScrew = [-0.6397485733032227, 0.3330232799053192, -0.032647453248500824, 1.5713677406311035, 0.0009110721875913441, 3.1414732933044434]



    # QNearTwistScrew = [2.2653207778930664, 0.7222155928611755, 2.1488735675811768, 0.25711384415626526, 2.2669219970703125, -0.013344867154955864]
    # PosTwistScrew = [-0.8056106567382812, 0.6195728778839111, -7.832080882508308e-06, 1.5707844495773315, -3.4144502478739014e-06, -3.1415765285491943]
    # PosTwistScrewSlowDown = [-0.7744351029396057, 0.6195857524871826, 3.364891017554328e-05, 1.5707625150680542, -3.788686444750056e-05, 3.1415858268737793]
    # PosTwistScrewSafeArea = [-0.17, 0.6195773482322693, 3.354552973178215e-05, 1.5707614421844482, -2.8890166504424997e-05, -3.1415791511535645]
    # QNearTwistScrewMoveJ_0 = [1.5743942260742188, -0.31151971220970154, 2.08732271194458, 1.359482765197754, 1.5750846862792969, -0.004919957369565964]
    # QNearTwistScrewMoveJ_1 = [1.574358344078064, -0.3114837408065796, 2.0872867107391357, 1.3595067262649536, 1.5751086473464966, -1.577489972114563]

    # 协作臂回到默认位置（吸钉区域安全位置）
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

    danikor.ClawCtrl(1)
    # 协作臂快速来到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,QNearGetScrew)

    # 协作臂慢速来到吸钉位置
    duco.DucoMoveL(PosGetScrew,vel_end,acc_end,QNearGetScrew)

    # 控制电批反转认帽
    danikor.ScrewMotorCtrl(2)
    time.sleep(1)

    # 第二次深入吸稳位置
    duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,QNearGetScrew)

    # 夹具夹紧
    danikor.ClawCtrl(0)

    # 协作臂慢速回到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,QNearGetScrew)

    # 协作臂来到吸钉退出位置
    duco.DucoMoveL(PosGetScrewLeave,vel_end,acc_end,QNearGetScrew)
    
    # 协作臂快速来到检测过渡位置
    duco.DucoMoveL(PosGetScrewGoToConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到吸钉检测位置
    duco.DucoMoveL(PosGetScrewConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到检测退出位置
    duco.DucoMoveL(PosGetScrewConfirmLeave,vel_move,acc_move,QNearGetScrew)

    # 协作臂快速来到吸钉方向第一轴可动位置
    duco.DucoMoveL(PosGetScrewMoveJ,vel_move,acc_move,QNearGetScrewMoveJ)
    
    #----------------------------------------------------------------------

    # 协作臂转动来到拧钉方向可旋转位置
    duco.DucoMoveJ(QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂快速来到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,QNearTwistScrew)

    # 协作臂慢速来到拧钉位置
    duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,QNearTwistScrew)

    # 夹具张开
    danikor.ClawCtrl(1)

    # 控制电批拧入螺钉
    danikor.ScrewMotorCtrl(1)
    time.sleep(10)

    # 协作臂慢速回到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,QNearTwistScrew)

    # 协作臂快速回到拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂快速轴动到轴动位置
    duco.DucoMoveL(PosTwistScrewMoveJ,vel_move,acc_move,QNearTwistScrewMoveJ)

    #---------------------------------------------------------------------------
    # 协作臂轴动转回至取钉轴动区域
    duco.DucoMoveJ(QNearGetScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到吸钉安全位置
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

def OnlyMove_FarPart_1():
#------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.10"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    # 机械臂末端的速度
    vel_move = 0.5
    acc_move = 0.3

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993

    #------------------------示教位置---------------------------
    # 吸钉区域
    # 参考六轴
    QNearGetScrew = [-1.6864653825759888, 0.32770687341690063, 2.014782190322876, 0.7951695919036865, 1.4564528465270996, -1.575836181640625]
    
    # 吸钉位置
    PosGetScrew = [0.03533635661005974, -0.9308379292488098, 0.3231221694946289, 3.133913516998291, 1.5691132545471191, 1.563124418258667]
    
    # 吸钉减速位置
    PosGetScrewSlowDown = [0.03533635661005974, -0.9308379292488098, 0.3653124272823334, 3.133913516998291, 1.5691132545471191, 1.563124418258667]
    
    # 吸稳位置
    PosGetScrewDownAgain = [0.03533635661005974, -0.9308379292488098, 0.31965842843055725, 3.133913516998291, 1.5691132545471191, 1.563124418258667]
    
    # 吸钉退出位置
    PosGetScrewLeave = [0.055,-0.955,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测过渡位置
    PosGetScrewGoToConfirm = [0.055,-0.7258163094520569,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测位置:
    PosGetScrewConfirm = [0.28018778562545776, -0.7258163094520569, 0.3547946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉检测位置退出:
    PosGetScrewConfirmLeave = [0.28018778562545776, -0.7258163094520569, 0.4047946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉方向就绪位置
    PosGetScrewStandby = [0.055,-0.955,0.8128352165222168,3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    # PosGetScrewSafeArea = [0.14271675050258636, -1.0591011047363281, 0.7753543853759766, -1.5495918989181519, 1.5690542459487915, -3.1204283237457275]
    
    PosGetScrewMoveJ = [0.06975271552801132, -0.48145827651023865, 0.8128352165222168, 3.1363139152526855, 1.569033145904541, 1.5655303001403809]
    QNearGetScrewMoveJ = [-1.8543643951416016, -0.5473452806472778, 2.1644771099090576, 1.5211260318756104, 1.287714958190918, -1.5744819641113281]



    # 拧钉区域
    # 拧钉方向第一轴可动位置:
    PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
    QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]

    # 拧钉参考六轴
    QNearTwistScrew = [1.565573811531067, 0.6790363788604736, 2.088568925857544, 0.36772826313972473, 1.5671511888504028, -0.007041165139526129]


    # 正常姿态下可以进行扭转位置:
    PosTwistScrewNormalStandby = [-0.16424693167209625, 0.9588245749473572, 0.050218433141708374, 1.5707995891571045, 0.0009492189856246114, 3.141490936279297]

    # 正常状态扭转至右侧拧钉状态中的过渡位置
    PosTwistScrewNormalStandbyToTarget = [-0.16425147652626038, 0.9588478803634644, 0.05021889507770538, 3.1415793895721436, 0.0009572505950927734, 3.1414880752563477]

    # 拧钉右侧就绪位置:
    PosTwistScrewStandby = [-0.16427645087242126, 0.9588826298713684, 0.05017045885324478, -1.5700844526290894, 0.001079818233847618, 3.1415560245513916]

    # 拧钉孔位减速位置:
    PosTwistScrewSlowDown = [-0.6097485733032227, 0.9588111639022827, -0.01189726684242487, -1.5702458620071411, 0.0009518721490167081, -3.141524076461792]

    # 拧钉孔位进入位置:
    PosTwistScrew = [-0.6397485733032227, 0.9588111639022827, -0.01189726684242487, -1.5702458620071411, 0.0009518721490167081, -3.141524076461792]



    # QNearTwistScrew = [2.2653207778930664, 0.7222155928611755, 2.1488735675811768, 0.25711384415626526, 2.2669219970703125, -0.013344867154955864]
    # PosTwistScrew = [-0.8056106567382812, 0.6195728778839111, -7.832080882508308e-06, 1.5707844495773315, -3.4144502478739014e-06, -3.1415765285491943]
    # PosTwistScrewSlowDown = [-0.7744351029396057, 0.6195857524871826, 3.364891017554328e-05, 1.5707625150680542, -3.788686444750056e-05, 3.1415858268737793]
    # PosTwistScrewSafeArea = [-0.17, 0.6195773482322693, 3.354552973178215e-05, 1.5707614421844482, -2.8890166504424997e-05, -3.1415791511535645]
    # QNearTwistScrewMoveJ_0 = [1.5743942260742188, -0.31151971220970154, 2.08732271194458, 1.359482765197754, 1.5750846862792969, -0.004919957369565964]
    # QNearTwistScrewMoveJ_1 = [1.574358344078064, -0.3114837408065796, 2.0872867107391357, 1.3595067262649536, 1.5751086473464966, -1.577489972114563]

    # 协作臂回到默认位置（吸钉区域安全位置）
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

    danikor.ClawCtrl(1)

    # 协作臂快速来到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,QNearGetScrew)

    # 协作臂慢速来到吸钉位置
    duco.DucoMoveL(PosGetScrew,vel_end,acc_end,QNearGetScrew)

    # 控制电批反转认帽
    danikor.ScrewMotorCtrl(2)
    time.sleep(1)

    # 第二次深入吸稳位置
    duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,QNearGetScrew)

    danikor.ClawCtrl(0)

    # 协作臂慢速回到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,QNearGetScrew)

    # 协作臂来到吸钉退出位置
    duco.DucoMoveL(PosGetScrewLeave,vel_end,acc_end,QNearGetScrew)
    
    # 协作臂快速来到检测过渡位置
    duco.DucoMoveL(PosGetScrewGoToConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到吸钉检测位置
    duco.DucoMoveL(PosGetScrewConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到检测退出位置
    duco.DucoMoveL(PosGetScrewConfirmLeave,vel_move,acc_move,QNearGetScrew)

    # 协作臂快速来到吸钉方向第一轴可动位置
    duco.DucoMoveL(PosGetScrewMoveJ,vel_move,acc_move,QNearGetScrewMoveJ)
    
    #----------------------------------------------------------------------

    # 协作臂转动来到拧钉方向可旋转位置
    duco.DucoMoveJ(QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到扭转就绪位置
    duco.DucoMoveL(PosTwistScrewNormalStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂来到扭转过渡位置
    duco.DucoMoveL(PosTwistScrewNormalStandbyToTarget,vel_end,acc_end,QNearTwistScrew)
    
    # 协作臂来到右侧拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_end,acc_end,QNearTwistScrew)
 
    # 协作臂快速来到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,QNearTwistScrew)

    # 协作臂慢速来到拧钉位置
    duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,QNearTwistScrew)

    danikor.ClawCtrl(1)

    # 控制电批拧入螺钉
    danikor.ScrewMotorCtrl(1)
    time.sleep(10)

    # 协作臂慢速回到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,QNearTwistScrew)

    # 协作臂快速回到右侧拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂来到扭转过渡位置
    duco.DucoMoveL(PosTwistScrewNormalStandbyToTarget,vel_end,acc_end,QNearTwistScrew)

    # 协作臂来到扭转就绪位置
    duco.DucoMoveL(PosTwistScrewNormalStandby,vel_end,acc_end,QNearTwistScrew)
 
    # 协作臂快速移动到轴动位置
    duco.DucoMoveL(PosTwistScrewMoveJ,vel_move,acc_move,QNearTwistScrewMoveJ)

    #---------------------------------------------------------------------------
    # 协作臂轴动转回至取钉轴动区域
    duco.DucoMoveJ(QNearGetScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到吸钉安全位置
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

def OnlyMove_FarPart_2():
#------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    duco = DucoCtrl(DucoIp,DucoPort)
    # 机械臂末端的速度
    vel_move = 0.5
    acc_move = 0.3

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993

    #------------------------示教位置---------------------------
    # 吸钉区域
    # 参考六轴
    QNearGetScrew = [-1.6864653825759888, 0.32770687341690063, 2.014782190322876, 0.7951695919036865, 1.4564528465270996, -1.575836181640625]
    
    # 吸钉位置
    PosGetScrew = [0.08566054701805115, -0.9311874508857727, 0.3231221694946289, 3.1291863918304443, 1.5690675973892212, 1.558384895324707]
    
    # 吸钉减速位置
    PosGetScrewSlowDown = [0.08566054701805115, -0.9311874508857727, 0.3653124272823334, 3.1291863918304443, 1.5690675973892212, 1.558384895324707]
    
    # 吸稳位置
    PosGetScrewDownAgain = [0.08566054701805115, -0.9311874508857727, 0.31965842843055725, 3.1291863918304443, 1.5690675973892212, 1.558384895324707]
    
    # 吸钉退出位置
    PosGetScrewLeave = [0.055,-0.955,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测过渡位置
    PosGetScrewGoToConfirm = [0.055,-0.7258163094520569,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

    # 吸钉检测位置:
    PosGetScrewConfirm = [0.28018778562545776, -0.7258163094520569, 0.3547946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉检测位置退出:
    PosGetScrewConfirmLeave = [0.28018778562545776, -0.7258163094520569, 0.4047946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

    # 吸钉方向就绪位置
    PosGetScrewStandby = [0.055,-0.955,0.8128352165222168,3.134021759033203, 1.5690670013427734, 1.5631933212280273]
    # PosGetScrewSafeArea = [0.14271675050258636, -1.0591011047363281, 0.7753543853759766, -1.5495918989181519, 1.5690542459487915, -3.1204283237457275]
    
    PosGetScrewMoveJ = [0.06975271552801132, -0.48145827651023865, 0.8128352165222168, 3.1363139152526855, 1.569033145904541, 1.5655303001403809]
    QNearGetScrewMoveJ = [-1.8543643951416016, -0.5473452806472778, 2.1644771099090576, 1.5211260318756104, 1.287714958190918, -1.5744819641113281]



    # 拧钉区域
    # 拧钉方向第一轴可动位置:
    PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
    QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]

    # 拧钉参考六轴
    QNearTwistScrew = [1.565573811531067, 0.6790363788604736, 2.088568925857544, 0.36772826313972473, 1.5671511888504028, -0.007041165139526129]


    # 正常姿态下可以进行扭转位置:
    PosTwistScrewNormalStandby = [-0.16424693167209625, 0.9588245749473572, 0.050218433141708374, 1.5707995891571045, 0.0009492189856246114, 3.141490936279297]

    # 正常状态扭转至右侧拧钉状态中的过渡位置
    PosTwistScrewNormalStandbyToTarget = [-0.16425147652626038, 0.9588478803634644, 0.05021889507770538, 3.1415793895721436, 0.0009572505950927734, 3.1414880752563477]

    # 拧钉右侧就绪位置:
    PosTwistScrewStandby = [-0.16427645087242126, 0.9588826298713684, 0.05017045885324478, -1.5700844526290894, 0.001079818233847618, 3.1415560245513916]

    # 拧钉孔位减速位置:
    PosTwistScrewSlowDown = [-0.6097485733032227, 0.9588069915771484, 0.11019549518823624, -1.5702608823776245, 0.0009298138902522624, -3.1414954662323]

    # 拧钉孔位进入位置:
    PosTwistScrew = [-0.6397485733032227, 0.9588069915771484, 0.11019549518823624, -1.5702608823776245, 0.0009298138902522624, -3.1414954662323]



    # QNearTwistScrew = [2.2653207778930664, 0.7222155928611755, 2.1488735675811768, 0.25711384415626526, 2.2669219970703125, -0.013344867154955864]
    # PosTwistScrew = [-0.8056106567382812, 0.6195728778839111, -7.832080882508308e-06, 1.5707844495773315, -3.4144502478739014e-06, -3.1415765285491943]
    # PosTwistScrewSlowDown = [-0.7744351029396057, 0.6195857524871826, 3.364891017554328e-05, 1.5707625150680542, -3.788686444750056e-05, 3.1415858268737793]
    # PosTwistScrewSafeArea = [-0.17, 0.6195773482322693, 3.354552973178215e-05, 1.5707614421844482, -2.8890166504424997e-05, -3.1415791511535645]
    # QNearTwistScrewMoveJ_0 = [1.5743942260742188, -0.31151971220970154, 2.08732271194458, 1.359482765197754, 1.5750846862792969, -0.004919957369565964]
    # QNearTwistScrewMoveJ_1 = [1.574358344078064, -0.3114837408065796, 2.0872867107391357, 1.3595067262649536, 1.5751086473464966, -1.577489972114563]

    # 协作臂回到默认位置（吸钉区域安全位置）
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)

    danikor.ClawCtrl(1)

    # 协作臂快速来到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,QNearGetScrew)

    # 协作臂慢速来到吸钉位置
    duco.DucoMoveL(PosGetScrew,vel_end,acc_end,QNearGetScrew)

    # 控制电批反转认帽
    danikor.ScrewMotorCtrl(2)
    time.sleep(1)

    # 第二次深入吸稳位置
    duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,QNearGetScrew)

    danikor.ClawCtrl(0)

    # 协作臂慢速回到吸钉减速位置
    duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,QNearGetScrew)

    # 协作臂来到吸钉退出位置
    duco.DucoMoveL(PosGetScrewLeave,vel_end,acc_end,QNearGetScrew)
    
    # 协作臂快速来到检测过渡位置
    duco.DucoMoveL(PosGetScrewGoToConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到吸钉检测位置
    duco.DucoMoveL(PosGetScrewConfirm,vel_move,acc_move,QNearGetScrew)

    # 协作臂来到检测退出位置
    duco.DucoMoveL(PosGetScrewConfirmLeave,vel_move,acc_move,QNearGetScrew)

    # 协作臂快速来到吸钉方向第一轴可动位置
    duco.DucoMoveL(PosGetScrewMoveJ,vel_move,acc_move,QNearGetScrewMoveJ)
    
    #----------------------------------------------------------------------

    # 协作臂转动来到拧钉方向可旋转位置
    duco.DucoMoveJ(QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到扭转就绪位置
    duco.DucoMoveL(PosTwistScrewNormalStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂来到扭转过渡位置
    duco.DucoMoveL(PosTwistScrewNormalStandbyToTarget,vel_end,acc_end,QNearTwistScrew)
    
    # 协作臂来到右侧拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_end,acc_end,QNearTwistScrew)
 
    # 协作臂快速来到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,QNearTwistScrew)

    # 协作臂慢速来到拧钉位置
    duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,QNearTwistScrew)

    danikor.ClawCtrl(1)

    # 控制电批拧入螺钉
    danikor.ScrewMotorCtrl(1)
    time.sleep(10)

    # 协作臂慢速回到拧钉减速位置
    duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,QNearTwistScrew)

    # 协作臂快速回到右侧拧钉就绪位置
    duco.DucoMoveL(PosTwistScrewStandby,vel_move,acc_move,QNearTwistScrew)

    # 协作臂来到扭转过渡位置
    duco.DucoMoveL(PosTwistScrewNormalStandbyToTarget,vel_end,acc_end,QNearTwistScrew)

    # 协作臂来到扭转就绪位置
    duco.DucoMoveL(PosTwistScrewNormalStandby,vel_end,acc_end,QNearTwistScrew)
 
    # 协作臂快速移动到轴动位置
    duco.DucoMoveL(PosTwistScrewMoveJ,vel_move,acc_move,QNearTwistScrewMoveJ)

    #---------------------------------------------------------------------------
    # 协作臂轴动转回至取钉轴动区域
    duco.DucoMoveJ(QNearGetScrewMoveJ,vel_joint*4,acc_joint)

    # 协作臂来到吸钉安全位置
    duco.DucoMoveL(PosGetScrewStandby,vel_move,acc_move,QNearGetScrew)



def StepMove():
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    # 机械臂末端的速度
    vel_move = 0.3
    acc_move = 0.1

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993
    # 电批以正常姿态对准惯导几何中心
    TargetPos_1 = [0.08558245748281479, -0.9865738749504089, 0.32388073205947876, 3.0768325328826904, 1.5638643503189087, 1.5060094594955444]
    TargetQNear_1 = [-1.6670390367507935, 0.38868260383605957, 2.02494478225708, 0.7291964292526245, 1.475939154624939, -1.576075792312622]

    TargetPos_2 = [-0.1136971265077591, -0.4934713840484619, 0.8395147323608398, 3.1413629055023193, -0.00023751870321575552, -3.1415765285491943]
    TargetQNear_2 = [4.147467136383057, -0.5267564058303833, 2.139753580093384, -0.03585250675678253, -1.5701152086257935, -0.5586271286010742]
    
    TargetQNear_3 = [1.574358344078064, -0.3114837408065796, 2.0872867107391357, 1.3595067262649536, 1.5751086473464966, -1.577489972114563]
    
    QNearTwistScrew = [2.2653207778930664, 0.7222155928611755, 2.1488735675811768, 0.25711384415626526, 2.2669219970703125, -0.013344867154955864]
    PosTwistScrew = [-0.8056106567382812, 0.6195728778839111, -7.832080882508308e-06, 1.5707844495773315, -3.4144502478739014e-06, -3.1415765285491943]


    duco = DucoCtrl(DucoIp,DucoPort)

    # duco.DucoMoveL(TargetPos_1,vel_move,acc_move,TargetQNear_1)

    duco.DucoMoveL(TargetPos_1,vel_move,acc_move,TargetQNear_1)


def DanikorTest():
    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888

    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)

    # danikor.InitialAllMould()

    # danikor.ClawCtrl(0)

    # danikor.ScrewConferm()

    # print(danikor.ScrewConferm())
    # danikor.ClawCtrl(1)

    # danikor.VacuumCtrl(1)

    # danikor.VacuumCtrl(0)

    # danikor.DriverCtrl(1)

    # danikor.DriverCtrl(0)

    danikor.ScrewMotorCtrl(2)

    # danikor.ScrewMotorCtrl(1)

def ProcessTest():
    ''' TCP 持续等待 Type'''


    

    process = StepProcess()

    GetScrew_1 = [0.08552581816911697, -0.9865832924842834, 0.32483397221565247, 3.0731868743896484, 1.5638149976730347, 1.502327799797058]
    GetScrew_2 = [0.03579577058553696, -0.9849782586097717, 0.3248028088092804, 3.075232744216919, 1.5638421773910522, 1.5044243335723877]
    GetScrew_3 = [0.037906792014837265, -0.9349860548973083, 0.324820525937080383, 3.075066089630127, 1.563859462738037, 1.504248857498169]
    GetScrew_4 = [0.08774872869253159, -0.9370216131210327, 0.3248088522052765, 3.0742757320404053, 1.5638936758041382, 1.5034552812576294]

    TwistScrew_1 = [-0.5797485733032227, 0.3337847888469696, 0.09016480296850204, 1.5707998275756836, -5.1200686357333325e-06, 3.1415693759918213]
    TwistScrew_2 = [-0.5797485733032227, 0.3330232799053192, -0.032647453248500824, 1.5713677406311035, 0.0009110721875913441, 3.1414732933044434]
    TwistScrew_3 = [-0.5797485733032227, 0.9588111639022827, -0.01189726684242487, -1.5702458620071411, 0.0009518721490167081, -3.141524076461792]
    TwistScrew_4 = [-0.5797485733032227, 0.9588069915771484, 0.11019549518823624, -1.5702608823776245, 0.0009298138902522624, -3.1414954662323]



    process.Screw_1(GetScrew_1,TwistScrew_1)
    time.sleep(0.5)
    process.Screw_2(GetScrew_3,TwistScrew_3)
    time.sleep(0.5)
    process.Screw_1(GetScrew_2,TwistScrew_2)
    time.sleep(0.5)
    process.Screw_2(GetScrew_4,TwistScrew_4)

def ShowPos():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    duco = DucoCtrl(DucoIp,DucoPort)
    # danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)

    print("当前姿态:",duco.GetDucoPos())
    print("当前六轴:",duco.GetQNear())



if __name__ == '__main__':

    # OnlyMove_ClosePart_1()
    # time.sleep(1)
    # OnlyMove_FarPart_1()
    # time.sleep(1)
    # OnlyMove_ClosePart_2()
    # time.sleep(1)
    # OnlyMove_FarPart_2()
    # OnlyMove_FarPart()
    # OnlyMove_FarPart()
    # OnlyMove()
    # StepMove()
    # ShowPos()
    # MainTest()
    # HikTest()
    # DanikorTest()
    ProcessTest()
