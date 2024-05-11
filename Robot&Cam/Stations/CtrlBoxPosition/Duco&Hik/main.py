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
    HikIp = "192.168.1.5"
    HikPort = 8192
    # 通讯触发信号
    msgStart = '123'

    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    result = hik.GetDataFromHik(msgStart)
    if result != 0:
        print (result)
    else:
        return 
    

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

    result = danikor.ScrewMotorCtrl(1)

    # print(float(result[0]))
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
    HikTest()
    # DanikorTest()
    # ProcessTest()
