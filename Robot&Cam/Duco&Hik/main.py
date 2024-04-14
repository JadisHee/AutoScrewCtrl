import numpy as np
import time
import math

from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

def main():
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
    RotEndToCamera = np.array([[-1, 0, 0],
                               [ 0,-1, 0],
                               [ 0, 0, 1]])
    
    # 示教参数-电批相对相机偏移，相机坐标系下，单位：mm
    dx = 297
    dy = -3.59

    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.102"
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


    # 机械臂回到拍照位置
    duco.DucoMove(origin_photo_pos,vel,acc)

    # 获取机械臂拍完照后末端位姿
    PhotoPos =  duco.GetDucoPos()

    # 给海康相机发出切换方案指令
    hik.SetHikSwitchPlan('switch','CircleDetect1')

    # 发出检测信号并记录数据
    DetectedData = hik.GetDataFromHik(msgStart,9)

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

        # 螺钉正上方与机械臂拍照位等高的位置
        screw_up = [screw_pos[j][0], screw_pos[j][1], PhotoPos[2], math.pi, 0, math.pi]
        # 控制机械臂移动至上方
        duco.DucoMove(screw_up,vel,acc)
        time.sleep(1)

        # 初始化电批模组
        # 夹爪张开，吸钉关闭，模组收回
        danikor.InitialAllMould()

        # 控制机械臂向下移动至吸钉位置
        duco.DucoMove(screw_pos[j],vel,acc)

        # 开启真空阀
        danikor.VacuumCtrl(1)
        time.sleep(1)

        # 闭合夹爪
        danikor.ClawCtrl(0)
        time.sleep(1)

        # 控制机械臂回到该螺钉上方
        duco.DucoMove(screw_up,vel,acc)
        time.sleep(1)

        # 计算当前目标螺纹孔的位置
        circle_target = hik.GetTargetPos(Width,Height,dx,dy,RotEndToCamera,Ruler,PhotoPos,CirclePos[j])

        # 控制机械臂来到当前目标螺纹孔上方
        duco.DucoMove(circle_target,vel,acc)

        # 控制电批模组伸出
        IsOk =  danikor.DriverCtrl(1)

        # 当电批模组伸出到位时继续执行
        if IsOk == 2:
            return 
            
        # 最终拧钉位置
        target_pos = [circle_target[0], circle_target[1], circle_target[2] - DeepIn, circle_target[3], circle_target[4], circle_target[5]]
        
        # 控制机械臂下降至螺纹孔上方
        duco.DucoMove(target_pos,vel,acc)
        time.sleep(1)

        '''
        在此处添加是否可以开始拧钉的依据
        '''
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