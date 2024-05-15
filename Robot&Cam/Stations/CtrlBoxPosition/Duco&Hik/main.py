import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess

ToolTransMat = np.array([[ 1, 0, 0, -0.00160632],
                         [ 0, 1, 0, 0.0500718],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])

ToolTransMat_1 = np.array([[ 1, 0, 0, 0],
                         [ 0, 1, 0, 0.0500718],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])
UnitTransMat = np.array([[ 1, 0, 0, 0],
                         [ 0, 1, 0, 0],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])

def HikTest():
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIp = "192.168.1.5"
    HikPort = 8192
    # 通讯触发信号
    msgStart = '123'

    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)

    PosVec = duco.GetDucoPos()

    TransMat = np.array([[ 1, 0, 0,-0.08239],
                         [ 0, 0, 1, 0.41251],
                         [ 0,-1, 0, 0.11332],
                         [ 0, 0, 0, 1]])
    

    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    result = hik.GetDataFromHik(msgStart)
    if result != 0:
        print ("相机反馈结果: ",result)
        DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],5.4)
        
        
        # TargetPos = hik.PosTrans(PosVec,TransMat,DPos)
        
        print(DPos)    

        return DPos
    
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
    TargetPos_1 = [-0.9916632175445557, -0.07183022052049637, 0.1606786847114563, -3.141380786895752, 0.00010996363562298939, 1.5708098411560059]
    TargetQNear_1 = [0.2291293740272522, -0.22558800876140594, -1.989296317100525, -0.9265435338020325, -1.3415416479110718, -3.1383919715881348]
    duco = DucoCtrl(DucoIp,DucoPort)

    # duco.DucoMoveL(TargetPos_1,vel_move,acc_move,TargetQNear_1)

    duco.DucoMovel(TargetPos_1,vel_move,acc_move,TargetQNear_1,'ElectricBit')


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

    process.AutoDo()

def Move2HikCenter():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003
    duco = DucoCtrl(DucoIp,DucoPort)

    # 海康相机的通讯地址
    HikIp = "192.168.1.5"
    HikPort = 8192
    # 通讯触发信号
    msgStart = '123'
    hik = HikCtrl(HikIp,HikPort)

    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)


    result = hik.GetDataFromHik(msgStart)
    if result != 0:
        print ("相机反馈结果: ",result)
        DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],11.94)
        # DPos = [-result[1]/1000,-result[2]/1000]
        
        # TargetPos = hik.PosTrans(PosVec,TransMat,DPos)
        print(DPos) 
    else:
        print ("请检查相机设置 ! ! !")
        return
    
    

    PosNow = duco.GetDucoPos(1)
    TargetPosUP = PosNow.copy()
    TargetPos = hik.GetTargetPos(DPos,PosNow,ToolTransMat)
    Qnear = duco.GetQNear()
    # TargetPos = [PosNow[0]+DPos[0],PosNow[1]+DPos[1],PosNow[2],PosNow[3],PosNow[4],PosNow[5]]
    print("当前位姿为: ",PosNow)
    print("目标位姿为: ",TargetPos)
    duco.DucoMovel(TargetPos,0.3,0.3,Qnear,'ElectricBit')
    # TargetPos[2] = TargetPos[2] - 0.125
    # duco.DucoMovel(TargetPos,0.2,0.2,Qnear,'ElectricBit')
    
    # danikor.DriverCtrl(1)
    

def ShowPos():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888
    duco = DucoCtrl(DucoIp,DucoPort)
    # danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    PosVec = duco.GetDucoPos(1)
    Qnear = duco.GetQNear()

    # DPos = [0.1,0.2]

    # PosTransed = PosTrans(PosVec,TransPos)

    print("当前姿态: ",PosVec)
    print("当前六轴: ",Qnear)
    # print("电批姿态: ",PosTransed)
    # print(PosTransed)
    


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
    StepMove()
    # ShowPos()
    # MainTest()
    # HikTest()
    # DanikorTest()
    # ProcessTest()
    # Move2HikCenter()