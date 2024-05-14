import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess

ToolTransMat = np.array([[ 1, 0, 0, -0.00158513087673],
                         [ 0, 1, 0, 0.1672619871841232],
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
    TargetPos_1 = [-1.033828854560852, 0.019116833806037903, 0.062176916748285294, -3.1414084434509277, 0.00011105268640676513, 1.5708006620407104]
    TargetQNear_1 = [0.08642122894525528, -0.18896421790122986, -2.0365021228790283, -0.9155300259590149, -1.484237790107727, -3.1383559703826904]

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

    result = hik.GetDataFromHik(msgStart)
    if result != 0:
        print ("相机反馈结果: ",result)
        DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],5.4)
        
        
        # TargetPos = hik.PosTrans(PosVec,TransMat,DPos)
        print(DPos) 
    else:
        print ("请检查相机设置 ! ! !")
        return
    
    

    PosNow = duco.GetDucoPos(1)
    TargetPos = hik.GetTargetPos(DPos,PosNow,ToolTransMat)
    Qnear = duco.GetQNear()
    # TargetPos = [PosNow[0]+DPos[0],PosNow[1]+DPos[1],PosNow[2],PosNow[3],PosNow[4],PosNow[5]]
    print("当前位姿为: ",PosNow)
    print("目标位姿为: ",TargetPos)
    duco.DucoMovel(TargetPos,0.2,0.2,Qnear,'ElectricBit')


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