import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess
# 0.000237055
ToolTransMat = np.array([[ 1, 0, 0, -0.000437055],
                         [ 0, 1, 0, 0.16615849733351],
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
def GetHikDPos(diameter):
        '''
            * Function:     GetHikDPos
            * Description:  获取识别后圆心在相机中心坐标系的坐标,单位:mm
            * Inputs:
                            
            * Outputs:      
            * Returns:      
                                DPos: 坐标 list[dx,dy]
                                    dx:float
                                    dy:float
                                0: 相机出错
            * Notes:
        '''
        #-------------------------设置视觉相关参数-----------------------------
        # 海康相机的通讯地址
        HikIp = "192.168.1.3"
        HikPort = 8192

        #------------------------实例化类----------------------------
        hik = HikCtrl(HikIp,HikPort)
        result = hik.GetDataFromHik('123')
        if result != 0:
            # print ("相机反馈结果: ",result)
            DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],diameter)
            return DPos
        else:
            print ("请检查相机设置 ! ! !")
            return 0

def HikTest():

    
    process = StepProcess()
    #-------------------------设置视觉相关参数-----------------------------
    # 海康相机的通讯地址
    HikIp = "192.168.1.3"
    HikPort = 8192

    #------------------------实例化类----------------------------
    hik = HikCtrl(HikIp,HikPort)
    
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003
    # 电批模组的通讯地址
    duco = DucoCtrl(DucoIp,DucoPort)
    # hik.SetHikSwitchPlan('switch','FindCircle-M8-6Angle_PlanB')

    DPos_B = process.GetHikPlanB()
    

    IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch','FindCircle-M8-6Angle')
    if IsSwitchPlanSuccessful == 0:
        return 0
    
    # 将相机移动至目标中心，直到接近中心后退出循环
    
    DPos_A = GetHikDPos(32)
    TargetPos_A = process.MoveToCenter(1,DPos_A)

    TransMat = np.array([[ 1, 0, 0, -DPos_B[0]],
                         [ 0, 1, 0, 0.180-DPos_B[1]],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])

    TargetPos_B = hik.GetTargetPos(DPos_B,duco.GetDucoPos(1),TransMat)

    print("PlanA: \n",TargetPos_A)
    print("PlanB: \n",TargetPos_B)
    
    # return(DPos)

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
    TargetPos_1 = [-0.00032147308229468763, -0.7025035619735718, 0.299015611410141, 3.141592502593994, 1.9189330942026572e-06, -3.141587495803833]
    # TargetPos_1 = [-0.00032147308229468763, -0.7025035619735718, 0.299015611410141, 3.141592502593994, 1.9189330942026572e-06, -3.141587495803833]
    # -0.8568769097328186, 0.6354405283927917, 0.17336474359035492, -0.02823464572429657, -1.5306755304336548, -0.0005614075344055891
    # -0.8580797910690308, 0.6356502771377563, 0.14294105768203735, 1.5306857824325562, -0.0032004942186176777, -1.5995079278945923
    TargetQNear_1 = [1.105727195739746, -0.4878915250301361, 2.2834084033966064, 1.3415184020996094, 1.1065014600753784, -1.5730797052383423]
    duco = DucoCtrl(DucoIp,DucoPort)


    duco.DucoRobot.tcp_move([0,0,1],vel_move,acc_move,r=0,tool='ElectricBit',block=False)
    # duco.DucoMoveL(TargetPos_1,vel_move,acc_move,TargetQNear_1)

    # duco.DucoMovel(TargetPos_1,vel_move,acc_move,TargetQNear_1,'ElectricBit')


def DanikorTest():
    #------------------------设置电批相关参数----------------------------
    # 电批模组的通讯地址
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888

    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)

    danikor.InitialAllMould()

    # danikor.ClawCtrl(1)

    # danikor.ScrewConferm()

    # print(danikor.ScrewConferm())
    # danikor.ClawCtrl(1)

    # danikor.VacuumCtrl(0)

    # danikor.VacuumCtrl(0)

    # danikor.DriverCtrl(0)

    # danikor.DriverCtrl(0)

    # result = danikor.ScrewMotorCtrl(2)

    # print(float(result[0]))
    # danikor.ScrewMotorCtrl(1)

def ProcessTest():

    process = StepProcess()
    # process.GoGetScrew()
    INSTarget = [1029.2046/1000,-1127.1646/1000,899.21655/1000,179.62177*math.pi/180,1.2740866*math.pi/180,134.41792*math.pi/180]
    process.GoStep(1,INSTarget)

def Move2HikCenter():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.16"
    DucoPort = 7003
    duco = DucoCtrl(DucoIp,DucoPort)

    # 海康相机的通讯地址
    HikIp = "192.168.1.4"
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
        DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],11.70)
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
    PosVec = duco.GetDucoPos(0)
    Qnear = duco.GetQNear()

    # DPos = [0.1,0.2]

    # PosTransed = PosTrans(PosVec,TransPos)

    print("当前姿态: ",PosVec)
    print("当前六轴: ",Qnear)
    # print("电批姿态: ",PosTransed)
    # print(PosTransed)
    


if __name__ == '__main__':

    
    # OnlyMove()
    StepMove()
    # ShowPos()
    # MainTest()
    # HikTest()
    # DanikorTest()
    # ProcessTest()
    # Move2HikCenter()