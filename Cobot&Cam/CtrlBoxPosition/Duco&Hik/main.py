import numpy as np
import time
import math
import socket

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess

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
    # TargetPos_1 = [-0.08788929134607315, -1.0715396404266357, 0.2525094747543335, 3.130729913711548, -1.5690492391586304, -3.130744218826294]
    # -0.8568769097328186, 0.6354405283927917, 0.17336474359035492, -0.02823464572429657, -1.5306755304336548, -0.0005614075344055891
    # -0.8580797910690308, 0.6356502771377563, 0.14294105768203735, 1.5306857824325562, -0.0032004942186176777, -1.5995079278945923
    TargetQNear_1 = [1.105727195739746, -0.4878915250301361, 2.2834084033966064, 1.3415184020996094, 1.1065014600753784, -1.5730797052383423]
    duco = DucoCtrl(DucoIp,DucoPort)


    # duco.DucoRobot.tcp_move([0,0,1],vel_move,acc_move,r=0,tool='ElectricBit',block=False)
    # duco.DucoMoveL(TargetPos_1,vel_move,acc_move,TargetQNear_1)

    duco.DucoMovel(TargetPos_1,vel_move,acc_move,TargetQNear_1,'ElectricBit')


def ProcessTest():

    process = StepProcess()
    # process.GoGetScrew()
    INSTarget = [1029.2046/1000,-1127.1646/1000,899.21655/1000,179.62177*math.pi/180,1.2740866*math.pi/180,134.41792*math.pi/180]
    process.GoStep(4,INSTarget)


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

    

    StepMove()
    # ShowPos()
    # CaliMove()
    ProcessTest()
