import numpy as np
import time
import math

from SocketCtrl import XmlData

from DucoCtrl import DucoCtrl

# from StepProcess import StepProcess
from TestProcess import StepProcess
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools as T

# ToolTransMat = np.array([[0.7986355,   0.6018150,  0.0000000,  0.07986],
#                          [-0.6018150,  0.7986355,  0.0000000, -0.06018],
#                          [0.0000000,   0.0000000,  1.0000000,   0.2035],
#                          [        0,           0,          0,        1] ])

# TcpVec = [0.07991, -0.05942, 0.20326, 0.18*math.pi/180,-0.41*math.pi/180,54.45*math.pi/180]

# 0.07986, -0.06018, 0.2035, 0.0, 0.0, -0.6457718034476305

# TargetPos = [1.06185, -0.28989, 0.27735, 0.15*math.pi/180, 178.24*math.pi/180,179.99*math.pi/180]
# 1.0611684322357178, -0.2894531786441803, 0.27714699506759644, -3.118072748184204, -4.84597148897592e-05, 1.5950757265090942
def StepMove():
    #------------------------设置协作臂相关参数----------------------------
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    # 机械臂末端的速度
    vel_move = 0.3
    acc_move = 0.1

    vel_end = 0.02
    acc_end = 0.2

    vel_joint = 0.2617993
    acc_joint = 0.2617993
    # 电批以正常姿态对准惯导几何中心
    TargetPos = [-0.8928816318511963, -0.14681445062160492, 0.2584591805934906, -3.141585111618042, 4.035204619867727e-05, -2.2165772914886475]
    TargetQNear_1 = [3.117692232131958, 0.3941248059272766, 2.1879897117614746, -1.0102074146270752, 4.712048053741455, -2.5242996215820312]

    duco = DucoCtrl(DucoIp,DucoPort)

    # TargetPos[2] = TargetPos[2] - (-0.65)
    duco.DucoMoveL(TargetPos,vel_move,acc_move,TargetQNear_1)


def ShowPos():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)
    # tool = T()

    # PosFlange = duco.GetDucoPos(0)
    # PosTool = duco.GetDucoPos(1)


    # ToolPos = tool.PosTrans(PosFlange,ToolTransMat)
    # FlangePos = tool.PosTrans(ToolPos,np.linalg.inv(ToolTransMat))

    # print("当前法兰姿态: ", PosFlange)
    # print("当前工具姿态: ", PosTool)
    # print("变换工具姿态: ", ToolPos)
    # print("变换法兰姿态: ", FlangePos)
    print("当前姿态:",duco.GetDucoPos(0))
    print("当前六轴:",duco.GetQNear())

def TestProcess():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)

    # 迁移ip
    ip = '192.168.1.10'
    # 迁移端口
    port = 5700

    StartSignal = '110,122'

    

    transfer = TransferCtrl(ip,port)

    process = StepProcess()
    
    for i in range(100):
        process.GoToGetAntenna()
        
        duco.DucoRobot.set_standard_digital_out(1,1,True)

        process.TakeAntennaToGetTcp()
        
        TcpVec = transfer.GetDataFromTransfer(0,StartSignal)
        # print('TcpVec: ', TcpVec)
        if(TcpVec == 0):
            return

        process.TakeCamToGetTarget()
        
        PosFlange = duco.GetDucoPos(0)
        TargetPos = transfer.GetDataFromTransfer(1,StartSignal,PosNow=PosFlange)
        # print('TargetPos: ', TargetPos)
        if(TargetPos == 0):
            return
        
        process.TakeAntennaToTarget(TargetPos,TcpVec)
        
        # duco.DucoRobot.set_standard_digital_out(1,0,True)

        time.sleep(1)

        print("成功次数: ", i)

        process.GoBackToDefault(TargetPos,TcpVec)

        process.GoToGetAntenna()

        duco.DucoRobot.set_standard_digital_out(1,0,True)
        
        time.sleep(2)

        StepMove()

def ProcessGo():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)

    # 迁移ip
    ip = '192.168.1.10'
    # 迁移端口
    port = 5700

    StartSignal = '110,122'

    

    transfer = TransferCtrl(ip,port)

    process = StepProcess()
    
    # for i in range(100):
    process.GoToGetAntenna()
    
    duco.DucoRobot.set_standard_digital_out(1,1,True)

    process.TakeAntennaToGetTcp()
    
    TcpVec = transfer.GetDataFromTransfer(0,StartSignal)
    print('TcpVec: ', TcpVec)
    if(TcpVec == 0):
        return

    process.TakeCamToGetTarget()
    
    PosFlange = duco.GetDucoPos(0)
    TargetPos = transfer.GetDataFromTransfer(1,StartSignal,PosNow=PosFlange)
    print('TargetPos: ', TargetPos)
    if(TargetPos == 0):
        return
    
    process.TakeAntennaToTarget(TargetPos,TcpVec)
    
    duco.DucoRobot.set_standard_digital_out(1,0,True)

    time.sleep(2)

    # print("成功次数: ", i)

    process.GoBackToDefault(TargetPos,TcpVec)

    # process.GoToGetAntenna()

    # duco.DucoRobot.set_standard_digital_out(1,0,True)
    
    # time.sleep(2)

    # StepMove()

def TransferTest():
    # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)

    PosFlange = duco.GetDucoPos(0)
    # 迁移ip
    ip = '192.168.1.10'
    # 迁移端口
    port = 5700
    transfer = TransferCtrl(ip,port)

    StartSignal = '110,122'

    # data = transfer.GetDataFromTransfer(1,StartSignal,PosNow=PosFlange)
    data = transfer.GetDataFromTransfer(0,StartSignal)


    print(data)

if __name__ == '__main__':

    # ProcessGo()
    # TestProcess()
    StepMove()
    # ShowPos()
    # TransferTest()
