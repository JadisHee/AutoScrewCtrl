import numpy as np
import time
import math

from SocketCtrl import XmlData

from DucoCtrl import DucoCtrl


from StepProcess import StepProcess

from CalcTools import CalcTools as T

ToolTransMat = np.array([[0.7986355,   0.6018150,  0.0000000,  0.07986],
                         [-0.6018150,  0.7986355,  0.0000000, -0.06018],
                         [0.0000000,   0.0000000,  1.0000000,   0.2035],
                         [        0,           0,          0,        1] ])

TcpVec = [0.07986, -0.06018, 0.2035, 0.0, 0.0, -0.6457718034476305]

# [1.1748631000518799, -0.3924994170665741, 0.4590481221675873, -2.9088878631591797, 0.17203548550605774, 0.9346023797988892]
TargetPos = [1.1180192232131958, -0.2919119596481323, 0.26395073533058167, -2.8531243801116943, -3.4653501643333584e-05, 1.5601879358291626]

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
    tool = T()

    PosFlange = duco.GetDucoPos(0)
    PosTool = duco.GetDucoPos(1)


    ToolPos = tool.PosTrans(PosFlange,ToolTransMat)
    FlangePos = tool.PosTrans(ToolPos,np.linalg.inv(ToolTransMat))

    print("当前法兰姿态: ", PosFlange)
    print("当前工具姿态: ", PosTool)
    print("变换工具姿态: ", ToolPos)
    print("变换法兰姿态: ", FlangePos)
    # print("当前姿态:",duco.GetDucoPos(0))
    # print("当前六轴:",duco.GetQNear())

def ProcessTest():
    process = StepProcess()
    process.GoToGetAntenna()

    process.TakeAntennaToConfirmPos()
    process.TakeTransferCamToPhotoPos()

    process.TakeAntennaToTarget(TargetPos,TcpVec)

    process.GoBackToDefault(TargetPos,TcpVec)

if __name__ == '__main__':

    ProcessTest()

    # StepMove()
    # ShowPos()

