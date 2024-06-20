import numpy as np
import time
import math
import csv

from SocketCtrl import XmlData
from DucoCtrl import DucoCtrl
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools

class StepProcess:

    def __init__(self):
        # Duco机械臂的通讯地址
        DucoIp = "192.168.1.47"
        DucoPort = 7003

        # 实例化Duco协作臂控制类
        self.duco = DucoCtrl(DucoIp,DucoPort)
        self.tools = CalcTools()

        # 正常运动速度
        self.vel_move = 0.3
        self.acc_move = 0.3

        # 末端运动速度
        self.vel_end = 0.02
        self.acc_end = 0.02

        # 关节运动速度
        self.vel_joint = 1
        self.acc_joint = 1
        
        self.file_PosDefault = 'Duco&Transfer/Pose/0_PosDefault.csv'
        self.file_PosGetAntenna = 'Duco&Transfer/Pose/1_PosGetAntenna.csv'
        self.file_PosGetTcp = 'Duco&Transfer/Pose/2_PosGetTcp.csv'
        self.file_PosGetTarget = 'Duco&Transfer/Pose/3_PosGetTarget.csv'

        self.PosDefault = self.GetPosFromCsv(self.file_PosDefault)
        self.PosGetAntenna = self.GetPosFromCsv(self.file_PosGetAntenna)
        self.PosGetTcp = self.GetPosFromCsv(self.file_PosGetTcp)
        self.PosGetTarget = self.GetPosFromCsv(self.file_PosGetTarget)

        self.Deepth = [
            # 吸盘位置安全高度
            -0.046,
            # 吸盘位置上升至旋转高度
            -0.9,
            # 装配位置上方高度
            -0.1
        ]
        # with open(file_PosDefault, newline='') as csvfile:
        #     csvreader = csv.reader(csvfile)
        #     self.PosDefault = [float(value) for value in next(csvreader)]
        #     self.QNearDefault = [float(value) for value in next(csvreader)]
        
        # pass

    def GetPosFromCsv(self,filename):
        with open(filename, newline='') as csvfile:
            csvreader = csv.reader(csvfile)
            Pos = [float(value) for value in next(csvreader)]
            QNear = [float(value) for value in next(csvreader)]
        return [Pos,QNear]

    def GoToGetAntenna(self):
        # 获取默认位置
        PosStart = self.PosDefault
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(0)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0])**2 + (PosFirst[1] - PosStart[0][1])**2 + (PosFirst[2] - PosStart[0][2])**2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)
        
        # PosGetAntenna = self.GetPosFromCsv(self.file_PosGetAntenna)

        # 
        PosGetAntennaUp = self.PosGetAntenna[0].copy()
        PosGetAntennaUp[2] = PosGetAntennaUp[2] - self.Deepth[0]

        # 控制机械臂移动至天线上方
        self.duco.DucoMovel(PosGetAntennaUp,self.vel_move,self.acc_move,self.PosGetAntenna[1],'default')

        # 控制协作臂移动至吸附位置
        self.duco.DucoMovel(self.PosGetAntenna[0],self.vel_end,self.acc_end,self.PosGetAntenna[1],'default')
        
        return 1
    
    def TakeAntennaToGetTcp(self):
        # 获取起始位置
        PosStart = self.PosGetAntenna
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(0)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0])**2 + (PosFirst[1] - PosStart[0][1])**2 + (PosFirst[2] - PosStart[0][2])**2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)

        # 计算吸取点上方的位置
        PosGetAntennaUp = self.PosGetAntenna[0].copy()
        PosGetAntennaUp[2] = PosGetAntennaUp[2] - self.Deepth[0]

        # 控制机械臂移动至天线上方
        self.duco.DucoMovel(PosGetAntennaUp,self.vel_move,self.acc_move,self.PosGetAntenna[1],'default')

        # 控制协作臂移动至初始位置
        self.duco.DucoMovel(self.PosDefault[0],self.vel_move,self.acc_move,self.PosDefault[1],'default')

        # 控制协作臂移动至tcp检测位置
        self.duco.DucoMovel(self.PosGetTcp[0],self.vel_move,self.acc_move,self.PosGetTcp[1],'default')

        return 1

    def TakeCamToGetTarget(self):
        # 获取起始位置
        PosStart = self.PosGetTcp
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(0)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0])**2 + (PosFirst[1] - PosStart[0][1])**2 + (PosFirst[2] - PosStart[0][2])**2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)
        
        # 控制协作臂移动至目标拍照位置
        self.duco.DucoMovel(self.PosGetTarget[0],self.vel_move,self.acc_move,self.PosGetTarget[1],'default')

        return 1
    
    def TakeAntennaToTarget(self,TargetPosVec,TcpVec):
        # 获取起始位置
        PosStart = self.PosGetTarget
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(0)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0])**2 + (PosFirst[1] - PosStart[0][1])**2 + (PosFirst[2] - PosStart[0][2])**2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)

        # 计算末端Z轴上方的虚拟Tcp
        TcpVecUp = TcpVec.copy()
        TcpVecUp[2] = TcpVecUp[2] + 0.1
        TcpUpMat = self.tools.PosVecToPosMat(TcpVecUp)
        TcpUpMat_inv = np.linalg.inv(TcpUpMat)
        
        # 计算目标位置到法兰的变换
        TargetPos = self.tools.PosVecToPosMat(TargetPosVec)
        TcpMat = self.tools.PosVecToPosMat(TcpVec)
        TcpMat_inv = np.linalg.inv(TcpMat)
        TargetPosFlangeMat = np.dot(TargetPos,TcpMat_inv)
        TargetPosFlangeVec = self.tools.PosMatToPosVec(TargetPosFlangeMat)
        TargetPosUpFlangeMat = np.dot(TargetPos,TcpUpMat_inv)
        TargetPosUpFlangeVec = self.tools.PosMatToPosVec(TargetPosUpFlangeMat)

        print("目标姿态为: ",TargetPosFlangeVec)
        print("目标上方姿态为: ", TargetPosUpFlangeVec)
        
        # 控制协作臂来到目标TcpZ上方
        self.duco.DucoMovel(TargetPosUpFlangeVec,self.vel_move/2,self.acc_move,self.PosGetTarget[1],'default')

        # 控制协作臂沿着TcpZ轴前往目标
        self.duco.DucoMovel(TargetPosFlangeVec,self.vel_end,self.acc_end,self.PosGetTarget[1],'default')
        
        return 1

    def GoBackToDefault(self,TargetPosVec,TcpVec):
        TcpVecUp = TcpVec.copy()
        TcpVecUp[2] = TcpVecUp[2] + 0.1
        TcpUpMat = self.tools.PosVecToPosMat(TcpVecUp)
        TcpUpMat_inv = np.linalg.inv(TcpUpMat)
        

        TargetPos = self.tools.PosVecToPosMat(TargetPosVec)

        TargetPosUpFlangeMat = np.dot(TargetPos,TcpUpMat_inv)
        TargetPosUpFlangeVec = self.tools.PosMatToPosVec(TargetPosUpFlangeMat)
        # TargetPosFlange = TargetPosFlangeVec.copy()
        # print("目标姿态为: ",TargetPosFlangeVec)
        print("目标上方姿态为: ", TargetPosUpFlangeVec)

        # 控制协作臂回到目标上方
        self.duco.DucoMovel(TargetPosUpFlangeVec,self.vel_move/2,self.acc_move,self.PosGetTarget[1],'default')

        # 控制协作臂移动至目标拍照位置
        self.duco.DucoMovel(self.PosGetTarget[0],self.vel_move,self.acc_move,self.PosGetTarget[1],'default')

        # 控制协作臂移动至tcp检测位置
        self.duco.DucoMovel(self.PosGetTcp[0],self.vel_move,self.acc_move,self.PosGetTcp[1],'default')

        # 控制协作臂移动至初始位置
        self.duco.DucoMovel(self.PosDefault[0],self.vel_move,self.acc_move,self.PosDefault[1],'default')

        return 1
