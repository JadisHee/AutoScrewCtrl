import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl



class StepProcess:
    

    def __init__(self):
        self.ToolTransMat = np.array([[ 1, 0, 0, -0.00158513087673],
                                    [ 0, 1, 0, 0.1672619871841232],
                                    [ 0, 0, 1, 0],
                                    [ 0, 0, 0, 1]])

        # ######### 固定示教位置 #########
        self.PosDefault = [-0.9916632175445557, -0.07183022052049637, 0.1606786847114563, -3.141380786895752, 0.00010996363562298939, 1.5708098411560059]

        # 吸钉区域
        # 参考六轴
        self.QNearGetScrew = [0.08642122894525528, -0.18896421790122986, -2.0365021228790283, -0.9155300259590149, -1.484237790107727, -3.1383559703826904]
        # 吸顶拍照高度
        self.PosGetScrewPhoto = [-1.033828854560852, 0.019116833806037903, 0.062176916748285294, -3.1414084434509277, 0.00011105268640676513, 1.5708006620407104]


        # 拧钉区域
        # 参考六轴
        self.QNearTwistScrew = [0.2291293740272522, -0.22558800876140594, -1.989296317100525, -0.9265435338020325, -1.3415416479110718, -3.1383919715881348]
        # 拧钉拍照位置
        self.PosTwistScrewPhoto = [-1.0756033658981323, -0.1040751114487648, 0.06219949200749397, -3.1413872241973877, 9.985838551074266e-05, 1.5708328485488892]

        # 位移常速
        self.vel_move = 0.3
        self.acc_move = 0.5
        # 末端慢速
        self.vel_end = 0.02
        self.acc_end = 0.05

        self.Deepth_1 = 0.0365
        self.Deepth_2 = 0.0035

        pass

    def AutoDo(self,XmlData):
        # Duco机械臂的通讯地址
        DucoIp = "192.168.1.16"
        DucoPort = 7003
        duco = DucoCtrl(DucoIp,DucoPort)

        # hik相机的通讯地址
        HikIp = "192.168.1.5"
        HikPort = 8192
        # 通讯触发信号
        msgStart = '123'
        hik = HikCtrl(HikIp,HikPort)

        # danikor的通讯地址
        DanikorIp = '192.168.1.15'
        DanikorPort = 8888
        danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
        while(True):
            PosFirst = duco.GetDucoPos(1)
            # 判断协作臂是否位于初始位姿附近
            DistDefualt = math.sqrt((PosFirst[0] - self.PosDefault[0])**2 + (PosFirst[1] - self.PosDefault[1])**2 + (PosFirst[2] - self.PosDefault[2])**2)
        
            if DistDefualt <= 0.1:
                break
            else:
                print("请手动将协作臂移动至初始位置附近 ! ! !")

        
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------
        # 协作臂去吸钉拍照位置
        duco.DucoMovel(self.PosGetScrewPhoto,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            
        # 控制相机切换为螺钉识别方案
        hik.SetHikSwitchPlan('switch','GetScrew')

        # 控制相机识别螺钉
        result = hik.GetDataFromHik(msgStart)
        if result != 0:
            DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],11.94)
        else:
            print ("请检查相机设置 ! ! !")
            return
        
        # 获取当前姿态
        PosNow = duco.GetDucoPos(1)
        # 计算目标位置
        TargetPos = hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)
        # 记录螺钉上方的位置
        TargetPosUp = TargetPos.copy()

        # 控制协作臂移动至螺钉上方
        duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制协作臂移动至螺钉表面位置
        TargetPos[2] = TargetPos[2] - self.Deepth_1
        duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制电批开始反转寻帽
        danikor.ScrewMotorCtrl(2,XmlData)

        # 控制协作臂下降吸钉
        TargetPos[2] = TargetPos[2] - self.Deepth_2
        duco.DucoMovel(TargetPos,self.vel_end,self.acc_end,self.QNearGetScrew,'ElectricBit')

        # 控制真空阀打开
        danikor.VacuumCtrl(1)

        # 控制协作臂上升至螺钉孔上方
        duco.DucoMovel(TargetPosUp,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制协作臂来到拧钉拍照位置
        duco.DucoMovel(self.PosTwistScrewPhoto,self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')

        # 控制相机切换为螺钉识别方案
        hik.SetHikSwitchPlan('switch','FindHole')

        # 控制相机识别螺钉
        result = hik.GetDataFromHik(msgStart)
        if result != 0:
            DPos = hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],5.4)
        else:
            print ("请检查相机设置 ! ! !")
            return 0
        
        # 获取当前姿态
        PosNow = duco.GetDucoPos(1)
        # 计算目标位置
        TargetPos = hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)

        # 控制协作臂来到螺纹孔上方
        duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')
        
        # 控制电批模组伸出
        danikor.DriverCtrl(1)

        # 控制电批拧钉
        result = danikor.ScrewMotorCtrl(1,XmlData)

        # 控制电批模组收回
        danikor.DriverCtrl(0)

        # 控制电批回到初始默认位置
        duco.DucoMovel(self.PosDefault,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 初始化电批所有内容
        danikor.InitialAllMould()

        return result












