import numpy as np
import time
import math

from SocketCtrl import XmlData

from DucoCtrl import DucoCtrl




class StepProcess:
    
    

    def __init__(self):
        # Duco机械臂的通讯地址
        DucoIp = "192.168.1.47"
        DucoPort = 7003

        self.duco = DucoCtrl(DucoIp,DucoPort)
        
        ######### 固定示教位置 #########
        # 默认位置
        self.PosDefault = [-0.8928816318511963, -0.14681445062160492, 0.2584591805934906, -3.141585111618042, 4.035204619867727e-05, -2.2165772914886475]
        self.QNearDefault = [3.117692232131958, 0.3941248059272766, 2.1879897117614746, -1.0102074146270752, 4.712048053741455, -2.5242996215820312]

        # 取吸盘的位置
        self.PosGetAntenna = [-0.9145084619522095, 0.26888221502304077, -0.08269526064395905, 3.1415555477142334, 1.4653284779342357e-05, -2.216585397720337]
        self.QNearGetAntenna = [2.6782548427581787, 0.8538756370544434, 2.1059577465057373, -1.3878902196884155, 4.713150978088379, -2.963437557220459]
        
        self.PosTargetJ = [
            # 吸天线可旋转位置
            [-0.8929035663604736, -0.14680883288383484, 1.158453345298767, -3.1415860652923584, 4.647010064218193e-05, -2.21659517288208],
            # 检测天线高度位置
            [0.1463165581226349, -0.8929854035377502, 1.1584316492080688, -3.141566753387451, 6.040588050382212e-05, -0.6463262438774109],
            # 检测天线位置
            [0.1462930142879486, -0.8929665684700012, 0.5435361266136169, -3.141589641571045, 5.114184386911802e-05, -0.6463720202445984],

            # 检测安装位置
            [0.837226152420044, -0.5443787574768066, 0.7296354174613953, -3.1415646076202393, 5.2592290558095556e-06, -0.6463878154754639]

        
        ]
        self.TargetJ = [
            # 吸天线可旋转位置
            [3.1180636882781982, -0.017029978334903717, 1.3774245977401733, 0.2115962952375412, 4.710741996765137, -2.522669792175293],
            # 检测天线高度位置
            [4.688284873962402, -0.01705394685268402, 1.3774245977401733, 0.21158431470394135, 4.710753917694092, -2.522669792175293],
            # 检测天线位置
            [4.68808126449585, 0.1193385198712349, 2.0500152111053467, -0.5973988175392151, 4.711185455322266, -2.5238802433013916],
            # 检测安装位置
            [5.537798881530762, 0.121208056807518, 1.7828389406204224, -0.3320920467376709, 4.711089611053467, -1.6738392114639282]

        ]

        self.Deepth = [
            # 吸盘位置下降高度
            0.046,
            # 吸盘位置上升至旋转高度
            -0.9
        ]

        
        # 
        self.vel_move = 0.3
        self.acc_move = 0.3

        self.vel_end = 0.02
        self.acc_end = 0.02

        self.vel_joint = 1
        self.acc_joint = 1
        pass


    def GoToGetAntenna(self):
        # 判断机械臂是否位于起始位姿
        
        PosFirst = self.duco.GetDucoPos(1)
        # 判断机械臂是否位于初始位姿附近
        DistDefualt = math.sqrt((PosFirst[0] - self.PosDefault[0])**2 + (PosFirst[1] - self.PosDefault[1])**2 + (PosFirst[2] - self.PosDefault[2])**2)
        if DistDefualt > 0.001:
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)

        # 控制机械臂移动至天线上方
        self.duco.DucoMovel(self.PosGetAntenna,self.vel_move,self.acc_move,self.QNearGetAntenna,'default')

        # 控制机械臂移动至吸天线表面
        PosTarget = self.PosGetAntenna.copy()
        PosTarget[2] = PosTarget[2] - self.Deepth[0]
        self.duco.DucoMovel(PosTarget,self.vel_end,self.acc_end,self.QNearGetAntenna,'default')
        
        return 1

    def TakeAntennaToConfirmPos(self):
        # 控制机械臂移动至吸天线表面
        PosTarget = self.PosGetAntenna.copy()
        PosTarget[2] = PosTarget[2] - self.Deepth[0]

        PosFirst = self.duco.GetDucoPos(1)
        # 判断机械臂是否位于初始位姿附近
        DistDefualt = math.sqrt((PosFirst[0] - PosTarget[0])**2 + (PosFirst[1] - PosTarget[1])**2 + (PosFirst[2] - PosTarget[2])**2)
        if DistDefualt > 0.001:
            print("机械臂未就位 ! ! !")
            return 0 
        time.sleep(1)

        # 控制机械臂将天线抬出托盘
        self.duco.DucoMovel(self.PosGetAntenna,self.vel_end,self.acc_end,self.QNearGetAntenna,'default')

        # 控制机械臂移动至可旋转高度
        PosTarget = self.PosDefault.copy()
        PosTarget[2] = PosTarget[2] - self.Deepth[1]
        self.duco.DucoMovel(PosTarget,self.vel_move,self.acc_move,self.QNearGetAntenna,'default')
        
        # 控制机械臂移动至吸取检测位置
        self.duco.DucoMoveJ(self.TargetJ[1],self.vel_joint,self.acc_joint)
        self.duco.DucoMovel(self.PosTargetJ[2],self.vel_move,self.acc_move,self.TargetJ[2],'default')
        
