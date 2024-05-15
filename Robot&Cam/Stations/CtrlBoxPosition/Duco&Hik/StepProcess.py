import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl



class StepProcess:
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

    def __init__(self):
        self.ToolTransMat = np.array([[ 1, 0, 0, -0.00160632],
                         [ 0, 1, 0, 0.0500718],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])

        self.UnitMat = np.array([[ 1, 0, 0, 0],
                                 [ 0, 1, 0, 0],
                                 [ 0, 0, 1, 0],
                                 [ 0, 0, 0, 1]])

        # ######### 固定示教位置 #########
        self.PosDefault = [-0.9916632175445557, -0.07183022052049637, 0.1606786847114563, -3.141380786895752, 0.00010996363562298939, 1.5708098411560059]

        # 吸钉区域
        # 参考六轴
        self.QNearGetScrew = [0.08642122894525528, -0.18896421790122986, -2.0365021228790283, -0.9155300259590149, -1.484237790107727, -3.1383559703826904]
        # 吸顶拍照高度
        self.PosGetScrewPhoto = [-0.9083356857299805, 0.025900205597281456, 0.14376148581504822, -3.1414151191711426, 0.00011959159019170329, 1.570803165435791]

        self.PosTargetJ = [
            # 吸钉正常位置
            [-0.9915969967842102, -0.07178846001625061, 0.560700535774231, -3.14133358001709, 0.00012591338600032032, 1.5707948207855225],
            # 拧钉正常位置
            [-0.07031814754009247, 0.9917040467262268, 0.560724675655365, -3.141310214996338, 0.00013094619498588145, -0.001475528464652598],
            # 拧钉扭转过渡位置
            [0.24383601546287537, 0.847603976726532, 0.5615622997283936, -3.139066457748413, 0.004121103789657354, -1.5493718385696411],
            # 拧钉扭转位置
            [0.09962490946054459, 0.5273926258087158, 0.5608445405960083, 3.1415650844573975, -2.1984840714139864e-05, -3.1415915489196777]
        ]
        self.TargetJ = [
            # 吸钉正常位置
            [0.21218368411064148, 0.05437547340989113, -1.6194990873336792, -1.5762203931808472, -1.3589427471160889, -3.138319969177246],
            # 拧钉正常位置
            [-1.3601106405258179, 0.05438745766878128, -1.6194990873336792, -1.5762443542480469, -1.3589667081832886, -3.138319969177246],
            # 拧钉扭转过渡位置
            [-1.3601226806640625, 0.05438745766878128, -1.6194871664047241, -1.5762323141098022, 0.1889277994632721, -3.138308048248291],
            # 拧钉扭转位置
            [-1.3679484128952026, 0.05201457813382149, -1.6170064210891724, -1.5779101848602295, 1.7733502388000488, -3.1450791358947754]
        ]
        # 吸钉正常位置
        self.PosTargetJ_0 = [-0.9915969967842102, -0.07178846001625061, 0.560700535774231, -3.14133358001709, 0.00012591338600032032, 1.5707948207855225]
        self.TargetJ_0 = [0.21218368411064148, 0.05437547340989113, -1.6194990873336792, -1.5762203931808472, -1.3589427471160889, -3.138319969177246]
        # 拧钉正常位置
        self.PosTargetJ_1 = [-0.07031814754009247, 0.9917040467262268, 0.560724675655365, -3.141310214996338, 0.00013094619498588145, -0.001475528464652598]
        self.TargetJ_1 = [-1.3601106405258179, 0.05438745766878128, -1.6194990873336792, -1.5762443542480469, -1.3589667081832886, -3.138319969177246]
        # 拧钉扭转过渡位置
        self.TargetJ_2 = [-1.3601226806640625, 0.05438745766878128, -1.6194871664047241, -1.5762323141098022, 0.1889277994632721, -3.138308048248291]
        # 拧钉扭转位置
        self.PosTargetJ_3 = [0.09962490946054459, 0.5273926258087158, 0.5608445405960083, 3.1415650844573975, -2.1984840714139864e-05, -3.1415915489196777]
        self.TargetJ_3 = [-1.3679484128952026, 0.05201457813382149, -1.6170064210891724, -1.5779101848602295, 1.7733502388000488, -3.1450791358947754]
        # 第二个拍照位扭转
        self.TargetJ_4 = [-1.802412509918213, -0.060097843408584595, -1.7820771932601929, -1.2992888689041138, 1.3390898704528809, -3.144815444946289]

        # 拧钉区域
        # 参考六轴
        self.QNearTwistScrew = [0.2291293740272522, -0.22558800876140594, -1.989296317100525, -0.9265435338020325, -1.3415416479110718, -3.1383919715881348]
        # 拧钉拍照位置
        self.PosTwistScrewPhoto = [
            [0.13663356006145477, 0.9747881293296814, 0.34453698992729187, -3.1411917209625244, 0.00015040839207358658, -0.0027639823965728283],
            [0.2663731276988983, 0.982558012008667, 0.3445827066898346, -3.1411333084106445, 0.00017464248230680823, -0.0026692578103393316],
            [0.2613394558429718, 0.991369366645813, 0.3445833623409271, 3.141557455062866, 6.769683386664838e-05, 3.1413822174072266],
            [0.1479903608560562, 0.950756311416626, 0.3444559724330902, 3.141585350036621, -2.805951169193577e-07, 3.1415867805480957]
        ]
        
        # 位移常速
        self.vel_move = 0.3
        self.acc_move = 0.5
        # 末端慢速
        self.vel_end = 0.02
        self.acc_end = 0.05

        self.vel_joint = 0.785398
        self.acc_joint = 0.785398

        self.Deepth_1 = 0.114
        self.Deepth_2 = 0.005
        self.DownDeepth = [
            # 拍照位到吸钉表面高度
            0.114,
            # 吸钉再次下降高度
            0.005,
            # 拍照位到拧钉表面高度
            0.125
        ]

        pass


    def MoveToHikCenter(self,DPos):
        PosNow = self.duco.GetDucoPos(1)
        TargetPos = self.hik.GetTargetPos(DPos,PosNow,self.UnitMat)
        Qnear = self.duco.GetQNear()
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,Qnear,'ElectricBit')


    def StepGo(self,StepNum,XmlData):
        
        StepNum = StepNum - 1
        # 判断协作臂是否位于起始位姿
        for i in range(10):
            PosFirst = self.duco.GetDucoPos(1)
            # 判断协作臂是否位于初始位姿附近
            DistDefualt = math.sqrt((PosFirst[0] - self.PosDefault[0])**2 + (PosFirst[1] - self.PosDefault[1])**2 + (PosFirst[2] - self.PosDefault[2])**2)
            if DistDefualt <= 0.1:
                print("协作臂就位 ! ! !")
                break
            if i == 0:
                print("请手动将协作臂移动至初始位置附近 ! ! !")
            elif i == 9:
                print("协作臂未在初始位置 ! ! !")
                return
            time.sleep(1)

        # 控制协作臂前往吸顶拍照位置
        self.duco.DucoMovel(self.PosGetScrewPhoto,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        result = self.hik.GetDataFromHik(self.msgStart)
        if result != 0:
            DPos = self.hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],11.94)
        else:
            print ("请检查相机设置 ! ! !")
            return
            # 获取当前姿态
        PosNow = self.duco.GetDucoPos(1)
        # 计算目标位置
        TargetPos = self.hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)
        # 记录螺钉上方的位置
        TargetPosUp = TargetPos.copy()

        # 控制协作臂移动至螺钉上方
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制协作臂移动至螺钉表面位置
        TargetPos[2] = TargetPos[2] - self.DownDeepth[0]
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制电批开始反转寻帽
        self.danikor.ScrewMotorCtrl(2,XmlData)

        # 控制协作臂下降吸钉
        TargetPos[2] = TargetPos[2] - self.DownDeepth[1]
        self.duco.DucoMovel(TargetPos,self.vel_end,self.acc_end,self.QNearGetScrew,'ElectricBit')

        # 控制真空阀打开
        self.danikor.VacuumCtrl(1)

        # 控制协作臂上升至螺钉孔上方
        self.duco.DucoMovel(TargetPosUp,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制协作臂来到吸钉正常位置准备旋转
        self.duco.DucoMovel(self.PosTargetJ[0],self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 控制协作臂旋转至拧钉方向
        self.duco.DucoMoveJ(self.TargetJ[1],self.vel_joint,self.acc_joint)

        if StepNum >= 2:
            self.duco.DucoMoveJ(self.TargetJ[2],self.vel_joint,self.acc_joint)
            self.duco.DucoMoveJ(self.TargetJ[3],self.vel_joint,self.acc_joint)
        
        self.duco.DucoMovel(self.PosTwistScrewPhoto[StepNum],self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')

        while(True):
            # 控制相机识别并记录螺纹孔
            result = self.hik.GetDataFromHik(self.msgStart)
            if result != 0:
                DPos = self.hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],4)
                # DPos = [-result[1]/1000,-result[2]/1000]
            else:
                print ("请检查相机设置 ! ! !")
                return 0
            dist = math.sqrt(DPos[0]**2 + DPos[1]**2)

            PosNow = self.duco.GetDucoPos(1)
            TargetPos = self.hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)
            if dist <= 0.002:
                break
            self.MoveToHikCenter(DPos)
        # 记录当前位置
        TargetPosUp = TargetPos.copy()
        
        # 控制协作臂来到孔位上方
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')
        
        # 控制协作臂来到孔位表面
        TargetPos[2] = TargetPos[2] - self.DownDeepth[2]
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')

        # 控制电批气缸下降抵紧孔位
        self.danikor.DriverCtrl(1)

        # 控制电批转动拧紧螺钉
        self.danikor.ScrewMotorCtrl(1,XmlData)

        # 控制电批气缸收回
        self.danikor.DriverCtrl(0)

        # 控制协作臂回到孔位上方
        self.duco.DucoMovel(TargetPosUp,self.vel_move,self.acc_move,self.QNearTwistScrew,'ElectricBit')

        if StepNum >= 2:
            # 协作臂回到拧钉扭转起始位
            self.duco.DucoMovel(self.PosTargetJ[3],self.vel_move,self.acc_move,self.TargetJ_3,'ElectricBit')
            
            # 协作臂转回拧钉正常起始位
            self.duco.DucoMoveJ(self.TargetJ[2],self.vel_joint,self.acc_joint)
            self.duco.DucoMoveJ(self.TargetJ[1],self.vel_joint,self.acc_joint)            
        else:
            self.duco.DucoMovel(self.PosTargetJ[1],self.vel_move,self.acc_move,self.TargetJ[1],'ElectricBit')
    
                # 协作臂回到吸钉正常起始位
        self.duco.DucoMoveJ(self.TargetJ[0],self.vel_joint,self.acc_joint)

        # 协作臂回到默认位置
        self.duco.DucoMovel(self.PosDefault,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')


    def AutoDo(self,XmlData):
        
        while(True):
            PosFirst = self.duco.GetDucoPos(1)
            # 判断协作臂是否位于初始位姿附近
            DistDefualt = math.sqrt((PosFirst[0] - self.PosDefault[0])**2 + (PosFirst[1] - self.PosDefault[1])**2 + (PosFirst[2] - self.PosDefault[2])**2)
        
            if DistDefualt <= 0.1:
                break
            else:
                print("请手动将协作臂移动至初始位置附近 ! ! !")

        # 控制协作臂先去拍照位置
        TargetPos = self.duco.GetDucoPos(1)
        TargetPos[2] = TargetPos[2] + 0.4
        self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
        self.duco.DucoMoveJ(self.TargetJ_1,self.vel_joint,self.acc_joint)
        
        self.hik.SetHikSwitchPlan('switch','FindHole')
        TargetPoses = [[],[],[],[]]

        for i in range(4):
            if i ==  2:
                self.duco.DucoMoveJ(self.TargetJ_4,self.vel_joint,self.acc_joint)

            self.duco.DucoMovel(self.PosTwistScrewPhoto[i],self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            # 控制螺钉
            
            while(True):

                # 控制相机识别并记录螺纹孔
                result = self.hik.GetDataFromHik(self.msgStart)
                if result != 0:
                    DPos = self.hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],4)
                    # DPos = [-result[1]/1000,-result[2]/1000]
                else:
                    print ("请检查相机设置 ! ! !")
                    return 0
                dist = math.sqrt(DPos[0]**2 + DPos[1]**2)

                PosNow = self.duco.GetDucoPos(1)
                TargetPos = self.hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)
                

                if dist <= 0.002:
                    break

                self.MoveToHikCenter(DPos)

            TargetPoses[i].extend(TargetPos)
        
        # 协作臂回到拧钉扭转起始位
        self.duco.DucoMovel(self.PosTargetJ_3,self.vel_move,self.acc_move,self.TargetJ_3,'ElectricBit')
        
        # 协作臂转回拧钉正常起始位
        self.duco.DucoMoveJ(self.TargetJ_2,self.vel_joint,self.acc_joint)
        self.duco.DucoMoveJ(self.TargetJ_1,self.vel_joint,self.acc_joint)

        # 协作臂回到吸钉正常起始位
        self.duco.DucoMoveJ(self.TargetJ_0,self.vel_joint,self.acc_joint)

        # 协作臂回到默认位置
        self.duco.DucoMovel(self.PosDefault,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        # 切换相机识别方案为取钉
        self.hik.SetHikSwitchPlan('switch','GetScrew')
        for i in range(4):
            # 控制协作臂来到拍照位置
            self.duco.DucoMovel(self.PosGetScrewPhoto,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

            result = self.hik.GetDataFromHik(self.msgStart)
            if result != 0:
                DPos = self.hik.GetDPosMillimeter(result[1]*2,[result[2],result[3]],11.94)
            else:
                print ("请检查相机设置 ! ! !")
                return
                # 获取当前姿态
            PosNow = self.duco.GetDucoPos(1)
            # 计算目标位置
            TargetPos = self.hik.GetTargetPos(DPos,PosNow,self.ToolTransMat)
            # 记录螺钉上方的位置
            TargetPosUp = TargetPos.copy()

            # 控制协作臂移动至螺钉上方
            self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

            # 控制协作臂移动至螺钉表面位置
            TargetPos[2] = TargetPos[2] - self.Deepth_1
            self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

            # 控制电批开始反转寻帽
            self.danikor.ScrewMotorCtrl(2,XmlData)

            # 控制协作臂下降吸钉
            TargetPos[2] = TargetPos[2] - self.Deepth_2
            self.duco.DucoMovel(TargetPos,self.vel_end,self.acc_end,self.QNearGetScrew,'ElectricBit')

            # 控制真空阀打开
            self.danikor.VacuumCtrl(1)

            # 控制协作臂上升至螺钉孔上方
            self.duco.DucoMovel(TargetPosUp,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

            # 协作臂来到初始位置
            # self.duco.DucoMovel(self.PosDefault,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

            # 控制协作臂来到吸顶正常位置
            TargetPos = self.duco.GetDucoPos(1)
            TargetPos[2] = TargetPos[2] + 0.4
            self.duco.DucoMovel(TargetPos,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            
            # 控制协作臂来到拧钉正常位置
            self.duco.DucoMoveJ(self.TargetJ_1,self.vel_joint,self.acc_joint)

            if i >= 2:
                self.duco.DucoMoveJ(self.TargetJ_2,self.vel_joint,self.acc_joint)
                self.duco.DucoMoveJ(self.TargetJ_3,self.vel_joint,self.acc_joint)

            self.duco.DucoMovel(TargetPoses[i],self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            TargetPos = TargetPoses[i].copy()
            
            TargetPos[2] = TargetPos[2] - 0.125
            self.duco.DucoMovel(TargetPoses[i],self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            self.danikor.DriverCtrl(1)

            self.danikor.ScrewMotorCtrl(1,XmlData)

            self.duco.DucoMovel(TargetPoses[i],self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')
            if i >= 2:

                # 协作臂回到拧钉扭转起始位
                self.duco.DucoMovel(self.PosTargetJ_3,self.vel_move,self.acc_move,self.TargetJ_3,'ElectricBit')
                
                # 协作臂转回拧钉正常起始位
                self.duco.DucoMoveJ(self.TargetJ_2,self.vel_joint,self.acc_joint)
                self.duco.DucoMoveJ(self.TargetJ_1,self.vel_joint,self.acc_joint)            
            else:
                self.duco.DucoMovel(self.PosTargetJ_1,self.vel_move,self.acc_move,self.TargetJ_1,'ElectricBit')
        
                    # 协作臂回到吸钉正常起始位
            self.duco.DucoMoveJ(self.TargetJ_0,self.vel_joint,self.acc_joint)

            # 协作臂回到默认位置
            self.duco.DucoMovel(self.PosDefault,self.vel_move,self.acc_move,self.QNearGetScrew,'ElectricBit')

        
        return
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
            # DPos = [-result[1]/1000,-result[2]/1000]
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












