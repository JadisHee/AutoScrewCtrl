import numpy as np
import time
import math
import csv

# from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools

class StepProcess:
    
    #####################################################
    # 外部设备参数
    #####################################################
    # --------------------Duco------------------------
    DucoIp = "192.168.1.16"
    DucoPort = 7003

    # 常速位移
    vel_move = 0.3
    acc_move = 0.5

    # 慢速位移
    vel_end = 0.02
    acc_end = 0.05

    # 关节常速
    vel_joint = 0.785398
    acc_joint = 0.785398

    DownDeepth = [
        # 拍照位到吸钉表面高度
        0.199,
        # 吸钉再次下降高度
        0.0095,
        # 吸钉回升的高度
        -0.07,
        # 吸钉检测下降高度
        0.05,
        # 拍照位到拧钉表面高度
        0.275
    ]

    # ---------------------Hik------------------------
    HikIp = "192.168.1.17"
    HikPort = 8192

    # 螺钉顶部圆圈直径
    ScrewCircleDiameter = 11.7
    # 目标圆孔直径
    TargetCircleDiameter = 32

    # ---------------------Danikor------------------------
    DanikorIp = '192.168.1.15'
    DanikorPort = 8888

    # ---------------------Transfer------------------------
    TransferIp = '192.168.1.48'
    TransferPort = 5700

    # 实例化
    tools = CalcTools()
    duco = DucoCtrl(DucoIp,DucoPort)
    hik = HikCtrl(HikIp,HikPort)
    danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
    transfer = TransferCtrl(TransferIp,TransferPort)

    def __init__(self):
        ################################################################
        ########################## 内参 ################################
        ################################################################
        # Bit --> Cam
        self.ToolCenterToCamCenterTransMat = np.array([
            [1, 0, 0, 0.003137055],
            [0, 1, 0, 0.16504149733351],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 单位阵
        self.UnitMat = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # DucoBase --> KukaBase
        self.DucoKukaTransMat = np.array([
            [0.69350914, 0.72032586, 0.01319006, -1.0707572],
            [-0.72008126, 0.69361943, -0.01938605, 2.18730646],
            [-0.02311231, 0.00394696, 0.99972512, -0.78090808],
            [0., 0., 0., 1.]
        ])

        #
        self.file_PosDefault = 'Pose/0_PosDefault.csv'
        self.file_PosGetScrewPhoto = 'Pose/1_GetScrewPhoto.csv'
        self.file_PosScrewConfirm = 'Pose/2_ConfermScrew.csv'

        self.file_PosTwistDefault = 'Pose/3_TwistDefault.csv'
        self.file_PosTwistDefault_mid = 'Pose/3_TwistDefault_mid.csv'

        self.file_PosTwistPhotoClose_1 = 'Pose/4_TwistPhotoClose_1.csv'
        self.file_PosTwistPhotoClose_2 = 'Pose/4_TwistPhotoClose_2.csv'
        self.file_PosTwistPhotoFar_3 = 'Pose/4_TwistPhotoFar_3.csv'
        self.file_PosTwistPhotoFar_4 = 'Pose/4_TwistPhotoFar_4.csv'
        self.file_PosTwistPhotoFar_mid = 'Pose/4_TwistPhotoFar_mid.csv'

        self.PosDefault = self.GetPosFromCsv(self.file_PosDefault)
        self.PosGetScrewPhoto = self.GetPosFromCsv(self.file_PosGetScrewPhoto)
        self.PosScrewConfirm = self.GetPosFromCsv(self.file_PosScrewConfirm)

        self.PosTwistDefault = self.GetPosFromCsv(self.file_PosTwistDefault)
        self.PosTwistDefault_mid = self.GetPosFromCsv(self.file_PosTwistDefault_mid)

        self.PosTwistPhotoClose_1 = self.GetPosFromCsv(self.file_PosTwistPhotoClose_1)
        self.PosTwistPhotoClose_2 = self.GetPosFromCsv(self.file_PosTwistPhotoClose_2)
        self.PosTwistPhotoFar_3 = self.GetPosFromCsv(self.file_PosTwistPhotoFar_3)
        self.PosTwistPhotoFar_4 = self.GetPosFromCsv(self.file_PosTwistPhotoFar_4)
        self.PosTwistPhotoFar_mid = self.GetPosFromCsv(self.file_PosTwistPhotoFar_mid)


    def GetPosFromCsv(self, filename):
        '''
        * Function:     GetPosFromCsv
        * Description:  获取csv文件中的位姿数据
        * Inputs:
                        filename : 文件名
        * Outputs:      
        * Returns:      list:[Pos,QNear]:
                            Pos:list[float]
                            QNear: list[float]
        * Notes:
        '''
        with open(filename, newline='') as csvfile:
            csvreader = csv.reader(csvfile)
            Pos = [float(value) for value in next(csvreader)]
            QNear = [float(value) for value in next(csvreader)]
        return [Pos, QNear]


    def ScrewConfirm(self):
        '''
            * Function:     ScrewConfirm
            * Description:  确认螺钉状态
            * Inputs:
                            
            * Outputs:      
            * Returns:      bool
                                1: 螺钉准备就绪
                                0: 螺钉未吸上或未吸紧
            * Notes:
        '''
        VacuumPressure = self.duco.DucoRobot.get_standard_digital_in(8)
        IsThereScrew = self.duco.DucoRobot.get_standard_digital_in(7)
        if VacuumPressure == 0 and IsThereScrew == 1:
            print("螺钉吸附稳定! ! !")
            return 1
        elif VacuumPressure == 1 and IsThereScrew == 1:
            print("螺钉吸附不稳定! ! !请检查批头末端或负压传感器! ! !")
        else:
            print("螺钉未吸附上! ! !")

    def MoveToCenter(self, mod, DPos):
        '''
            * Function:     MoveToCenter
            * Description:  在识别后控制协作臂移动对准中心
            * Inputs:
                            mod:
                                0: 对准相机中心
                                1: 对准电批中心
                            DPos: 圆心在相机中心坐标系的坐标,单位:mm
                                list[dx,dy]
            * Outputs:      
            * Returns:      
                            TargetPos:协作臂目标位姿
            * Notes:
        '''
        PosNow = self.duco.GetDucoPos(1)
        if mod == 0:
            TargetPos = self.hik.GetTargetPos(DPos, PosNow, self.UnitMat)
        elif mod == 1:
            TargetPos = self.hik.GetTargetPos(DPos, PosNow, self.ToolCenterToCamCenterTransMat)
        Qnear = self.duco.GetQNear()
        # print(TargetPos)
        self.duco.DucoMovel(TargetPos, self.vel_move, self.acc_move, Qnear, 'ElectricBit')

        return TargetPos

    def GoToGetScrew(self):
        '''
            * Function:     GoGetScrew
            * Description:  前往获取螺钉
            * Inputs:
                                
            * Outputs:      
            * Returns:      
                            IsScrewOk: bool
                                1: 螺钉准备就绪
                                0: 螺钉未吸上或未吸紧
            * Notes:
        '''
        # 获取默认位置
        PosStart = self.PosGetScrewPhoto
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(1)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0]) ** 2 + (PosFirst[1] - PosStart[0][1]) ** 2 + (
                PosFirst[2] - PosStart[0][2]) ** 2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0
        time.sleep(1)

        # 控制相机切换方案
        IsSwitchPlanSuccessful = self.hik.SetHikSwitchPlan('switch', 'FindScrew-M8-6Angle')
        if IsSwitchPlanSuccessful == 0:
            return 0

        # 将相机移动至目标中心，直到接近中心后退出循环
        while (True):
            DPos = self.hik.GetHikDPos(self.ScrewCircleDiameter)
            dist = math.sqrt(DPos[0] ** 2 + DPos[1] ** 2)
            if dist <= 0.002:
                break
            self.MoveToCenter(0, DPos)

        # 控制协作臂将披头移动至目标中心上方
        TargetPos = self.MoveToCenter(1, DPos)

        # 控制机械臂下降至螺帽上方
        TargetPos[2] = TargetPos[2] - self.DownDeepth[0] 
        self.duco.DucoMovel(TargetPos, self.vel_move, self.acc_move, PosStart[1], 'ElectricBit')

         # 控制电批反拧寻帽
        self.danikor.ScrewMotorCtrl(2)

        # 控制机械臂下降寻帽子
        TargetPos[2] = TargetPos[2] - self.DownDeepth[1]
        self.duco.DucoMovel(TargetPos, self.vel_end, self.acc_end, PosStart[1], 'ElectricBit')

        # 打开真空阀吸钉
        self.danikor.VacuumCtrl(1)

        # 控制夹爪闭合
        self.danikor.ClawCtrl(0)

        time.sleep(1)

        # 控制机械臂抬起
        TargetPos[2] = TargetPos[2] - self.DownDeepth[2]
        self.duco.DucoMovel(TargetPos, self.vel_end, self.acc_end, PosStart[1], 'ElectricBit')

        # 控制机械臂前往吸钉确认位置
        TargetPos = self.PosScrewConfirm[0].copy()
        QNear = self.PosScrewConfirm[1].copy()
        self.duco.DucoMovel(TargetPos, self.vel_move, self.acc_move, QNear, 'ElectricBit')

        # 控制机械臂下降至吸钉检测位置
        GoDown = TargetPos.copy()
        GoDown[2] = GoDown[2] - self.DownDeepth[3]
        self.duco.DucoMovel(GoDown, self.vel_move, self.acc_move, QNear, 'ElectricBit')

        # 螺钉检测系统对螺钉的吸附情况进行检测
        time.sleep(0.5)
        IsScrewOk = self.ScrewConfirm()
        time.sleep(0.5)

        # 控制协作臂回到螺钉检测上方
        self.duco.DucoMovel(TargetPos, self.vel_move, self.acc_move, QNear, 'ElectricBit')

        # 控制协作臂回到初始位置
        self.duco.DucoMovel(self.PosDefault[0], self.vel_move, self.acc_move, self.PosDefault[1], 'ElectricBit')

        return IsScrewOk



    def StartProcess(self,StepNum,XmlData=None):
        # 获取默认位置
        PosStart = self.PosDefault
        # 判断机械臂是否位于初始位姿附近
        PosFirst = self.duco.GetDucoPos(1)
        DistDefualt = math.sqrt((PosFirst[0] - PosStart[0][0]) ** 2 + (PosFirst[1] - PosStart[0][1]) ** 2 + (
                PosFirst[2] - PosStart[0][2]) ** 2)
        if DistDefualt > 0.001:
            print("dist: ", DistDefualt)
            print("机械臂未就位 ! ! !")
            return 0
        time.sleep(1)

        print("电批模组初始化中 ! ! !")
        # 控制电批模组初始化
        self.danikor.InitialAllMould()
        print("电批模组初始化完成 ! ! !")

        # 控制机械臂移动至拍照位置附近
        self.duco.DucoMovel(self.PosGetScrewPhoto[0], self.vel_move, self.acc_move, self.PosGetScrewPhoto[1],'ElectricBit')

         # 判断吸钉是否就绪
        IsScrewOk = self.GoGetScrew()
        if IsScrewOk != 1:
            print("螺钉未就位 ! ! !")
            return 0

         # 控制协作臂转动到拧钉方向
        self.duco.DucoMoveJ(self.PosTwistDefault[1], self.vel_joint, self.acc_joint)
        self.duco.DucoMoveJ(self.PosTwistDefault_mid[1], self.vel_joint, self.acc_joint)

        # 与迁移相机通讯获取数据
        PosINSTarget = self.transfer.GetRotData('123')

        # 提取出旋转分类
        RotVec = self.tools.GetPhotoRot(StepNum,PosINSTarget,self.DucoKukaTransMat)

        # 根据不同的点位获取不同的拍照位置
        if StepNum == 1:
            PosPhoto = self.PosTwistPhotoClose_1[0].copy()
        elif StepNum == 2:
            PosPhoto = self.PosTwistPhotoClose_2[0].copy()
        elif StepNum == 3:
            PosPhoto = self.PosTwistPhotoFar_3[0].copy()
        elif StepNum == 4:
            PosPhoto = self.PosTwistPhotoFar_4[0].copy()
        
        # 对拍照位置的旋转分量进行替换
        PosPhoto[3:] = RotVec

        # 根据不同的步骤选择不同的路径点
        if StepNum == 1 or StepNum == 2:
            self.duco.DucoMovel(PosPhoto, self.vel_move,self.acc_move,self.PosTwistPhotoClose_1[1], 'ElectricBit')
        elif StepNum == 3 or StepNum == 4:
            self.duco.DucoMoveJ(self.PosTwistPhotoFar_mid[1], self.vel_joint, self.acc_joint)
            self.duco.DucoMovel(PosPhoto, self.vel_move / 3, self.acc_move / 3, self.PosTwistPhotoFar_3[1],'ElectricBit')
            
        
        #









