import numpy as np
import time
import math

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl


class StepProcess:
    
    def __init__(self):
        ######### 固定示教位置 #########
        # 参考六轴
        self.QNearGetScrew = [-1.6864653825759888, 0.32770687341690063, 2.014782190322876, 0.7951695919036865, 1.4564528465270996, -1.575836181640625]
        # 吸钉退出位置
        self.PosGetScrewLeave = [0.06171630509197712, -0.9607846736907959,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

        # 吸钉检测过渡位置
        self.PosGetScrewGoToConfirm = [0.06171630509197712,-0.7258163094520569,0.3653124272823334,3.134021759033203, 1.5690670013427734, 1.5631933212280273]

        # 吸钉检测位置:
        self.PosGetScrewConfirm = [0.28018778562545776, -0.7258163094520569, 0.3547946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

        # 吸钉检测位置退出:
        self.PosGetScrewConfirmLeave = [0.28018778562545776, -0.7258163094520569, 0.4047946810722351, 3.132913589477539, 1.5690628290176392, 1.5620795488357544]

        # 吸钉方向就绪位置
        self.PosGetScrewStandby = [0.06171630509197712, -0.9607846736907959,0.8128352165222168,3.134021759033203, 1.5690670013427734, 1.5631933212280273]
        
        self.PosGetScrewMoveJ = [0.06975271552801132, -0.48145827651023865, 0.8128352165222168, 3.1363139152526855, 1.569033145904541, 1.5655303001403809]
        self.QNearGetScrewMoveJ = [-1.8543643951416016, -0.5473452806472778, 2.1644771099090576, 1.5211260318756104, 1.287714958190918, -1.5744819641113281]



        # 拧钉区域
        # 拧钉方向第一轴可动位置:
        self.PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
        self.QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]

        # 拧钉左侧参考六轴
        self.QNearTwistScrewLeft = [1.5505576133728027, 0.8574095964431763, 2.8965694904327393, -0.6185015439987183, 1.5518832206726074, -0.007149023003876209]
        # 拧钉左侧就绪位置:
        self.PosTwistScrewLeftStandby = [-0.16429604589939117, 0.33377784490585327, 0.03015068918466568, 1.5707684755325317, 1.137180151999928e-05, -3.141587972640991]
        
        # 拧钉右侧参考六轴
        self.QNearTwistScrewRight = [1.565573811531067, 0.6790363788604736, 2.088568925857544, 0.36772826313972473, 1.5671511888504028, -0.007041165139526129]
        # 正常姿态下可以进行扭转位置:
        self.PosTwistScrewNormalRightStandby = [-0.16424693167209625, 0.9588245749473572, 0.050218433141708374, 1.5707995891571045, 0.0009492189856246114, 3.141490936279297]
        # 正常状态扭转至右侧拧钉状态中的过渡位置
        self.PosTwistScrewNormalRightStandbyToTarget = [-0.16425147652626038, 0.9588478803634644, 0.05021889507770538, 3.1415793895721436, 0.0009572505950927734, 3.1414880752563477]
        # 拧钉右侧就绪位置:
        self.PosTwistScrewRightStandby = [-0.16427645087242126, 0.9588826298713684, 0.05017045885324478, -1.5700844526290894, 0.001079818233847618, 3.1415560245513916]

        pass


    def Screw_1(self,PosGetScrew,PosTwistScrewSlowDown):
        '''
            * Function:     Screw_1
            * Description:  惯导拧钉系统执行惯导左边（面向惯导）孔位的拧钉
            * Inputs:       
                            PosGetScrew: 取钉位置位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
                            PosTwistScrewSlowDown: 拧钉减速位置位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
            * Outputs:        
            * Returns:      
                            0: 未吸附上螺钉
                            其他: 拧钉力矩
            * Notes:
        '''

        # 电批模组的通讯地址
        DanikorIp = '192.168.1.15'
        DanikorPort = 8888
        #------------------------设置协作臂相关参数----------------------------
        # Duco机械臂的通讯地址
        DucoIp = "192.168.1.16"
        DucoPort = 7003

        danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
        duco = DucoCtrl(DucoIp,DucoPort)
        # 机械臂末端的速度
        vel_move = 0.5
        acc_move = 0.3

        vel_end = 0.02
        acc_end = 0.2

        vel_joint = 0.2617993
        acc_joint = 0.2617993

        #------------------------示教位置---------------------------
        # 吸钉区域
        
        
        # 吸取出螺钉后的安全高度
        HeightSafty = 0.05

        # 到达吸钉位置后再次下降确保吸稳固
        HeightDownAgain = 0.005

        # 吸钉位置
        # PosGetScrew = [0.08444300293922424, -0.9816458821296692, 0.3237221694946289, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
        
        # 吸钉减速位置
        PosGetScrewSlowDown = [PosGetScrew[0],PosGetScrew[1],PosGetScrew[2]+HeightSafty,PosGetScrew[3],PosGetScrew[4],PosGetScrew[5]]

        # PosGetScrewSlowDown = [0.08444300293922424, -0.9816458821296692, 0.3653124272823334, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
        
        # 吸稳位置
        PosGetScrewDownAgain = [PosGetScrew[0],PosGetScrew[1],PosGetScrew[2]-HeightDownAgain,PosGetScrew[3],PosGetScrew[4],PosGetScrew[5]]
        
        
        
        # 螺钉入孔深度
        HeightScrewInto = 0.05
        
        # 入孔位置
        PosTwistScrew = [PosTwistScrewSlowDown[0] - HeightScrewInto,PosTwistScrewSlowDown[1],PosTwistScrewSlowDown[2],PosTwistScrewSlowDown[3],PosTwistScrewSlowDown[4],PosTwistScrewSlowDown[5]]

        # 协作臂回到默认位置（吸钉区域安全位置）
        duco.DucoMoveL(self.PosGetScrewStandby,vel_move,acc_move,self.QNearGetScrew)

        danikor.ClawCtrl(1)

        # 协作臂快速来到吸钉减速位置
        duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂慢速来到吸钉位置
        duco.DucoMoveL(PosGetScrew,vel_end,acc_end,self.QNearGetScrew)

        # 控制电批反转认帽
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)
        # 第二次深入吸稳位置
        duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,self.QNearGetScrew)

        # 夹具夹紧
        danikor.ClawCtrl(0)

        # 协作臂慢速回到吸钉减速位置
        duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,self.QNearGetScrew)

        # 协作臂快速来到吸钉退出位置
        duco.DucoMoveL(self.PosGetScrewLeave,vel_move,acc_move,self.QNearGetScrew)
        
        # 协作臂快速来到检测过渡位置
        duco.DucoMoveL(self.PosGetScrewGoToConfirm,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂来到吸钉检测位置
        duco.DucoMoveL(self.PosGetScrewConfirm,vel_move,acc_move,self.QNearGetScrew)

        # 等待检测
        IsScrewOk = danikor.ScrewConferm()
        if IsScrewOk == False:
            # 协作臂来到检测退出位置
            duco.DucoMoveL(self.PosGetScrewConfirmLeave,vel_move,acc_move,self.QNearGetScrew)
            return 0

        # 协作臂来到检测退出位置
        duco.DucoMoveL(self.PosGetScrewConfirmLeave,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂快速来到吸钉方向第一轴可动位置
        duco.DucoMoveL(self.PosGetScrewMoveJ,vel_move,acc_move,self.QNearGetScrewMoveJ)
        
        #----------------------------------------------------------------------

        # 协作臂转动来到拧钉方向可旋转位置
        duco.DucoMoveJ(self.QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

        # 协作臂来到拧钉就绪位置
        duco.DucoMoveL(self.PosTwistScrewLeftStandby,vel_move,acc_move,self.QNearTwistScrewLeft)

        # 协作臂快速来到拧钉减速位置
        duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,self.QNearTwistScrewLeft)

        # 协作臂慢速来到拧钉位置
        duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,self.QNearTwistScrewLeft)

        # 夹具松开
        danikor.ClawCtrl(1)

        # 控制电批拧入螺钉
        danikor.ScrewMotorCtrl(1)
        time.sleep(10)

        # 协作臂慢速回到拧钉减速位置
        duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,self.QNearTwistScrewLeft)

        # 协作臂快速回到拧钉就绪位置
        duco.DucoMoveL(self.PosTwistScrewLeftStandby,vel_move,acc_move,self.QNearTwistScrewLeft)

        # 协作臂快速轴动到轴动位置
        duco.DucoMoveL(self.PosTwistScrewMoveJ,vel_move,acc_move,self.QNearTwistScrewMoveJ)

        #---------------------------------------------------------------------------
        # 协作臂轴动转回至取钉轴动区域
        duco.DucoMoveJ(self.QNearGetScrewMoveJ,vel_joint*4,acc_joint)

        # 协作臂来到吸钉安全位置
        duco.DucoMoveL(self.PosGetScrewStandby,vel_move,acc_move,self.QNearGetScrew)

        return 3.12

    def Screw_2(self,PosGetScrew,PosTwistScrewSlowDown):
        '''
            * Function:     Screw_2
            * Description:  惯导拧钉系统执行惯导右边（面向惯导）孔位的拧钉
            * Inputs:       
                            PosGetScrew: 取钉位置位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
                            PosTwistScrewSlowDown: 拧钉减速位置位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
            * Outputs:        
            * Returns:      
            * Notes:
        '''

        #------------------------设置电批相关参数----------------------------
        # 电批模组的通讯地址
        DanikorIp = '192.168.1.15'
        DanikorPort = 8888
        #------------------------设置协作臂相关参数----------------------------
        # Duco机械臂的通讯地址
        DucoIp = "192.168.1.16"
        DucoPort = 7003

        danikor = DanikorCtrl(DucoIp,DucoPort,DanikorIp,DanikorPort)
        duco = DucoCtrl(DucoIp,DucoPort)
        # 机械臂末端的速度
        vel_move = 0.5
        acc_move = 0.3

        vel_end = 0.02
        acc_end = 0.2

        vel_joint = 0.2617993
        acc_joint = 0.2617993

        #------------------------示教位置---------------------------
        # 吸钉区域

        # 吸取出螺钉后的安全高度
        HeightSafty = 0.05

        # 到达吸钉位置后再次下降确保吸稳固
        HeightDownAgain = 0.005

        # 吸钉减速位置
        PosGetScrewSlowDown = [PosGetScrew[0],PosGetScrew[1],PosGetScrew[2]+HeightSafty,PosGetScrew[3],PosGetScrew[4],PosGetScrew[5]]

        # PosGetScrewSlowDown = [0.08444300293922424, -0.9816458821296692, 0.3653124272823334, 3.134021759033203, 1.5690670013427734, 1.5631933212280273]
        
        # 吸稳位置
        PosGetScrewDownAgain = [PosGetScrew[0],PosGetScrew[1],PosGetScrew[2]-HeightDownAgain,PosGetScrew[3],PosGetScrew[4],PosGetScrew[5]]
    
        
        # # 拧钉区域
        # # 拧钉方向第一轴可动位置:
        # PosTwistScrewMoveJ = [-0.16429050266742706, 0.4578847289085388, 0.8128461837768555, 3.137009620666504, 1.5690369606018066, -1.57540762424469]
        # QNearTwistScrewMoveJ = [1.5593180656433105, -0.5359961986541748, 2.1596834659576416, 1.5134440660476685, 1.559828758239746, -1.575560450553894]


        # 螺钉入孔深度
        HeightScrewInto = 0.05
        
        # 入孔位置
        PosTwistScrew = [PosTwistScrewSlowDown[0] - HeightScrewInto,PosTwistScrewSlowDown[1],PosTwistScrewSlowDown[2],PosTwistScrewSlowDown[3],PosTwistScrewSlowDown[4],PosTwistScrewSlowDown[5]]

        # 协作臂回到默认位置（吸钉区域安全位置）
        duco.DucoMoveL(self.PosGetScrewStandby,vel_move,acc_move,self.QNearGetScrew)

        danikor.ClawCtrl(1)

        # 协作臂快速来到吸钉减速位置
        duco.DucoMoveL(PosGetScrewSlowDown,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂慢速来到吸钉位置
        duco.DucoMoveL(PosGetScrew,vel_end,acc_end,self.QNearGetScrew)

        # 控制电批反转认帽
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)
        danikor.ScrewMotorCtrl(2)
        time.sleep(0.5)

        # 第二次深入吸稳位置
        duco.DucoMoveL(PosGetScrewDownAgain,vel_end,acc_end,self.QNearGetScrew)

        danikor.ClawCtrl(0)

        # 协作臂慢速回到吸钉减速位置
        duco.DucoMoveL(PosGetScrewSlowDown,vel_end,acc_end,self.QNearGetScrew)

        # 协作臂来到吸钉退出位置
        duco.DucoMoveL(self.PosGetScrewLeave,vel_move,acc_move,self.QNearGetScrew)
        
        # 协作臂快速来到检测过渡位置
        duco.DucoMoveL(self.PosGetScrewGoToConfirm,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂来到吸钉检测位置
        duco.DucoMoveL(self.PosGetScrewConfirm,vel_move,acc_move,self.QNearGetScrew)

        # 等待检测
        IsScrewOk = danikor.ScrewConferm()
        if IsScrewOk == False:
            # 协作臂来到检测退出位置
            duco.DucoMoveL(self.PosGetScrewConfirmLeave,vel_move,acc_move,self.QNearGetScrew)
            return 0

        
        # 协作臂来到检测退出位置
        duco.DucoMoveL(self.PosGetScrewConfirmLeave,vel_move,acc_move,self.QNearGetScrew)

        # 协作臂快速来到吸钉方向第一轴可动位置
        duco.DucoMoveL(self.PosGetScrewMoveJ,vel_move,acc_move,self.QNearGetScrewMoveJ)
        
        #----------------------------------------------------------------------

        # 协作臂转动来到拧钉方向可旋转位置
        duco.DucoMoveJ(self.QNearTwistScrewMoveJ,vel_joint*4,acc_joint)

        # 协作臂来到扭转就绪位置
        duco.DucoMoveL(self.PosTwistScrewNormalRightStandby,vel_move,acc_move,self.QNearTwistScrewRight)

        # 协作臂来到扭转过渡位置
        duco.DucoMoveL(self.PosTwistScrewNormalRightStandbyToTarget,vel_end,acc_end,self.QNearTwistScrewRight)
        
        # 协作臂来到右侧拧钉就绪位置
        duco.DucoMoveL(self.PosTwistScrewRightStandby,vel_end,acc_end,self.QNearTwistScrewRight)
    
        # 协作臂快速来到拧钉减速位置
        duco.DucoMoveL(PosTwistScrewSlowDown,vel_move,acc_move,self.QNearTwistScrewRight)

        # 协作臂慢速来到拧钉位置
        duco.DucoMoveL(PosTwistScrew,vel_end,acc_end,self.QNearTwistScrewRight)

        danikor.ClawCtrl(1)

        # 控制电批拧入螺钉
        danikor.ScrewMotorCtrl(1)
        time.sleep(10)

        # 协作臂慢速回到拧钉减速位置
        duco.DucoMoveL(PosTwistScrewSlowDown,vel_end,acc_end,self.QNearTwistScrewRight)

        # 协作臂快速回到右侧拧钉就绪位置
        duco.DucoMoveL(self.PosTwistScrewRightStandby,vel_move,acc_move,self.QNearTwistScrewRight)

        # 协作臂来到扭转过渡位置
        duco.DucoMoveL(self.PosTwistScrewNormalRightStandbyToTarget,vel_end,acc_end,self.QNearTwistScrewRight)

        # 协作臂来到扭转就绪位置
        duco.DucoMoveL(self.PosTwistScrewNormalRightStandby,vel_end,acc_end,self.QNearTwistScrewRight)
    
        # 协作臂快速移动到轴动位置
        duco.DucoMoveL(self.PosTwistScrewMoveJ,vel_move,acc_move,self.QNearTwistScrewMoveJ)

        #---------------------------------------------------------------------------
        # 协作臂轴动转回至取钉轴动区域
        duco.DucoMoveJ(self.QNearGetScrewMoveJ,vel_joint*4,acc_joint)

        # 协作臂来到吸钉安全位置
        duco.DucoMoveL(self.PosGetScrewStandby,vel_move,acc_move,self.QNearGetScrew)

        return 2.56












