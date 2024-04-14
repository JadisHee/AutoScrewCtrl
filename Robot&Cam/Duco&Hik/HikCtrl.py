import math
import numpy as np
import cv2
import socket

class HikCtrl:

    def __init__(self,ip,port):
        self.ip = ip
        self.port = port
        pass

    def SetHikSwitchPlan(self, SwitchCode, PlanName):
        '''
        * Function:     SetHikSwitchPlan
        * Description:  控制海康相机切换方案
        * Inputs:
                            SwitchCode: 切换语句
                            PlanName:   待切换方案的名称
        * Outputs:      切换方案成功
        * Returns:      
        * Notes:
        '''
        # 创建客户端
        HikClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 链接服务端

    
        HikClient.connect((self.ip, self.port))
        # 整理发送数据
        msgSend = SwitchCode + ' ' + PlanName
        # 发送数据
        while 1:
            HikClient.send(msgSend.encode('utf-8'))
            data = HikClient.recv(1024)
            # data1 = data.decode('utf-8')
            # print(data1[0])
            data = str(data, 'utf-8')
            # print(data[0])
            if data == str('ok'):
                print('成功切换至方案： ', PlanName)
                break

    def GetDataFromHik(self, StartMsg, Num):
        '''
        * Function:     GetDataFromHik
        * Description:  控制海康相机进行识别并获取检测结果
        * Inputs:
                            StartMsg:   切换语句
                            Num:        需要读取的数据个数，所有数据均需采用4.2长度设计，各数据含义需在SC MVS软件中自行注明
        * Outputs:      
        * Returns:      
        * Notes:        检测结果的数据
        '''
        # 创建客户端
        HikClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 链接客户端
        HikClient.connect((self.ip, self.port))
        # 读取客户端
        # Flag = True
        while 1:
            msgStart = StartMsg
            HikClient.send(msgStart.encode('utf-8'))
            data = HikClient.recv(1024)
            # data1 = data.decode('utf-8')
            # print(data1[0])
            data = str(data, 'utf-8')
            # print(data[0])
            if data[0] == str(1):
                print('收到智能相机数据')
                print(data)
                break
            # else:
            # sleep(0.1)
        Data = np.zeros((Num, 1))
        # print(Data)
        for i in range(Num):
            Data[i] = data[8 * i + 2+i:8 * i + 10+i]
        # np.array(Data)
        HikClient.close()
        return Data

    def QuartToRpy(self,x,y,z,w):
        '''
        * Function:     quart_to_rpy
        * Description:  对气动夹爪进行控制
        * Inputs:       想要控制的状态
                            0:闭合夹爪
                            1:张开夹爪
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:夹爪闭合
                            1:夹爪张开
        * Notes:
        '''
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw
    
    def GetRotVec2RotMat(RotVec):
        '''
        * Function:     GetRotVec2RotMat
        * Description:  将旋转向量转换为旋转矩阵
        * Inputs:       RotVec: 旋转向量
        * Outputs:      无输出
        * Returns:      旋转矩阵
        * Notes:
        '''
        RotMat = cv2.Rodrigues(RotVec)[0]
        return RotMat


    def GetTargetPos(self,width,height,disCamX,disCamY,RotationEndToCam,dRuler,PosNow,CirclePos):
        '''
        * Function:     GetTargetPos
        * Description:  计算目标点在机械臂下的位置
        * Inputs:       width: 图像的宽度
                        height: 图像的高度
                        disCamX: 电批头相对于相机的X轴偏移值
                        disCamY: 电批头相对于相机的X轴偏移值
                        RotationEndToCam: 末端到相机的旋转矩阵
                        dRuler: 比例尺
                        PosNow: 机械臂末端当前姿态
                        CirclePos: 孔在图像中的坐标
        * Outputs:      无输出
        * Returns:      目标孔位的位置坐标 list[list]
        * Notes:        
        '''
        
        # 计算旋转矩阵
        RotVec = np.array([PosNow[3], PosNow[4], PosNow[5]])
        RotBaseToEnd = self.GetRotVec2RotMat(RotVec)

        MoveX = (CirclePos[0] - width / 2) * dRuler + disCamX
        MoveY = (CirclePos[1] - height / 2) * dRuler + disCamY

        Move1 = np.array([[MoveX], [MoveY], [0]])
        Move1 = Move1 / 1000

        MoveCam = RotBaseToEnd @ RotationEndToCam @ Move1

        px = PosNow[0] + MoveCam[0]
        py = PosNow[1] + MoveCam[1]
        pz = PosNow[2]

        return [px[0],py[0],pz,PosNow[3], PosNow[4], PosNow[5]]
    