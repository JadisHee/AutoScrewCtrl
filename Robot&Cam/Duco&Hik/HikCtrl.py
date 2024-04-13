import math
import numpy as np
import cv2

class HikCtrl:

    def __init__(self):
        pass

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

        return [px[0],py[0],pz]
    