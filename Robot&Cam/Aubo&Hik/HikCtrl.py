import math

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
        
    