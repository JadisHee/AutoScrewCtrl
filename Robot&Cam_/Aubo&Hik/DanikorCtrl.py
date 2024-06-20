from AuboClass import robotcontrol as RC
import socket
import time

class DanikorCtrl:
    def __init__(self):
        RC.Auboi5Robot.initialize()
        self.robot = RC.Auboi5Robot()
        pass

    def ClawCtrl(self,target_status):
        '''
        * Function:     ClawCtrl
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
        if target_status == 0:
            self.robot.set_board_io_status(5,"U_DO_03",0)
            self.robot.set_board_io_status(5,"U_DO_04",1)
            return 0
            # robot.set_board_io_status(5,"U_DO_04",0)
        elif target_status == 1:
            self.robot.set_board_io_status(5,"U_DO_04",0)
            self.robot.set_board_io_status(5,"U_DO_03",1)
            return 1

    def VacuumCtrl(self,target_status):
        '''
        * Function:     VacuumCtrl
        * Description:  对真空阀进行控制
        * Inputs:       想要控制的状态
                            0:关闭真空阀
                            1:打开真空阀
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:真空阀关闭
                            1:真空阀打开
                            2:气压异常
        * Notes:
        '''
        if target_status == 0:
            self.robot.set_board_io_status(5,"U_DO_05",0)
            return 0
        elif target_status == 1:
            # 设置启动io
            self.robot.set_board_io_status(5,"U_DO_05",1)
            # 等待两秒
            time.sleep(2)
            # 通过气压表io检测气压是否达标
            IsPressOk = self.robot.get_board_io_status(4,"U_DI_02")
            if IsPressOk == 0:
                return 1
            else:
                return 2
        
    def DriverCtrl(self,target_status):
        '''
        * Function:     DriverCtrl
        * Description:  对拧钉模组气缸进行控制
        * Inputs:       想要控制的状态
                            0:模组收回
                            1:模组伸出
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:模组已收回
                            1:模组已伸出
                            2:模组未运动到位
        * Notes:
        '''
        if target_status == 0:
            # 通过io控制模组收回
            self.robot.set_board_io_status(5,"U_DO_10",0)
            self.robot.set_board_io_status(5,"U_DO_07",1)
            
            # 循环读取到位模块，若到位则正常返回，若超过3s未到位，则异常返回
            for i in range(0,5,1):
                IsBackOk = self.robot.get_board_io_status(4,"U_DI_04")
                if IsBackOk == 1:
                    return 0
                else:
                    time.sleep(1)
                    if i > 3:
                        return 2
        elif target_status == 1:
            self.robot.set_board_io_status(5,"U_DO_07",0)
            self.robot.set_board_io_status(5,"U_DO_10",1)
            # 循环读取到位模块，若到位则正常返回，若超过3s未到位，则异常返回
            for i in range(0,5,1):
                IsBackOk = self.robot.get_board_io_status(4,"U_DI_03")
                if IsBackOk == 1:
                    return 1
                else:
                    time.sleep(1)
                    if i > 3:
                        return 2

    def ScrewMotorCtrl(self,ip,port):
        '''
        * Function:     ScrewMotorCtrl
        * Description:  对拧钉电批进行控制
        * Inputs:       ip:电批的ip地址
                        port:电批的端口号
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:与电批的通讯发生错误
                            1:通讯电批启动成功
        * Notes:
        '''

        MotorStart_Hex = "020000000A573033303130313D313B03"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            # 连接到目标设备
            s.connect((ip, port))
            
            # 将十六进制字符串转换为字节串
            byte_data = bytes.fromhex(MotorStart_Hex)
            
            # 发送数据
            s.sendall(byte_data)
            
            return 1
            # print("\n电批启动成功！")
            
        except Exception as e:

            return 0
            # print(f"\n电批启动时发生错误：{e}")
            
        finally:
            # 关闭连接
            s.close()


    def InitialAllMould(self):
        '''
        * Function:     InitialAllMould
        * Description:  初始化所有模块
                            拧钉模组气缸收回
                            真空阀关闭
                            夹爪张开
        * Inputs:
        * Outputs:      
        * Returns:      初始化结果
                            0: 初始化失败
                            1: 初始化成功
        * Notes:
        '''
        # 控制夹爪张开
        ClawStatus = self.ClawCtrl(1)

        # 控制真空阀关闭
        VacuumStatus = self.VacuumCtrl(0)

        # 控制拧钉模组气管收回
        DriverStatus = self.DriverCtrl(0)

        time.sleep(2)
        # 将所有io恢复0位
        # self.robot.set_board_io_status(5,"U_DO_03",0)
        # self.robot.set_board_io_status(5,"U_DO_04",0)

        # self.robot.set_board_io_status(5,"U_DO_05",0)

        # self.robot.set_board_io_status(5,"U_DO_07",0)
        # self.robot.set_board_io_status(5,"U_DO_10",0)

        if ClawStatus == 1 & VacuumStatus == 0 & DriverStatus == 0:
            return 1
        else:
            return 0
