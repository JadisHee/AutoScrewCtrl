import robotcontrol as RC
import time
import math
import numpy as np

# 系统初始化
RC.Auboi5Robot.initialize()
robot = RC.Auboi5Robot()


def ConnectToAubo(ipAddress, port):
    RC.logger_init()
    RC.logger.info("{0} connecting...".format(RC.Auboi5Robot.get_local_time()))
    handle = robot.create_context()
    RC.logger.info("robot.rsh={0}".format(handle))
    
    queue = RC.Queue()

    p = RC.Process(target=RC.runWaypoint, args=(queue,))
    p.start()
    RC.time.sleep(5)
    print("process started.")

    
    result = robot.connect(ipAddress, port)

    if result != RC.RobotErrorType.RobotError_SUCC:
        RC.logger.info("connect server{0}:{1} failed.".format(ipAddress, port))
    else:
        RC.logger.info("connect server{0}:{1} successed.".format(ipAddress, port))

def InitialAll():

    ClawCtrl(1)
    VacuumCtrl(0)
    DriverCtrl(0)

    time.sleep(3)
    # 待传感器就位后，将时延替换为缩回限位的判断

    robot.set_board_io_status(5,"U_DO_03",0)
    robot.set_board_io_status(5,"U_DO_04",0)

    robot.set_board_io_status(5,"U_DO_05",0)

    robot.set_board_io_status(5,"U_DO_07",0)
    robot.set_board_io_status(5,"U_DO_10",0)

def ClawCtrl(target_status):
    '''
    0:闭合夹爪
    1:张开夹爪
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_03",0)
        robot.set_board_io_status(5,"U_DO_04",1)
        # robot.set_board_io_status(5,"U_DO_04",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_04",0)
        robot.set_board_io_status(5,"U_DO_03",1)
        # robot.set_board_io_status(5,"U_DO_03",0)


def VacuumCtrl(target_status):
    '''
    0:关闭真空阀
    1:打开真空阀
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_05",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_05",1)

def DriverCtrl(target_status):
    '''
    0:模组收回
    1:模组伸出
    '''
    if target_status == 0:
        robot.set_board_io_status(5,"U_DO_10",0)
        robot.set_board_io_status(5,"U_DO_07",1)
        # robot.set_board_io_status(5,"U_DO_04",0)
    elif target_status == 1:
        robot.set_board_io_status(5,"U_DO_07",0)
        robot.set_board_io_status(5,"U_DO_10",1)

def IsVacuumPressOk():
    IsPressOk = robot.get_board_io_status(4,"U_DI_02")
    if IsPressOk == 0:
        return True
    else:
        return False

def IsMouldDownOk():
    IsDownOk = robot.get_board_io_status(4,"U_DI_03")
    if IsDownOk == 1:
        return True
    else:
        return False
    
def IsMouldUpOk():
    IsUpOk = robot.get_board_io_status(4,"U_DI_04")
    if IsUpOk == 1:
        return True
    else:
        return False

if __name__ == '__main__':

    # 设置Aubo的ip和port
    AuboIpAddress = '192.168.1.107'
    AuboPort = 8899

    # 连接至Aubo
    ConnectToAubo(AuboIpAddress,AuboPort)

    IsVacuumPressOk()
    # InitialAll()
    # DriverCtrl(1)
    # print("初始化模组！！！")
    # time.sleep(3)
    # InitialAll()
    # print("初始化完成！！！")
    # print("-------------------------------------")

    # print("伸出模组！！!")
    # time.sleep(3)
    # DriverCtrl(1)
    # print("夹爪张开完成！！！")
    # print("-------------------------------------")
    # 
    # print("准备张开夹爪！！!")
    # time.sleep(3)
    # ClawCtrl(1)
    # print("夹爪张开完成！！！")
    # print("-------------------------------------")

    # print("准备闭合夹爪！！!")
    # time.sleep(3)
    # ClawCtrl(0)
    # print("夹爪闭合完成！！！")
    # print("-------------------------------------")

    # print("准备开启吸钉！！!")
    # time.sleep(3)
    # VacuumCtrl(1)
    # print("开启吸钉完成！！!")
    # print("-------------------------------------")

    # print("准备关闭吸钉！！!")
    # time.sleep(3)
    # VacuumCtrl(0)
    # print("关闭吸钉完成！！!")
    # print("-------------------------------------")

    