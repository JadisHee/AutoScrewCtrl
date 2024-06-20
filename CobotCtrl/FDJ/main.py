import socket
import threading
import math

from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl
from HikCtrl import HikCtrl


duco_ip = "192.168.1.16"
duco_port = 7003
duco = DucoCtrl(duco_ip, duco_port)

bit_ip = "192.168.1.15"
bit_port = 8888
danikor = DanikorCtrl(duco_ip, duco_port, bit_ip, bit_port)

hik_ip = "192.168.1.17"
hik_port = 8192
hik = HikCtrl(hik_ip, hik_port)

host = '192.168.1.225'
py_port = 9999

cam_port = 9995
bit_port = 9996

# M6螺钉的识别直径和螺纹孔的识别识别直径
M6ScrewDiameter = 8.80
M6HoleDiameter = 14.12
M4ScrewDiameter = 6.10
M4HoleDiameter = 14.12


def cam_ctrler():
    
    # 创建socket对象
    cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    cam_server.bind((host,cam_port))

    # 设置最大连接数，超过后排队
    cam_server.listen(1)
    print('cam_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = cam_server.accept()
    print(f"cam_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("cam_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        # print("收到的信息: ", data)
        # 切换方案
        IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch', data)
        if IsSwitchPlanSuccessful == 0:
            print("cam_ctrler: 切换方案失败")
            return 0
        if data == "M6FindScrew":
            DPos = hik.GetHikDPos(M6ScrewDiameter)
        elif data == "M6FindHole":
            DPos = hik.GetHikDPos(M6HoleDiameter)
        elif data == "M4FindScrew":
            DPos = hik.GetHikDPos(M4ScrewDiameter)
        elif data == "M4FindHole":
            DPos = hik.GetHikDPos(M4HoleDiameter)
        
        if DPos != 0:
            print("没寄")
            str_DPos = '(' + str(DPos[0]) + ',' + str(DPos[1]) + ')'
        # print("cam_ctrler: DPos: ", str_DPos)
            cobot.sendall(str_DPos.encode('utf-8'))
        else:
            str_DPos = '(0,0)'
            print("寄",str_DPos)
            cobot.sendall(str_DPos.encode('utf-8'))

        # # 将相机移动至目标中心，直到接近中心后退出循环
        # while True:
        #     DPos = hik.GetHikDPos(M6Diameter)
        #     dist = math.sqrt(DPos[0] ** 2 + DPos[1] ** 2)
        #     if dist <= 0.002:
        #         break

def bit_ctrler():
    # 创建socket对象
    bit_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    bit_server.bind((host,bit_port))

    # 设置最大连接数，超过后排队
    bit_server.listen(1)
    print('bit_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = bit_server.accept()
    print(f"bit_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("bit_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        # print("收到的信息: ", data)
        # 在模组下降至中间位置后,接收旋转信号控制电批反转与批头锁紧
        # signal = cobot_socket.recv(1024).decode('utf-8')
        # print("recv signal: ", data)
        if data == "rot_bit_inv":
            danikor.ScrewMotorCtrl(2)
        elif data == "M6_rot_bit":
            result = danikor.ScrewMotorCtrl(3)
            cobot.sendall(str(result[0]).encode('utf-8'))
            print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")
        elif data == "M4_rot_bit":
            result = danikor.ScrewMotorCtrl(1)
            cobot.sendall(str(result[0]).encode('utf-8'))
            print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")

def py_ctrler():

    # 创建socket对象
    py_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    py_server.bind((host,py_port))

    # 设置最大连接数，超过后排队
    py_server.listen(2)
    print('py_ctrler: 等待客户端连接 ! ! !')

    cobot, addr_B = py_server.accept()
    print(f"py_ctrler: 协作臂已连接，地址: {addr_B}")

    ctrl_system, addr_A = py_server.accept()
    print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")
    
    

    while True:
        print("py_ctrler: 等待主控系统指令 ! ! !")
        data = ctrl_system.recv(1024).decode('utf-8')
        cobot.sendall(data.encode('utf-8'))
        print("py_ctrler: 指定发送 ", data)
        # end_signal = cobot.recv(1024).decode('utf-8')
        # print("EndSignal: ", end_signal)
        # 控制从快换处取批头
        # if data == "GM4" or data == "GM6":
            
        #     if end_signal != "GetBitFinished":
        #         duco.DucoStop()     
        
        # # 获取螺钉
        # elif data == "GScrew":
            
        #     if end_signal != "GetScrewFinished":
        #         duco.DucoStop()

        # elif data == "first" or data == "second" or data == "third" or data == "fourth":
        #     if end_signal != data + "Finished":
        #         duco.DucoStop()


def thread_ctrler():
    py_thread = threading.Thread(target=py_ctrler)
    bit_thread = threading.Thread(target=bit_ctrler)
    cam_thread = threading.Thread(target=cam_ctrler)

    py_thread.start()
    bit_thread.start()
    cam_thread.start()

def show_pos():
    PosVec = duco.GetDucoPos(1)
    print(PosVec)

if __name__ == "__main__":
    # server()
    # danikor_test()
    # CamCtrler()
    thread_ctrler()
    # show_pos()