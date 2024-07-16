import socket
import threading
import math
import numpy as np

from DucoCtrl import DucoCtrl
# from DanikorCtrl import DanikorCtrl
# from HikCtrl import HikCtrl
from SocketCtrl import CommunicateData
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools


import xml.etree.ElementTree as ET

tools = CalcTools()
xml_data = CommunicateData()

duco_ip = "192.168.1.47"
duco_port = 7003
duco = DucoCtrl(duco_ip,duco_port)


transfer_ip = "192.168.1.10"
transfer_port = 5700
transfer = TransferCtrl(transfer_ip,transfer_port)

host = '192.168.1.101'
py_port = 9999

TransferCtrler_port = 9995


# M6螺钉的识别直径和螺纹孔的识别识别直径
M6ScrewDiameter = 8.80
M6HoleDiameter = 14.12
M4ScrewDiameter = 6.10
M4HoleDiameter = 14.12

transfer_command = '110,122'
# def cam_ctrler_():
#     # 创建socket对象
#     cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     # 绑定端口
#     cam_server.bind((host,cam_port))

#     # 设置最大连接数，超过后排队
#     cam_server.listen(1)
#     print('cam_ctrler: 等待协作臂连接 ! ! !')

#     cobot, addr_B = cam_server.accept()
#     print(f"cam_ctrler: 协作臂已连接，地址: {addr_B}")
    
#     while True:
#         print("cam_ctrler: 等待切换方案的信号 ! ! !")
#         data = cobot.recv(1024).decode('utf-8')
#         IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch', data)
#         if IsSwitchPlanSuccessful == 0:
#             print("cam_ctrler: 切换方案失败 ! ! !")
#             cobot.sendall("switch failed".encode('utf-8'))
#         else:
#             print("cam_ctrler: 成功切换到方案 " + str(data) + " ! ! !")
#             while True:
#                 print("cam_ctrler:  " + str(data) +" 等待检测触发检测指令")
#                 data_ = cobot.recv(1024).decode('utf-8')
#                 if data_ == "check":
#                     DPos = hik.GetHikDPos(M8ScrewDiameter)
#                     if DPos != 0:
#                         str_DPos = '(' + str(DPos[0]) + ',' + str(DPos[1]) + ')'
#                         cobot.sendall(str_DPos.encode('utf-8'))
#                     else:
#                         str_DPos = '(0,0)'
#                         cobot.sendall(str_DPos.encode('utf-8'))
#                         break
#                 elif data_ == "check_over":
#                     break


def transfer_ctrler():
    
    # 创建socket对象
    transfer_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    transfer_server.bind((host,TransferCtrler_port))

    # 设置最大连接数，超过后排队
    transfer_server.listen(1)
    print('transfer_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = transfer_server.accept()
    print(f"transfer_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("transfer_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        print('transfer_ctrler: 收到出发指令 ', data)
        if str(data) == 'GetTcp_1':
            # print('使用第一种获取tcp的方案')
            TcpVec = transfer.GetDataFromTransfer([0,0],transfer_command)
            send_data = '(' + str(TcpVec[0]) + ',' + str(TcpVec[1]) + ',' + str(TcpVec[2]) + ',' + str(TcpVec[3]) + ',' + str(TcpVec[4]) + ',' + str(TcpVec[5]) + ')'
            print('transfer_ctrler: 当前tcp为：',send_data)

            cobot.sendall(send_data.encode('utf-8'))
        elif str(data) == 'GetTarget_1':
            PosNow = duco.GetDucoPos(0)
            print('transfer_ctrler: 协作臂当前法兰姿态：', PosNow)
            
            # 从迁移获取相机的结果
            TargetVec = transfer.GetDataFromTransfer([1,0],transfer_command,PosNow=PosNow)

            # 计算目标点位上方的位置
            TargetMat = tools.PosVecToPosMat(TargetVec)
            UpVec = [0,0,-0.22,0,0,0]
            UpMat = tools.PosVecToPosMat(UpVec)
            UpTargetMat = np.dot(TargetMat,UpMat)
            UpTargetVec = tools.PosMatToPosVec(UpTargetMat)

            send_data = '(' + str(UpTargetVec[0]) + ',' + str(UpTargetVec[1]) + ',' + str(UpTargetVec[2]) + ',' + str(UpTargetVec[3]) + ',' + str(UpTargetVec[4]) + ',' + str(UpTargetVec[5]) + ')'

            
            print("transfer_ctrler: 目标位置为：",TargetVec)
        # elif str(data) == 'GetAngle_1':

        cobot.sendall(send_data.encode('utf-8'))





def py_ctrler():

    # 创建socket对象
    py_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    py_server.bind((host,py_port))

    # 设置最大连接数，超过后排队
    py_server.listen(2)
    print('py_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = py_server.accept()
    print(f"py_ctrler: 协作臂已连接，地址: {addr_B}")

    # ctrl_system, addr_A = py_server.accept()
    # print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

    while True:
        print("py_ctrler: 等待主控系统连接 ! ! !")

        ctrl_system, addr_A = py_server.accept()
        print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

        try:
            while True:
                print("py_ctrler: 等待主控系统指令 ! ! !")
                data = ctrl_system.recv(1024).decode('utf-8')
                if not data:  # 检查是否收到空数据，表示客户端已断开连接
                    print("py_ctrler: 主控系统断开连接")
                    break
                Message = ET.fromstring(data)


                for Command in Message.findall('Command'):                    
                    for ProductType in Message.findall('ProductType'):
                        for ProductNum in Message.findall('ProductNum'):
                            data_ = '(' + str(int(Command.text)) + ','  + str(int(ProductType.text)) + ',' + str(int(ProductNum.text)) + ')'
                            print('即将发送的消息为：',data_)
                            cobot.sendall(data_.encode('utf-8'))
                    print("指令发送完成，等待协作臂回传完成消息")
                    recv = cobot.recv(1024).decode('utf-8')
                    if int(recv) == 100:
                        xml_data.Error_Data = ""
                    elif int(recv) == 101:
                        xml_data.Error_Data = "101" #可循环五次
                    elif int(recv) == 102:
                        xml_data.Error_Data = "102" 
                    elif int(recv) == 103:
                        xml_data.Error_Data = "103"
                    
                    print("协作臂完成任务：" + str(recv) + " ! ! !")
                    
                    # xml_data.TypeData = Command.text
                    xml_data.Command_Data = Command.text
                    # xml_data.Error_Data = "no error"

                    ctrl_system.sendall(str(xml_data.XmlData()).encode('utf-8'))

                    # if int(Command.text) == 1:
                    #     # 一共有五种天线
                    #     # 第一种为：高度表发射天线×1
                    #     # 第二种为：高度表接收天线×1
                    #     # 第三种为：遥测天线×2
                    #     # 第四种为：安控天线×2
                    #     # 第五种为：GPS天线×1
                    #     for ProductType in Message.findall('ProductType'):
                    #         for ProductNum in Message.findall('ProductNum'):
                    #             data_ = '(' + str(int(ProductType.text)) + ',' + str(int(ProductNum.text)) + ')'
                    #             print('即将发送的消息为：',data_)
                    #             cobot.sendall(data_.encode('utf-8'))
                    # elif int(Command.text) == 2:
                        

                
                    
                    # print("指令发送完成，等待协作臂回传完成消息")
                    # recv = cobot.recv(1024).decode('utf-8')
                    # print("协作臂完成任务：" + str(recv) + " ! ! !")
                    
                    # xml_data.TypeData = Type.text
                    # xml_data.Command_Data = Command.text
                    # xml_data.Error_Data = "no error"

                    # ctrl_system.sendall(str(xml_data.XmlData()).encode('utf-8'))

                # cobot.sendall(data.encode('utf-8'))
                # print("py_ctrler: 指定发送 ", data)
        except Exception as e:
            print(f"py_ctrler: 主控系统通信错误: {e}")

        # 关闭当前主控系统连接，并重新等待
        ctrl_system.close()

        # print("py_ctrler: 等待主控系统指令 ! ! !")
        # data = ctrl_system.recv(1024).decode('utf-8')
        # cobot.sendall(data.encode('utf-8'))
        # print("py_ctrler: 指定发送 ", data)
       

def thread_ctrler():
    py_thread = threading.Thread(target=py_ctrler)
    # bit_thread = threading.Thread(target=bit_ctrler)
    transfer_thread = threading.Thread(target=transfer_ctrler)

    py_thread.start()
    # bit_thread.start()
    transfer_thread.start()

def show_pos():
    PosVec = duco.GetDucoPos(0)
    print(PosVec)

if __name__ == "__main__":
    # server()
    # danikor_test()
    # CamCtrler()
    thread_ctrler()
    # show_pos()