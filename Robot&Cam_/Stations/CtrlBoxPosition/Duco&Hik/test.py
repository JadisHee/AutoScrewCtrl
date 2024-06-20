import numpy as np
import time
import math
import socket

from SocketCtrl import XmlData
from HikCtrl import HikCtrl
from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl

from StepProcess import StepProcess


def danikor_test():
    duco_ip = "192.168.1.16"
    duco_port = 7003
    bit_ip = "192.168.1.15"
    bit_port = 8888
    danikor = DanikorCtrl(duco_ip,duco_port,bit_ip,bit_port)

    danikor.ScrewMotorCtrl(2)

def test():
    duco_ip = "192.168.1.16"
    duco_port = 7003
    bit_ip = "192.168.1.15"
    bit_port = 8888
    danickor = DanikorCtrl(duco_ip,duco_port,bit_ip,bit_port)
     # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 获取本地主机名
    host = "192.168.1.225"
    port = 9999

    suc_data = "successed"
    fai_data = "failed"

    print("ip: ", host)
    # 绑定端口
    server_socket.bind((host, port))

    # 设置最大连接数，超过后排队
    server_socket.listen(5)

    print('等待客户端连接...')



    # 建立客户端连接
    client_socket, addr = server_socket.accept()
    print('连接地址：', addr)
    for i in range(2):
        data = client_socket.recv(1024)
        data = str(data, 'utf-8')
        print(data)

        if data == str('123'):
            result = danickor.ScrewMotorCtrl(2)
        elif data == str("456"):
            result = danickor.ScrewMotorCtrl(1)
        if result == 0:
            client_socket.send(fai_data.encode('utf-8'))
        else:
            client_socket.send(suc_data.encode('utf-8'))

if __name__ == '__main__':

    # danikor_test()
    test()
