from SocketCtrl import XmlData
import socket
import time
import threading
import xml.etree.ElementTree as ET


def test():
    XmlData.TypeData = 1
    time.sleep(2)

    XmlData.SystemStatusData = True
    XmlData.StageNumData = 1
    XmlData.StageData = '正在进行第一步'

    for i in 4:
        XmlData.StageNumData = XmlData.StageNumData + 1
        if i == 0:
            XmlData.StageData = '正在进行第一阶段的第一步'
        elif i == 1:
            XmlData.StageData = '正在进行第二阶段的第一步'
        elif i == 2:
            XmlData.StageData = '正在进行第三阶段的第一步'
        elif i == 3:
            XmlData.StageData = '正在进行第四阶段的第一步'

        time.sleep(2)

        XmlData.VacuumStateData = True
        XmlData.StageNumData = XmlData.StageNumData + 1
        time.sleep(2)

        XmlData.ScrewStateData = True
        XmlData.StageNumData = XmlData.StageNumData + 1
        time.sleep(2)

        XmlData.ClampingForceData = 2.4124
        XmlData.VacuumStateData = False
        XmlData.StageNumData = XmlData.StageNumData + 1

        
    XmlData.TypeData = 2

if __name__ == '__main__':
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 获取本地主机名
    host = socket.gethostname()
    port = 9999

    # 绑定端口
    server_socket.bind((host, port))

    # 设置最大连接数，超过后排队
    server_socket.listen(5)

    print('等待客户端连接...')

    while True:
        # 建立客户端连接
        client_socket, addr = server_socket.accept()
        print('连接地址：', addr)

        # 接收数据
        data = b''
        while True:
            chunk = client_socket.recv(1024)
            if not chunk:
                break
            data += chunk

        Message = ET.fromstring(data)
        for Type in Message.findall('Type'):
            
        # 向客户端发送数据
        message = '欢迎访问服务器！'
        client_socket.send(message.encode('utf-8'))

        # 关闭连接
        client_socket.close()