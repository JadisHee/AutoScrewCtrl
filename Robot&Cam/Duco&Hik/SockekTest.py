from SocketCtrl import XmlData
import socket
import time
import threading
import xml.etree.ElementTree as ET

set_xml = XmlData()
def test():
    
    set_xml.TypeData = 1
    time.sleep(2)

    set_xml.SystemStatusData = True
    set_xml.StageNumData = 1
    set_xml.StageData = '正在进行第一步'

    
    for i in range(4):
        set_xml.StageNumData = set_xml.StageNumData + 1
        if i == 0:
            set_xml.StageData = '正在进行第一阶段的第一步'
        elif i == 1:
            set_xml.StageData = '正在进行第二阶段的第一步'
        elif i == 2:
            set_xml.StageData = '正在进行第三阶段的第一步'
        elif i == 3:
            set_xml.StageData = '正在进行第四阶段的第一步'

        time.sleep(2)

        set_xml.VacuumStateData = True
        set_xml.StageNumData = set_xml.StageNumData + 1
        time.sleep(2)

        set_xml.ScrewStateData = True
        set_xml.StageNumData = set_xml.StageNumData + 1
        time.sleep(2)

        set_xml.ClampingForceData = 2.4124
        set_xml.VacuumStateData = False
        set_xml.StageNumData = set_xml.StageNumData + 1

        
    set_xml.TypeData = 2

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
        data = client_socket.recv(1024)
        # data = b''
        # while data != 0:
        #     chunk = client_socket.recv(1024)
        #     if not chunk:
        #         break
        #     data += chunk

        # Message = ET.fromstring(data)

        thd_test = threading.Thread(target=test)
        Message = ET.fromstring(data)
        for Type in Message.findall('Type'):
            if int(Type.text) == 0:
                thd_test.start()


        while thd_test.is_alive():

            # 向客户端发送数据
            message = str(set_xml.SetXmlData())
            client_socket.send(message.encode('utf-8'))
            time.sleep(1)

        # 关闭连接
        client_socket.close()
        break