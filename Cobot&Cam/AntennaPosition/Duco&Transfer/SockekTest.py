from SocketCtrl import XmlData
from StepProcess import StepProcess
import socket
import time
import threading
import xml.etree.ElementTree as ET
from TransferCtrl import TransferCtrl

transfer_ip = '192.168.1.xx'
transfer_port = 5700
StartSignal = '110,122'

set_xml = XmlData()
transfer = TransferCtrl(transfer_ip,transfer_port)
process = StepProcess()

def OneStep(Command):
    
    if Command == 1:
        process.GoToGetAntenna()
    elif Command == 2:
        process.TakeAntennaToConfirmPos()
        # TcpVec = transfer.GetDataFromTransfer(0,StartSignal)
    elif Command == 3:
        process.TakeTransferCamToPhotoPos()
        # TargetVec = transfer.GetDataFromTransfer(1,StartSignal)
    # elif Command == 4:
        # process.TakeAntennaToTarget(TargetVec,TcpVec)



if __name__ == '__main__':
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 获取本地主机名
    host = "192.168.1.100"
    port = 9999

    print("ip: ", host)
    # 绑定端口
    server_socket.bind((host, port))

    # 设置最大连接数，超过后排队
    server_socket.listen(5)

    print('等待客户端连接...')

    # 建立客户端连接
    client_socket, addr = server_socket.accept()
    print('连接地址：', addr)
    while True:
    
        # 接收数据
        data = client_socket.recv(1024)
        # data = b''
        # while data != 0:
        #     chunk = client_socket.recv(1024)
        #     if not chunk:
        #         break
        #     data += chunk

        # Message = ET.fromstring(data)

        
        Message = ET.fromstring(data)
        for Type in Message.findall('Type'):
            if int(Type.text) == 0:
                for Command in Message.findall('Command'):
                    thd_test = threading.Thread(target=OneStep,args=(int(Command.text),))
                    thd_test.start()
                    print("线程是否存活: ",thd_test.is_alive())


        
        while thd_test.is_alive():

            # 向客户端发送数据
            message = str(set_xml.SetXmlData())
            client_socket.send(message.encode('utf-8'))
            #  message.encode('utf-8')
            time.sleep(1)

        # # 关闭连接
        # client_socket.close()
        # break