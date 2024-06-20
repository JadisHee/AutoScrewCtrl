from SocketCtrl import XmlData
from StepProcess import StepProcess
import socket
import time
import threading
import xml.etree.ElementTree as ET

set_xml = XmlData()

process = StepProcess()

GetScrew_1 = [0.08552581816911697, -0.9865832924842834, 0.32483397221565247, 3.0731868743896484, 1.5638149976730347, 1.502327799797058]
GetScrew_2 = [0.03579577058553696, -0.9849782586097717, 0.3248028088092804, 3.075232744216919, 1.5638421773910522, 1.5044243335723877]
GetScrew_3 = [0.037906792014837265, -0.9349860548973083, 0.324820525937080383, 3.075066089630127, 1.563859462738037, 1.504248857498169]
GetScrew_4 = [0.08774872869253159, -0.9370216131210327, 0.3248088522052765, 3.0742757320404053, 1.5638936758041382, 1.5034552812576294]

TwistScrew_1 = [-0.5797485733032227, 0.3337847888469696, 0.09016480296850204, 1.5707998275756836, -5.1200686357333325e-06, 3.1415693759918213]
TwistScrew_2 = [-0.5797485733032227, 0.3330232799053192, -0.032647453248500824, 1.5713677406311035, 0.0009110721875913441, 3.1414732933044434]
TwistScrew_3 = [-0.5797485733032227, 0.9588111639022827, -0.01189726684242487, -1.5702458620071411, 0.0009518721490167081, -3.141524076461792]
TwistScrew_4 = [-0.5797485733032227, 0.9588069915771484, 0.11019549518823624, -1.5702608823776245, 0.0009298138902522624, -3.1414954662323]



# process.Screw_1(GetScrew_1,TwistScrew_1)
# time.sleep(0.5)
# process.Screw_2(GetScrew_3,TwistScrew_3)
# time.sleep(0.5)
# process.Screw_1(GetScrew_2,TwistScrew_2)
# time.sleep(0.5)
# process.Screw_2(GetScrew_4,TwistScrew_4)


def OneStep(Command):
    set_xml.TypeData = 1
    set_xml.SystemStatusData = "true"
    set_xml.CooperativeArmData = "true"
    set_xml.CooperativeArmCameraData = "true"
    set_xml.ConnectStateData = "true"
    

    if Command == 1:
        set_xml.ScrewStateData = "true"
        result = process.Screw_1(GetScrew_1,TwistScrew_1,set_xml)
    elif Command == 2:
        set_xml.ScrewStateData = "true"
        result = process.Screw_2(GetScrew_3,TwistScrew_3)
    elif Command == 3:
        result = process.Screw_1(GetScrew_2,TwistScrew_2)
    elif Command == 4:
        result = process.Screw_2(GetScrew_4,TwistScrew_4)

    # set_xml.ClampingForceData = result[0]
    # time.sleep(2)
    return result[0]




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