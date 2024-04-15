import socket
import xml.etree.ElementTree as ET

# 创建XML数据
Message = ET.Element("Message")
Type = ET.SubElement(Message,"Type")

Status = ET.SubElement(Message,"Status")
SystemStatus = ET.SubElement(Status,"SystemStatus")
CooperativeArmCamera = ET.SubElement(Status,"CooperativeArmCamera")
ImgPath = ET.SubElement(Status,"ImgPath")
CooperativeArm = ET.SubElement(Status,"CooperativeArm")

ElectricBatchModule = ET.SubElement(Status,"ElectricBatchModule")
ConnectState = ET.SubElement(ElectricBatchModule,"ConnectState")
VacuumState = ET.SubElement(ElectricBatchModule,"VacuumState")
ScrewState = ET.SubElement(ElectricBatchModule,"ScrewState")
ClampingForce = ET.SubElement(ElectricBatchModule,"ClampingForce")

Stage = ET.SubElement(Status,"Stage")
StageNum = ET.SubElement(Status,"StageNum")
Error = ET.SubElement(Status,"Error")




Type.text = str(0)
# SystemStatus.text = str(True)
# CooperativeArmCamera.text = str(True)
# CooperativeArm.text = str(True)
# ImgPath.text = "/home/jadis/pictures/DetectPics.pmb"
# ConnectState.text = str(True)
# VacuumState.text = str(True)
# ScrewState.text = str(True)
# ClampingForce.text = str(3.14)
# Stage.text = "正在前往料盒"
# StageNum.text = str(2)

# Error.text = "气压不足，请检查气压表"

xml_data = ET.tostring(Message)
# 设置服务器地址和端口
server_address = ('localhost', 9999)

# 创建TCP socket对象
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # 连接服务器
    sock.connect(server_address)

    # 发送XML数据
    sock.sendall(xml_data)

finally:
    # 关闭socket连接
    sock.close()
 