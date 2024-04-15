import xml.etree.ElementTree as ET
import socket

class XmlData:
    # 启停，0为启动，1为运行中，2为停止
    TypeData = 2

    # 系统运行状态，True为正常，False为异常
    SystemStatusData = False

    # 相机状态，True为已检测，False为未检测
    CooperativeArmCameraData = False

    # 协作臂连接状态，True为已连接，False为未连接
    CooperativeArmData = False

    # 检测图像路径
    ImgPathData = ''

    # 电批模组连接状态，True为已连接，False为未连接
    ConnectStateData = False

    # 真空发生器状态，True为打开，False为关闭
    VacuumStateData = False

    # 螺钉状态检测，True为已吸上，False为未吸上
    ScrewStateData = False

    # 拧紧力度数值
    ClampingForceData = 0.0

    # 当前任务阶段
    StageData = ''

    # 当前任务阶段的步骤数
    StageNumData = 1

    # 错误信息
    ErrorData = ''


    def SetXmlData(self):
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

        Type.text = str(self.TypeData)
        SystemStatus.text = str(self.SystemStatusData)
        CooperativeArmCamera.text = str(self.CooperativeArmCameraData)
        CooperativeArm.text = str(self.CooperativeArmData)
        ImgPath.text = self.ImgPathData
        ConnectState.text = str(self.ConnectStateData)
        VacuumState.text = str(self.VacuumStateData)
        ScrewState.text = str(self.ScrewStateData)
        ClampingForce.text = str(self.ClampingForceData)
        Stage.text = self.StageData
        StageNum.text = str(self.StageNumData)
        Error.text = self.ErrorData

        xml_data = ET.tostring(Message,encoding='unicode')
        return xml_data
