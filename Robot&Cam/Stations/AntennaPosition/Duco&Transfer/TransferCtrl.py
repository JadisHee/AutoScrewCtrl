import socket
import time
import math

class TransferCtrl:
    def __init__(self,ip,port):
        
        self.ip = ip
        self.port = port
        
        self.mod = [
            '160,1,0',
            '160,2,0'
                    ]
        
        pass

    def GetDataFromTransfer(self,DetectMod,StartSignal,PosNow=None):
        '''
            * Function:     GetTcpFromTransfer
            * Description:  控制固定机位的迁移相机对工件底部进行识别并返回姿态结果
            * Inputs:
                                DetectMod:     检测模式
                                    0: 眼在手外确认Tcp方案
                                    1: 眼在手上确认目标位置方案
                                StartSignal:   开始信号
            * Outputs:      
            * Returns:      0: 超过3s未检测目标
                            data: 检测结果
            * Notes:        
        '''
        # 与迁移相机建立连接
        TransferClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        TransferClient.connect((self.ip,self.port))

        
        
        # 
        if DetectMod == 0:
            ModCode = self.mod[0]    
        elif DetectMod == 1 and PosNow is not None:
            ModCode = self.mod[1]
            PosDataSend = '172,'+ str(PosNow[0]*1000) + ',' + str(PosNow[1]*1000) + ',' + str(PosNow[2]*1000) + ',' + str(PosNow[5]*180/math.pi) + ',' + str(PosNow[4]*180/math.pi) + ',' + str(PosNow[3]*180/math.pi)
        else:
            print('方案选择输入错误 ! ! !')
            return 0
        # 指令迁移相机切换方案
        TransferClient.send(ModCode.encode('utf-8'))
        FeedBack = TransferClient.recv(1024)
        FeedBackString = FeedBack.decode()
        # 返回'160'为切换成功，'001'为失败
        if FeedBackString == '001':
            print('切换失败 ! ! !')
            return 0

        # 开始时间
        TimeStart = time.time()
        while 1:
            # 若检测模式为检测放置点，则需要先发送当前位姿
            if DetectMod == 1:
                TransferClient.send(PosDataSend.encode('utf-8'))
                FeedBack = TransferClient.recv(1024)
                FeedBackString = FeedBack.decode()
                # 返回'172'为切换成功，'001'为失败
                if FeedBackString == '001':
                    print('切换失败 ! ! !')
                    return 0
                
            # 给相机发送启动指令
            TransferClient.send(StartSignal.encode('utf-8'))
            FeedBack = TransferClient.recv(1024)
            FeedBackString = FeedBack.decode()
            if FeedBackString[0] == '2' or FeedBackString[0] == '1': 
                print("检测成功! ! !")
                DataList = FeedBackString.split(",")[:]

                UsefulData = [int(DataList[0])]
                UsefulData.extend(float(i) for i in DataList[1:])
                Data = UsefulData[1:]
                Data[0] = Data[0]/1000
                Data[1] = Data[1]/1000
                Data[2] = Data[2]/1000
                Data[3] = Data[3]*math.pi/180
                Data[4] = Data[4]*math.pi/180
                Data[5] = Data[5]*math.pi/180
                return Data
            else:
                TimeEnd = time.time()
                RunTime = TimeEnd - TimeStart
                if RunTime >= 10:
                    print("检测失败")
                    break
        return 0








