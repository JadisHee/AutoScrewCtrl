import socket
import time

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
            PosDataSend = '172'+ str(PosNow[0]) + str(PosNow[1]) + str(PosNow[2]) + str(PosNow[3]) + str(PosNow[4]) + str(PosNow[0])
        else:
            print('方案选择输入错误 ! ! !')
            return 0
        # 指令迁移相机切换方案
        TransferClient.send(ModCode.encode('utf-8'))
        FeedBack = TransferClient.recv(1024)
        FeedBackString = FeedBack.decode()
        # 返回'160'为切换成功，'001'为失败
        if FeedBackString == '001':
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
                    return 0
                
            # 给相机发送启动指令
            TransferClient.send(StartSignal.encode('utf-8'))
            FeedBack = TransferClient.recv(1024)
            FeedBackString = FeedBack.decode()
            if FeedBackString[0] == '001': 
                print("检测成功! ! !")
                DataList = FeedBackString.split(",")[:-1]

                UsefulData = [int(DataList[0])]
                UsefulData.extend(float(i) for i in DataList[1:])
                Data = UsefulData[1:]
                return Data
            else:
                TimeEnd = time.time()
                RunTime = TimeEnd - TimeStart
                if RunTime >= 3:
                    print("检测失败")
                    break
        return 0








