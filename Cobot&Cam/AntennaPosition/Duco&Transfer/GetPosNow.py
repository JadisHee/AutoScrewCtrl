import csv
from DucoCtrl import DucoCtrl


if __name__ == '__main__':
     # Duco机械臂的通讯地址
    DucoIp = "192.168.1.47"
    DucoPort = 7003

    duco = DucoCtrl(DucoIp,DucoPort)

     #     TcpPos = duco.GetDucoPos(1)
    FlangePos = duco.GetDucoPos(0)
    QNear = duco.GetQNear()

    filename = 'Duco&Transfer/Pose/PosNow.csv'

    with open(filename, mode = 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)

        csvwriter.writerow(FlangePos)
        csvwriter.writerow(QNear)

     