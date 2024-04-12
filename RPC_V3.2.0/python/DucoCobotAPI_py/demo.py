from array import array
import sys
import time
import threading

sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode,TaskState,Op,RealTimeControlData


ip='127.0.0.1'
stopheartthread = False


def hearthread_fun():
    duco_heartbeat = DucoCobot(ip, 7003)
    duco_heartbeat.open()
    while not stopheartthread:
        duco_heartbeat.rpc_heartbeat()
        time.sleep(1)
    duco_heartbeat.close()
    

def thread_fun():
    duco_cobot = DucoCobot(ip, 7003)
    # Connect!
    duco_cobot.open()
    while not stopheartthread:
        tcp_pose = []
        tcp_pose = duco_cobot.get_robot_state()
        print("state: ", tcp_pose)
        time.sleep(1)
    duco_cobot.close()

def main():
    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    thd_B = threading.Thread(target=hearthread_fun)
    thd_B.start()
    
    duco_cobot = DucoCobot(ip,7003)
    op = Op()
    op.time_or_dist_1 = 0
    op.trig_io_1 = 1
    op.trig_value_1 = False
    op.trig_time_1 = 0.0
    op.trig_dist_1 = 0.0
    op.trig_event_1 = ''
    op.time_or_dist_2 = 0
    op.trig_io_2 = 1
    op.trig_value_2 = False
    op.trig_time_2 = 0.0
    op.trig_dist_2 = 0.0
    op.trig_event_2 = ''

    # Connect!
    rlt = duco_cobot.open()
    print("open:", rlt)
    rlt = duco_cobot.power_on(True)
    print("power_on:", rlt)
    rlt = duco_cobot.enable(True)
    print("enable:", rlt)
    rlt = duco_cobot.set_tool_data("default",[0,0,0,0,0,0],[1,0,0,0],[0,0,0,0,0,0])
    print("set tool:", rlt)
    rlt = duco_cobot.movej2([0,0,1.57,0,-1.57,0],1.0,1.0,0,True)
    print("movej2", rlt)
    rlt = duco_cobot.tcp_move([0.35,0.09,0.2,1.7,0.45,-0.13],0.5,0.5,0,"default",True)
    print("tcp move:",rlt)

    input()
    stopheartthread = True
    rlt = duco_cobot.close()
    print("close:", rlt)

if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
