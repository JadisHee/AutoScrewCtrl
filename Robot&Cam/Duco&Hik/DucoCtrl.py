from array import array
import sys
import time
import threading

sys.path.append('DucoCobotAPI_py')

from DucoCobotAPI_py.DucoCobot import DucoCobot
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode,TaskState,Op,RealTimeControlData