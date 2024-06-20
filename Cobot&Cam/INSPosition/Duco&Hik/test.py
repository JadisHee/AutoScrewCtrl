from CalcTools import CalcTools
import math
import numpy as np

tools = CalcTools()

# Kuka_Flange --> Cam
V1 = [63.657833/1000,-218.59802/1000,77.2072/1000,1.5547558,0.15384263,0.13716835]
# T1 = tools.PosVecToPosMat(V1)

T1 = np.array([[0.982771,0.009544,0.18458,0.06365783],
               [0.184463,0.012036,-0.982765,-0.218598],
               [-0.011601,0.999882,0.0100685,0.0772072],
               [0,0,0,1]])
V1_ = tools.PosMatToPosVec(T1)
print("V1: \n", V1)
print("V1_: \n", V1_)
print("Kuka_Flange --> Cam: \n", T1)
# print("相机在库卡法兰的变换矩阵: \n", T1_)

# Kuka_Base --> Kuka_Flange
V2 = [1015.69/1000,-220.51/1000,1191.08/1000,-179.48*math.pi/180,-2.38*math.pi/180,-48.36*math.pi/180]
T2 = tools.PosVecToPosMat(V2)
print("Kuka_Base --> Kuka_Flange: \n", T2)

# Duco_Base --> Cam
V3 = [-297.01443/1000,1197.834/1000,308.5733/1000,1.9497347,-2.4527433,2.4921033]
# T3 = tools.PosVecToPosMat(V3)
T3 = np.array([[-0.25941,-0.020083,0.96555686,-0.29701443],
               [-0.965115,-0.03130886,-0.25994825,1.197834],
               [0.035451,-0.999308,-0.011260,0.3085733],
               [0,0,0,1]])
print("Duco_Base --> Cam: \n", T3)

# Kuka_Base --> Cam
T4 = np.dot(T2,T1)
# print("库卡基座到相机的变换矩阵: \n", T4)


# Duco_Base --> Kuka_Base
T6 = np.dot(T3,np.linalg.inv(T4))
# print(T6)
V6 = tools.PosMatToPosVec(T6)

print("Duco_Base --> Kuka_Base: \n",T6)