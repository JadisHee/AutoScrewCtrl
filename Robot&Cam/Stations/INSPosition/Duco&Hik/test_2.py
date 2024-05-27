from CalcTools import CalcTools
import math
import numpy as np

tools = CalcTools()

V1 = [63.657833/1000,-218.59802/1000,77.2072/1000,1.5547558,0.15384263,0.13716835]
w = [1.5547558,0.15384263,0.13716835]

theta = math.sqrt(w[0]**2 + w[1]**2 + w[2]**2) 
u = [w[0]/theta,w[1]/theta,w[2]/theta]

u_T = np.array([[w[0]/theta,w[1]/theta,w[2]/theta]])
u = np.array([[w[0]/theta],
            [w[1]/theta],
               [w[2]/theta]])
U = np.dot(u,u_T)

# print("theta: ",theta)
# print("单位旋转向量: ", u_T)
# print("单位旋转向量: ", u)
# print(U)


I = np.array([[1,0,0],
              [0,1,0],
              [0,0,1]])



U_ = np.array([[0,-u[2][0],u[1][0]],
               [u[2][0],0,-u[0][0]],
               [-u[1][0],u[0][0],0]])
# print(U_)

R = math.cos(theta) * I + (1-math.cos(theta))*U + math.sin(theta)*U_

print(R)