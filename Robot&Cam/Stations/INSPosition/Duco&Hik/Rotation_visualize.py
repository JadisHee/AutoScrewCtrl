import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from CalcTools import CalcTools

tools = CalcTools()

def draw_axes(ax, origin, rotation_matrix, length=1.0, colors=('r', 'g', 'b'),system_label=''):
    """
    绘制3D坐标轴
    :param ax: 3D坐标轴对象
    :param origin: 坐标轴原点
    :param rotation_matrix: 旋转矩阵
    :param length: 坐标轴长度
    :param colors: 坐标轴颜色
    """
    axes = np.eye(3)  # 基础坐标轴
    transformed_axes = rotation_matrix @ axes  # 应用旋转矩阵
    for i in range(3):
        ax.quiver(*origin, *transformed_axes[:, i], length=length, color=colors[i], arrow_length_ratio=0.1)
    if system_label:
        ax.text(*origin, system_label, color='k', fontsize=12, ha='center', va='center')


# 创建一个新的图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 设置坐标轴范围
# ax.set_xlim([0, 2])
# ax.set_ylim([-2, 0])
# ax.set_zlim([0, 2])

ax.set_xlim([-2.5, 0])
ax.set_ylim([0, 2.5])
ax.set_zlim([-1.5, 1])


# Duco
T0 = np.eye(4)
draw_axes(ax, T0[:3, 3], T0[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='Duco_Base')

# Duco -- > Kuka
T1 = np.array([[ 0.69350914,0.72032586,0.01319006,-1.0707572 ],
                                [-0.72008126  ,0.69361943 ,-0.01938605 , 2.18730646],
                                [-0.02311231 , 0.00394696 , 0.99972512, -0.78090808],
                                [ 0.        ,  0.   ,       0.   ,       1.        ]])
draw_axes(ax, T1[:3, 3], T1[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='Kuka_Base')

# Kuka --> TargetKuka
V2 = [1032.7104/1000,-1131.4907/1000,905.4189/1000,179.59668*math.pi/180,3.2624285*math.pi/180,134.49574*math.pi/180]
T2 = tools.PosVecToPosMat(V2)

V3 = [0,0,0,90*math.pi/180,90*math.pi/180,0]
T3 = tools.PosVecToPosMat(V3)

T4 = np.dot(T2,T3)

# Duco --> TargetKuka
T5 = np.dot(T1,T4)

# Duco --> TargetUp
V6 = [-0.7798377275466919, 0.6467392444610596, 0.17206327617168427, 0.27459419179179356, 1.5635934209400553, -2.8965454018957812]
T6 = tools.PosVecToPosMat(V6)

# TargetUp --> Target
V7 = [0,0,0.275,0,0,0]
T7 = tools.PosVecToPosMat(V7)


T8 = np.dot(T6,T7)

draw_axes(ax, T5[:3, 3], T5[:3, :3], length=0.2, colors=('r', 'g', 'b'),system_label='Bit_Target')
# draw_axes(ax, T8[:3, 3], T8[:3, :3], length=0.2, colors=('r', 'g', 'b'),system_label='Bit_Target_')
# 设置标签和标题
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('KukaBase-INSTarget')

# 显示图形
plt.pause(5)

# 运行结束但保持窗口打开
input("Press [enter] to close the plot")

print("lalala")