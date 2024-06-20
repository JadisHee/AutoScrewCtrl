import numpy as np

# 定义旋转矩阵
def rotation_matrix_x(rx):
    return np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

def rotation_matrix_y(ry):
    return np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

def rotation_matrix_z(rz):
    return np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

# 定义旋转矢量
rz, ry, rx = np.radians([45, 30, 60])  # 将角度转换为弧度

# 计算旋转矩阵
Rz = rotation_matrix_z(rz)
Ry = rotation_matrix_y(ry)
Rx = rotation_matrix_x(rx)

# 组合旋转矩阵 (顺序 [rx, ry, rz])
R = Rz @ Ry @ Rx

# 输出旋转矩阵
print("组合后的旋转矩阵 R:")
print(R)

# 测试向量
vector = np.array([1, 0, 0])

# 应用旋转矩阵
rotated_vector = R @ vector

# 输出旋转后的向量
print("旋转后的向量:")
print(rotated_vector)