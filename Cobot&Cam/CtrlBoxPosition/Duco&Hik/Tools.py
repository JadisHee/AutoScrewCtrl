from scipy.spatial.transform import Rotation



class Tools:
    def __init__(self):
        pass

    def Euler2Quat(self,Rx,Ry,Rz):
        # 将欧拉角转换为四元数
        r = Rotation.from_euler('xyz', [Rx,Ry,Rz], degrees=False)
        quaternion = r.as_quat()


        print("Quaternion:", quaternion)
        return quaternion
    
    def Euler2RotMatrix(self,Rx,Ry,Rz):
        # 将欧拉角转换为旋转矩阵
        r = Rotation.from_euler('xyz', [Rx,Ry,Rz], degrees=True)
        rotation_matrix = r.as_matrix()
        print("Rotation Matrix:")
        print(rotation_matrix)
        return rotation_matrix