import numpy as np
from scipy.spatial.transform import Rotation

class Tools:
    def __init__(self):
        pass


    def Euler2RotMat(self,rx, ry, rz):
        """
        Convert Euler angles [Rx, Ry, Rz] to rotation matrix.

        Parameters:
            rx: float
                Rotation angle around X-axis in radians.
            ry: float
                Rotation angle around Y-axis in radians.
            rz: float
                Rotation angle around Z-axis in radians.

        Returns:
            rotation_matrix: numpy array
                3x3 rotation matrix.
        """
        # Calculate rotation matrices for each axis
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
        
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
        
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
        
        # Combine the rotation matrices
        rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))
        
        return rotation_matrix
    
    def PosTrans(self, PosNow, TransMat):
        RotMat = self.Euler2RotMat(PosNow[3],PosNow[4],PosNow[5])
        PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosNow[0]],
                           [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosNow[1]],
                           [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosNow[2]],
                           [0,0,0,1]])

        TargetPos = np.dot(PosMat, TransMat)

        # 变换为欧拉角
        TargetPosRotMat = np.array([[TargetPos[0][0],TargetPos[0][1],TargetPos[0][2]],
                                    [TargetPos[1][0],TargetPos[1][1],TargetPos[1][2]],
                                    [TargetPos[2][0],TargetPos[2][1],TargetPos[2][2]]])
        TargetPosRot_ = Rotation.from_matrix(TargetPosRotMat)
        TargetPosRot = TargetPosRot_.as_euler('xyz',degrees=False)
        TargetPosVec = [TargetPos[0][3],TargetPos[1][3],TargetPos[2][3],TargetPosRot[0],TargetPosRot[1],TargetPosRot[2]]

        return TargetPosVec
