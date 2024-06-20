import numpy as np
from scipy.spatial.transform import Rotation

class CalcTools:
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
    
    def PosVecToPosMat(self,PosVec):
        RotMat = self.Euler2RotMat(PosVec[3],PosVec[4],PosVec[5])
        PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosVec[0]],
                           [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosVec[1]],
                           [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosVec[2]],
                           [0,0,0,1]])

        return PosMat

    def PosMatToPosVec(self,PosMat):
        RotMat = np.array([[PosMat[0][0],PosMat[0][1],PosMat[0][2]],
                           [PosMat[1][0],PosMat[1][1],PosMat[1][2]],
                           [PosMat[2][0],PosMat[2][1],PosMat[2][2]]])
        RotVec_ = Rotation.from_matrix(RotMat)
        RotVec = RotVec_.as_euler('xyz',degrees=False)

        PosVec = [PosMat[0][3],PosMat[1][3],PosMat[2][3],RotVec[0],RotVec[1],RotVec[2]]

        return PosVec

    def PosTrans(self, PosNow, TransMat):
        # RotMat = self.Euler2RotMat(PosNow[3],PosNow[4],PosNow[5])
        # PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosNow[0]],
        #                    [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosNow[1]],
        #                    [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosNow[2]],
        #                    [0,0,0,1]])

        PosMat = self.PosVecToPosMat(PosNow)

        TargetPos = np.dot(PosMat, TransMat)

        # 变换为欧拉角
        TargetPosVec = self.PosMatToPosVec(TargetPos)

        return TargetPosVec
