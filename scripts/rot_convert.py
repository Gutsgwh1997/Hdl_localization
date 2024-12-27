import numpy as np
from pyquaternion import Quaternion


def rotation_matrix_to_quaternion(matrix):
    """
    Convert a rotation matrix to a quaternion.

    Parameters:
    matrix (numpy.ndarray): A 3x3 rotation matrix.

    Returns:
    Quaternion: A quaternion [w, x, y, z].
    """
    return Quaternion(matrix=matrix)

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.

    Parameters:
    roll (float): Rotation around the x-axis in radians.
    pitch (float): Rotation around the y-axis in radians.
    yaw (float): Rotation around the z-axis in radians.

    Returns:
    Quaternion: A quaternion [w, x, y, z].
    """
    q = Quaternion(axis=[1, 0, 0], angle=roll) * \
        Quaternion(axis=[0, 1, 0], angle=pitch) * \
        Quaternion(axis=[0, 0, 1], angle=yaw)
    return q 

if __name__ == "__main__":
    # 示例旋转矩阵
    rotation_matrix = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]
    ])

    # 转换为四元数
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    print("Rotation Matrix:")
    print(rotation_matrix)
    print("Quaternion [w, x, y, z]:")
    print(quaternion)

    

    print(euler_to_quaternion(0.0,0.0,-135/180*3.1415))

    