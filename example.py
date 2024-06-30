
import torch
import os
import articulate as art
import numpy as np

def quaternion_to_rotation_matrix(quaternion):
    w, x, y, z = quaternion
    
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    
    return R

def realtime_display():
    # quaternion_matrix = np.ones((500, 24, 4))
    # quaternion_matrix[..., 1:] = 0
    quaternion_matrix =( np.random.rand(500, 24, 4)-0.5) * 0.1  # 小的随机数
    quaternion_matrix[..., 0] += 1  # 保证 w 较大
    rotation_matrix = np.zeros((500, 24, 3, 3))

    # 转换每一个四元数为旋转矩阵
    for i in range(500):
        for j in range(24):
            rotation_matrix[i, j] = quaternion_to_rotation_matrix(quaternion_matrix[i, j])
    pose_real = torch.from_numpy(rotation_matrix).float()
    o3dmodel = art.ParametricModel("smpl_model/basicModel_f_lbs_10_207_0_v1.0.0.pkl")
    o3dmodel.view_motion_while_prepare([pose_real])
    while 1:
        import random
        random_number = random.randint(0, 499)
        pose = pose_real[random_number:random_number+1]
        o3dmodel.view_motion_while_display([pose])


if __name__ == '__main__':
    realtime_display()
    # my_example()
 