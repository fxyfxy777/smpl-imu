import numpy as np
import time 
import torch
import os
import articulate as art
import numpy as np
from utils import quaternion_to_rotation_matrix
from uart import UARTTable
import traceback
import chumpy
# scene = bpy.context.scene
# # 获取 'Armature' 骨骼对象
# armature = bpy.data.objects["骨架"]
# # 获取当前对象（这里假设是骨骼对象的所有者，可以根据实际情况修改）
# ob = bpy.context.active_object

# get the bones we need
# bone_upper_arm_R = armature.pose.bones.get("armUp.R")
# bone_lower_arm_R = armature.pose.bones.get("armDown.R")
# bone_upper_arm_L = armature.pose.bones.get("armUp.L")
# bone_lower_arm_L = armature.pose.bones.get("armDown.L")
# bone_trunk = armature.pose.bones.get("trunk.001")
# bone_head = armature.pose.bones.get("head.001")
# bone_upper_leg_R = armature.pose.bones.get("legUp.R")
# bone_lower_leg_R = armature.pose.bones.get("legDown.R")
# bone_upper_leg_L = armature.pose.bones.get("legUp.L")
# bone_lower_leg_L = armature.pose.bones.get("legDown.L")

# # 定义四元数共轭函数
# def quaternion_conjugate(q):
#     return np.array([q[0], -q[1], -q[2], -q[3]])

# 定义四元数乘法函数
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])



def multiplyQuaternion(q1, q0):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def setBoneRotation(bone, rotation):
    w, x, y, z = rotation
    if bone:
        bone.rotation_quaternion[0] = w
        bone.rotation_quaternion[1] = x
        bone.rotation_quaternion[2] = y
        bone.rotation_quaternion[3] = z

def xyz_exchange(array):
    t = array[2]
    array[2] = array[3]
    array[3] = t

    t = array[1]
    array[1] = array[3]
    array[3] = t
    return array

def updateAngles(angles, calibration_quaternions ):
    # abs_matrix = np.abs(np.array(angles))  # 取绝对值
    # if np.any(abs_matrix > 1):
    #     print(f"Invalid quaternion: {angles} 大于 1")
    #     return 0

    # print("angles, calibration_quaternions: ",angles, calibration_quaternions )
    calibrated_angles = []
    for i, angle in enumerate(angles):
        calibrated_angle = quaternion_multiply(calibration_quaternions[i], angle)
        calibrated_angles.append(calibrated_angle)
#    print("calibrated_angles",calibrated_angles)
    upperarmR_out = calibrated_angles[0]
    lowerarmR_out = calibrated_angles[1]
    upperarmL_out = calibrated_angles[2]
    lowerarmL_out = calibrated_angles[3]
    trunk_out = calibrated_angles[4]
    upperLegR_out = calibrated_angles[5]
    lowerLegR_out = calibrated_angles[6]
    upperLegL_out = calibrated_angles[7]
    lowerLegL_out = calibrated_angles[8]
    head_out = calibrated_angles[9]
    # lowerarmR_out = calibrated_angles[0]
    # upperarmR_out = calibrated_angles[1]
    # lowerarmL_out = calibrated_angles[2]
    # upperarmL_out = calibrated_angles[3]
    # trunk_out = calibrated_angles[4]
    # upperLegR_out = calibrated_angles[5]
    # lowerLegR_out = calibrated_angles[6]
    # upperLegL_out = calibrated_angles[7]
    # lowerLegL_out = calibrated_angles[8]
    # head_out = calibrated_angles[9]
    
    trunk_inv = trunk_out * np.array([1, -1, -1, -1])
    head_rel = multiplyQuaternion(trunk_inv, head_out)
    upperarmR_inv = upperarmR_out * np.array([1, -1, -1, -1])
    lowerarmR_rel = multiplyQuaternion(upperarmR_inv, lowerarmR_out)
    upperarmL_inv = upperarmL_out * np.array([1, -1, -1, -1])
    lowerarmL_rel = multiplyQuaternion(upperarmL_inv, lowerarmL_out)
    upperLegR_inv = upperLegR_out * np.array([1, -1, -1, -1])
    lowerLegR_rel = multiplyQuaternion(upperLegR_inv, lowerLegR_out) * np.array([1, -1, -1, 1])
    upperLegL_inv = upperLegL_out * np.array([1, -1, -1, -1])
    lowerLegL_rel = multiplyQuaternion(upperLegL_inv, lowerLegL_out) * np.array([1, -1, -1, 1])

    angles = {}

        
    angles['RSHOULDER'] = quaternion_to_rotation_matrix(xyz_exchange(upperarmR_out * np.array([1, -1, -1, 1])))
    angles['RELBOW'] = quaternion_to_rotation_matrix(xyz_exchange(lowerarmR_rel * np.array([1, -1, -1, 1])) ) 
    # angles['RELBOW'] = quaternion_to_rotation_matrix(xyz_exchange(lowerarmR_rel ) ) 
    angles['LSHOULDER'] = quaternion_to_rotation_matrix(xyz_exchange(upperarmL_out))
    angles['LELBOW'] = quaternion_to_rotation_matrix(xyz_exchange(lowerarmL_rel))

    angles['SPINE'] = quaternion_to_rotation_matrix(trunk_out)

    angles['RHIP'] = quaternion_to_rotation_matrix(upperLegR_out * np.array([1, -1, -1, 1]) )
    angles['RKNEE'] = quaternion_to_rotation_matrix(lowerLegR_rel)
    angles['LHIP'] = quaternion_to_rotation_matrix(upperLegL_out * np.array([1, -1, -1, 1]) )
    angles['LKNEE'] = quaternion_to_rotation_matrix(lowerLegL_rel)

    angles['HEAD'] = quaternion_to_rotation_matrix(head_rel)
    return angles

class SMPL_display():
    def __init__(self) -> None:
        self.model = art.ParametricModel("smpl_model/basicmodel_m_lbs_10_207_0_v1.0.0.pkl")
        FPS = 60
        print("请输入port:")
        port = input()
        if not port:
            port = 'COM6'  # 指定串口号，根据实际情况修改
        baudrate = 256000  # 波特率，根据实际情况修改
        print("请输入port_wit:")
        port_wit = input()
        if not port_wit:
            port_wit = 'COM18'
        baudrate_wit = 9600  # 波特率，根据实际情况修改
        self.uart_table = UARTTable(port=port, baudrate=baudrate, port_wit = port_wit,baudrate_wit = baudrate_wit)
        self.uart_table.startThreaded()
        # 定义四元数 (1, 0, 0, 0)
        # rotation_matrix = np.eye(3).reshape(1, 3, 3).repeat(24, axis=0)
        rotation_matrix = np.eye(3).reshape(1, 3, 3)
        self.debug = 1

        # 定义24个关节名称
        joints = [
            "SPINE", "LHIP", "RHIP", "SPINE1", "LKNEE", "RKNEE", "SPINE2", 
            "LANKLE", "RANKLE", "SPINE3", "LFOOT", "RFOOT", "NECK", "LCLAVICLE", "RCLAVICLE", 
            "HEAD", "LSHOULDER", "RSHOULDER", "LELBOW", "RELBOW", "LWRIST", "RWRIST", "LHAND", 
            "RHAND"
        ]
        # joints = [
        #     "ROOT", "PELVIS", "SPINE", "LHIP", "RHIP", "SPINE1", "LKNEE", "RKNEE", "SPINE2", 
        #     "LANKLE", "RANKLE", "SPINE3", "LFOOT", "RFOOT", "NECK", "LCLAVICLE", "RCLAVICLE", 
        #     "HEAD", "LSHOULDER", "RSHOULDER", "LELBOW", "RELBOW", "LWRIST", "RWRIST", "LHAND", 
        #     "RHAND"
        # ]
        self.joint_map = {joint: rotation_matrix for joint in joints}
        self.joints_to_assign = [
            "RSHOULDER", "RELBOW", "LSHOULDER", "LELBOW", "SPINE", "RHIP", "RKNEE", "LHIP", "LKNEE", "HEAD"]
        self.initok = False
    
    def smpl_model_init(self):
        while 1:
            rotation_matrix_seq = np.eye(3).reshape(1, 1, 3, 3).repeat(500, axis=0).repeat(24, axis=1)
            for i in range(500):
                angles_uart = self.uart_table.get("angles")
                calibration_quaternions = self.uart_table.get("calibration_quaternions")
                if angles_uart and len(angles_uart)== 10  and len(calibration_quaternions) == 10 :
                    angles_update = updateAngles(angles_uart, calibration_quaternions)
                    if angles_update != 0:
                        angles_temp = angles_update
                        for joint, quat in zip(self.joints_to_assign, angles_temp):#赋值10个需要更改的关节
                            self.joint_map[joint] = angles_temp[joint]
                        joint_matrices = np.zeros((24, 3, 3))#将24关节的数据存起来
                        for idx, joint in enumerate(self.joint_map):
                            joint_matrices[idx] = self.joint_map[joint][0]
                        rotation_matrix_seq[i] = joint_matrices
                    else:
                        if not i:
                            pass
                        else:
                            rotation_matrix_seq[i] = rotation_matrix_seq[i-1]
                        continue
                        # for joint, quat in zip(self.joints_to_assign, angles_temp):#赋值10个需要更改的关节
                        #     self.joint_map[joint] = angles_temp[joint]
                else:
                    time.sleep(1)
                    print("初始化中...")
                    self.initok = False
                    break
            rotation_matrix_seq = torch.from_numpy(rotation_matrix_seq).float()
            self.model.view_motion_while_prepare([rotation_matrix_seq])#pose_real.shape torch.Size([500, 24, 3, 3])为了获取均值和获取mesh
            self.initok = True
            print("初始化成功！")
            break
            
    
    def smpl_model_realtime(self):
        angles_temp = None
       
        while 1:
            try:
                angles_uart = self.uart_table.get("angles")
                calibration_quaternions = self.uart_table.get("calibration_quaternions")
                if angles_uart and len(angles_uart)== 10  and len(calibration_quaternions) == 10 and self.initok:
                    if self.debug == 50:
                        print(angles_uart)
                        print('')
                        self.debug = 0
                    else:
                        self.debug+=1
                    angles_update = updateAngles(angles_uart, calibration_quaternions)
                    if angles_update:
                        angles_temp = angles_update
                        for joint, quat in zip(self.joints_to_assign, angles_temp):#赋值10个需要更改的关节
                            if sum(sum(angles_temp[joint])) != 3:#排除掉突然闪出的1000
                                self.joint_map[joint] = angles_temp[joint]
                    else:
                        print("uart数据异常")
                        continue
                    joint_matrices = np.zeros((24, 3, 3))#将24关节的数据存起来
                    for idx, joint in enumerate(self.joint_map):
                        joint_matrices[idx] = self.joint_map[joint]
                    joint_matrices = torch.from_numpy(joint_matrices).float()
                    self.model.view_motion_while_display([joint_matrices])
                else:
                    if not self.initok:
                        time.sleep(1)
                        print("初始化失败！")
                    pass
            except:
                traceback.print_exc()


if __name__ == "__main__":
    smpl = SMPL_display()
    smpl.smpl_model_init()
    smpl.smpl_model_realtime()


