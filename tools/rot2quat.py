#!/usr/bin/env python3

#encoding:UTF-8

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
np.set_printoptions(precision=8)
np.set_printoptions(suppress=True)


def convert_eular(rotation):
    # EulerOrder::XYZ
    n = rotation[:,0]   # col(0)
    o = rotation[:,1]
    a = rotation[:,2]
    y = math.atan2(n[1], n[0])
    p = math.atan2(-n[2], n[0]*math.cos(y) + n[1]*math.sin(y))
    r = math.atan2(a[0]*math.sin(y) - a[1]*math.cos(y), -o[0]*math.sin(y) + o[1]*math.cos(y))
    print("-"*40)
    print("output Yaw(z) Pitch(y) Roll(x): ", math.degrees(y), math.degrees(p), math.degrees(r))
    return y,p,r

def main():    
    ## input
    rotation_matrix_12t6 = np.array([[-0.0115706,   -0.999739,   0.0196797], 
                                    [0.999887,  -0.0117557, -0.00931833], 
                                    [0.00954725,   0.0195697,    0.999763]])    

    t_ = np.array([-3.07985, 1.2, -0.300541])


    r_i = np.linalg.inv(rotation_matrix_12t6)
    # 旋转矩阵到四元数
    r3 = R.from_matrix(r_i)
    qua = r3.as_quat()

    t_n = -(np.dot(r_i, t_))
    # t_ = [-4.64862, 0.5, -0.195306]
    q_c = [0.00678763, 0.01542545, 0.70693851, 0.7070742] # x,y,z,w
    # 四元数到欧拉角
    r = R.from_quat(qua)
    euler0 = r.as_euler('ZYX', degrees=False)
    # compare
    convert_eular(r_i)

    r_c = R.from_quat(q_c)
    euler1 = r_c.as_euler('ZYX', degrees=False)
    rot_ = r_c.as_matrix()
    rot_1 = np.linalg.inv(rot_)
    # 旋转矩阵到四元数
    r4 = R.from_matrix(rot_1)
    euler2 = r4.as_euler('ZYX', degrees=False)

    rotation_lidar_pose = np.array([[-0.0115706,    0.999888,  0.00954725],
                                    [-0.99974,  -0.0117557,   0.0195697],
                                    [0.0196797, -0.00931833,    0.999763]])
    # 旋转矩阵到四元数
    r4 = R.from_matrix(rotation_lidar_pose)
    q_direct = r4.as_quat()

    print("q:( The returned value is in scalar-last (x, y, z, w) )", qua)
    print("t: ", t_n)
    print("e: ", euler0)
    print("e compare: ", euler1)
    print("q_d:( The returned value is in scalar-last (x, y, z, w) )", q_direct)

    print("euler init from pose to lidar:( The returned value is in scalar-last (x, y, z, w) )", euler2)



if __name__ == "__main__":
    main()
