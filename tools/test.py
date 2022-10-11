from markupsafe import string
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import math
import scipy.linalg as linalg
import csv
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)



# pose
def get_pose_data(path,t,ax,ay,az):
    # 读取外部数据
    print("-" * 40)
    print("read file: ", path)
    with open(path) as file:
        lines = csv.reader(file)
        head = next(lines)
        for line in lines:
            t.append(line[0][6:16])
            ax.append(float(line[2]))
            ay.append(float(line[3]))
            az.append(float(line[4]))
            # print(line[0], " ", line[2], " ", line[3], " ", line[4])
    return t,ax,ay,az

def plot_pose(t,x,y,z):
    plt.figure("x-y")
    plt.scatter(x, y, s = 2,c = 'r',alpha = .5)
    plt.grid(c='g')

    t = [int(i) for i in t]
    plt.figure("x")
    plt.scatter(t, x, s = 2,c = 'r',alpha = .5)
    plt.grid(c='g')

    plt.figure("y")
    plt.scatter(t, y, s = 2,c = 'r',alpha = .5)
    plt.grid(c='g')

    plt.figure("z")
    plt.scatter(t, z, s = 2,c = 'b',alpha = .5)
    plt.grid(c='g')

    plt.show()


def write_csv(t):
    with open('t.csv', 'w', encoding='utf-8', newline='') as file_obj:
        # 创建对象
        writer = csv.writer(file_obj)
        # # 写表头
        # writer.writerow(header)
        # 3.写入数据(一次性写入多行)
        # writer.writerow(t)
        for p in t:
            writer.writerow([p])

if __name__ == '__main__':
    pt = "/home/one/src/Apollo_Lib/deepway_apollo/debug_output/pose.csv"
    tt,x,y,z = get_pose_data(pt,[],[],[],[]) 
    write_csv(tt)
    plot_pose(tt,x,y,z) 