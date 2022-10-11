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
            t.append(float(line[0]))
            ax.append(float(line[2]))
            ay.append(float(line[3]))
            az.append(float(line[4]))
    return t,ax,ay,az

def plot_pose(t,x,y,z):
    plt.scatter(x, y, s = 2,c = 'r',alpha = .5)
    plt.show()

if __name__ == '__main__':
    pt = "/home/one/src/Apollo_Lib/deepway_apollo/debug_output/pose.csv"
    tt,x,y,z = get_pose_data(pt,[],[],[],[]) 
    plot_pose(tt,x,y,z) 