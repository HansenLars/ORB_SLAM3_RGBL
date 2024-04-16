from multiprocessing.resource_sharer import stop
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import copy
import imageio as im
from scipy.spatial.transform.rotation import Rotation 
import numpy.linalg as linalg


sequence = "00"
dataset_path = "/home/lars/Data_IAC/Datasets_Kitti_Style/Monza_Tunnel"

filelist = [os.path.splitext(filename)[0] for filename in os.listdir(os.path.join(dataset_path,sequence, "image_0"))] 
filelist.sort()

path_pcd = "/home/lars/Data_IAC/Datasets_Kitti_Style/BackupForTesting/00/luminar_front/"

id = 0

for name in filelist:
    
    luminar_path = os.path.join(dataset_path, sequence,  "luminar_front")
    binary = os.path.join(luminar_path, name + ".bin")
    scan = np.fromfile(binary, dtype=np.float32)#.reshape((-1, 4))
    print(scan.shape)
    points = scan[:, 0:3]

    fig = plt.figure(1, figsize=(12,5), dpi=96, tight_layout=True)
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    plt.show()
    
    file_list = os.listdir(path_pcd)

    line_reached = False

    with open(path_pcd+file_list[id], 'r') as f:
        fig1 = plt.figure(1, figsize=(12,5), dpi=96, tight_layout=True)
        ax1 = fig1.add_subplot(121, projection='3d')
        points_x = np.array([])
        points_y = np.array([])
        points_z = np.array([])
        
        for line in f.readlines():
            if "DATA ascii" in line:
                line_reached = True
                continue
            elif line == "":
                line_reached = False
            if line_reached:
                x,y,z,_,_,_ = line.split(" ")
                points_x = np.append(points_x, [float(x)])
                points_y = np.append(points_y, [float(y)])
                points_z = np.append(points_z, [float(z)])
            
        ax1.scatter(points_x,points_y,points_z)
        plt.show()
        id += 1