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
import math

sequence = "00"

show_overlay = True

cur_dir = os.path.abspath(os.path.dirname(__file__))

dataset_path = "/home/lars/Data_IAC/Datasets_Kitti_Style/Monza_Tunnel"

image_path = os.path.join(dataset_path, sequence, "image_0")
luminar_path = os.path.join(dataset_path, sequence,  "luminar_front")
calib_path = os.path.join(dataset_path, sequence)
save_path = os.path.join(dataset_path, sequence, "depth_png")

filelist = [os.path.splitext(filename)[0] for filename in os.listdir(os.path.join(dataset_path,sequence, "image_0"))]
filelist.sort()
print("Generating " + str(len(filelist)) + " overlay images!")

image_counter = 0
num_images = len(filelist)

fig = plt.figure(1, figsize=(12,5), dpi=96, tight_layout=True)
ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
ax_3d.view_init(elev=0, azim=-180)
ax_2d = fig.add_subplot(1, 2, 2)

for name in filelist:
    img = os.path.join(image_path, name + ".jpg")
    binary = os.path.join(luminar_path, name + ".bin")
    with open(os.path.join(calib_path, "calib.txt"),'r') as f:
        calib = f.readlines()

    K = np.matrix([1724.778859, 0.000000,       489.853476,
                   0.000000,    1723.969299,    250.216425,
                   0.000000,    0.000000,       1.000000    ]).reshape(3,3)
        
    Rotation_xz = Rotation.from_euler("XYZ",[np.pi/2-math.radians(5),0,np.pi/2-math.radians(4)])

    T_cam_camglob = np.concatenate((np.concatenate((Rotation.as_matrix(Rotation_xz),np.matrix([0,0,0]).T),axis=1),np.matrix([0,0,0,1])),axis=0)
   
    T_world_lidar = np.matrix([1,   0,  0,  2.242,
                               0,   1,  0,  0,
                               0,   0,  1,  0.448,
                               0,   0,  0,  1      ]).reshape(4,4)

    R = Rotation.from_euler("XYZ",[0,0,0])
    R_camglob_world = linalg.inv(Rotation.as_matrix(R))

    translation = np.matrix([2.184,0.171,0.422]).T
    translation_camglob_world = -np.matmul(R_camglob_world, translation)
    
    T_camglob_world = np.concatenate((np.concatenate((R_camglob_world,translation_camglob_world),axis=1),np.matrix([0,0,0,1])),axis=0)

    T_camglob_lidar = np.matmul(T_camglob_world, T_world_lidar)
    T_cam_lidar = np.delete(np.matmul(T_cam_camglob,T_camglob_lidar),3,0)


    # Odometry: No rect needed
    # R0_rect = np.matrix([float(x) for x in calib[4].strip('\n').split(' ')[1:]]).reshape(3,3)
    # Add a 1 in bottom-right, reshape to 4 x 4
    # R0_rect = np.insert(R0_rect,3,values=[0,0,0],axis=0)
    # R0_rect = np.insert(R0_rect,3,values=[0,0,0,1],axis=1)

    # Odometry: index 4 needed instead of 5

    # read raw data from binary
    scan = np.fromfile(binary, dtype=np.float64).reshape((-1, 4))
    points = scan[:, 0:3]
    # lidar xyz (front, left, up)
    # TODO: use fov filter? 
    velo = np.insert(points, 3, 1, axis=1).T
    velo = np.delete(velo, np.where(velo[0,:]<0), axis=1)
    # Odometry: no rect needed
    # cam = P2 * R0_rect * Tr_velo_to_cam * velo
    asdfmovie = np.matmul(T_cam_lidar, velo)
    Xlidar_cam = np.matmul(K, asdfmovie)
    
    # Xlidar_cam = np.delete(Xlidar_cam, np.where(Xlidar_cam[2,:]<0)[1], axis=1)
    # get u,v,z
    points_projected_lidar_cam = Xlidar_cam
    points_projected_lidar_cam[:2] = Xlidar_cam[:2] / Xlidar_cam[2,:]
    # do projection staff
    
    png = mpimg.imread(img)
    IMG_H,IMG_W,_ = png.shape

    # restrict canvas in range
    # filter point out of canvas
    
    u,v,z = points_projected_lidar_cam
    u_out = np.logical_or(u<0, u>IMG_W)
    v_out = np.logical_or(v<0, v>IMG_H)
    outlier = np.logical_or(u_out, v_out)   
    points_projected_lidar_cam = np.delete(points_projected_lidar_cam, np.where(outlier),axis=1)
    scan_skimmed = np.delete(scan.T, np.where(outlier),axis=1)
    # generate color map from depth
    u,v,z = points_projected_lidar_cam

    if show_overlay:

        ax_2d.axis([0,IMG_W,IMG_H,0])
        ax_2d.imshow(png)
        # u = np.subtract(IMG_W,u)[0]
        # v = np.subtract(IMG_H,v)[0]
        ax_2d.scatter([u], [v], c=[z], cmap='rainbow', alpha=0.5, s=2, marker="x")
        c = np.sum(np.abs(scan)**2,axis=1)**(1./2)
        # ax_3d.scatter(scan[:,0],scan[:,1],scan[:,2],c=c, cmap='rainbow')
        ax_3d.scatter(scan_skimmed[0,:],scan_skimmed[1,:],scan_skimmed[2,:],c=[z], cmap='rainbow')
        # plt.title(name)
        # plt.axis("off")
        figmgr = plt.get_current_fig_manager()  
        bkend = plt.get_backend()
        figmgr.resize(*figmgr.window.maxsize())
        plt.show()
        # plt.pause(0.0001)
        # plt.clf()
    
    image_data = np.array(points_projected_lidar_cam)
    image_matrix = np.ones((IMG_H, IMG_W), dtype=np.uint16) * 0

    index=0
    for i in image_data[1,:]:
        image_matrix[int(i), int(image_data[0, index])] = image_data[2, index] * (256.)
        # print(image_matrix[int(i), int(image_data[0, index])])
        index = index+1

    im.imwrite(os.path.join(save_path, name + ".png"), im=(image_matrix))

    print("Finished ", image_counter, " out of ", num_images) 
    image_counter += 1
