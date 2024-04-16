import numpy as np
import os


dataset_path = "/media/lars/SSD/Semesterarbeit/Data_IAC/Datasets_Kitti_Style/Monza_Tunnel/"

sequence = "03"
images = ["0"]

jpg_paths = [os.path.join(dataset_path,sequence,"image_"+end+"/") for end in images]
pcd_path = os.path.join(dataset_path,sequence,"velodyne_points/")

filenames = sorted(os.listdir(pcd_path))

id = 0


line_reached = False
line_id = 0
numberOfFiles = len(filenames)

listlistfilesnames = []

for path in jpg_paths:
    filesnames_images = sorted(os.listdir(path))
    listlistfilesnames.append(filesnames_images)
    print(len(filesnames_images))
    print(numberOfFiles)
    assert len(filesnames_images) == numberOfFiles

for idx, filename in enumerate(filenames):
    with open(pcd_path+filename, "r") as f: 
        lines = f.readlines()
        numberofPoints = len(lines)-10
        points_x = np.zeros(numberofPoints)
        points_y = np.zeros(numberofPoints)
        points_z = np.zeros(numberofPoints)
        intensities = np.zeros(numberofPoints)
        for line in lines:
            if "DATA ascii" in line:
                line_reached = True
                continue
            if line_reached:
                x,y,z,intensity,_,_ = line.split(" ")
                points_x[line_id] = float(x)
                points_y[line_id] = float(y)
                points_z[line_id] = float(z)
                intensities[line_id] = float(intensity)
                line_id += 1
        line_reached = False
        line_id = 0
    points_x = points_x.reshape((-1,1))
    points_y = points_y.reshape((-1,1))
    points_z = points_z.reshape((-1,1))
    intensities = intensities.reshape((-1,1))
    pointCloudDataArray = np.concatenate((points_x,points_y,points_z,intensities),axis=1)
    
    bin_filename = os.path.splitext(os.path.basename(pcd_path+filename))[0]

    with open("/media/lars/SSD/Semesterarbeit/Data_IAC/Datasets_Kitti_Style/Monza_Tunnel/"+sequence+"/velodyne_points/"+f"{idx:06}"+".bin", "w+") as f:
        pointCloudDataArray.tofile(f)

    print(f"{idx} of {numberOfFiles}")


for list_idx, image_list in enumerate(listlistfilesnames):
    for idx, image in enumerate(image_list):
        parent_path = jpg_paths[list_idx]
        os.rename(parent_path + image,parent_path + f"{idx:06}.jpg")

