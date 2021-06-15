#!/root/anaconda3/envs/envname/bin/python2.7

from plyfile import PlyData
import random
import os
import sys
import numpy as np
import math
from sem_seg import batch_inference
from sem_seg import Predict
# from sem_seg import batch_inference2
# from sem_seg import batch_inference3
import PointCloudUtils

from struct import pack, unpack
import pcl
import sys
import logging
import GeneratePointCloud_New as gpc

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
def ply_to_CityGML_Grid(args):
    """Reading the .ply data and divide space

    Dividing the entire PointCloud data into a certain size.
    Find the minimum value of each segmented data and subtract it to move the data to the origin.

    Args:
        args: Parameter list that user inputs
        args[1] : File path of PointCloud data
        args[2] : Distance threshold
        args[3] : Epsilon Value
    Returns:
        model_path: Path with the trained model needed to do Semantic segmentation.
        out_filename: The path where the 3D model will be saved
        npy_list: List of split PointCloud information
        min_list: Minimum value of each PointCloud data
    """

    distance_threshold = 0.05
    epsilon_value = 0.5

    logger.info("Starting Process")


    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    elif len(args) == 3:
        distance_threshold = float(args[2])
    elif len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])

    ply_path = args[1]

    # Setting the save path for result of PointNet
    folder_name = (ply_path.split("/")[-1]).split(".")[0]
    folder_name = folder_name + "_result"
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)
    filepath = os.path.join(dir, folder_name)
    if os.path.exists(filepath) is not True:
        os.mkdir(filepath)

    # Checking the rgb value is existing

    has_color = False
    for prop in plydata.elements[0].properties:
        if prop.name == 'red' or prop.name == 'blue' or prop.name == 'green':
            has_color = True

    x = plydata.elements[0].data['x']
    y = plydata.elements[0].data['y']
    z = plydata.elements[0].data['z']
    point_value = np.stack([x, y, z], axis=1)
    if has_color:
        red = plydata.elements[0].data['red']
        green = plydata.elements[0].data['green']
        blue = plydata.elements[0].data['blue']
    else:
        red = np.full(len(point_value), 255)
        green = np.full(len(point_value), 255)
        blue = np.full(len(point_value), 255)

    color_value = np.stack([red, green, blue], axis=1)


    voxelize = False
    if voxelize:
        # Running the voxelization using voxel size
        xyzrgb_value = PointCloudUtils.grid_subsampling(point_value, color_value, 0.02)
    else:
        xyzrgb_value = np.concatenate((point_value, color_value), axis=1)

    xyzrgb_grid_value = PointCloudUtils.grid_subsampling2(xyzrgb_value, 10)
    count = 0
    xyzrgb_grid_value_list = list()
    for key, value in xyzrgb_grid_value.items():
        count +=1
        xyzrgb_grid_value_list.append(value)
    npy_list, min_list = make_point_label(xyzrgb_grid_value_list)

    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)


    model_path = os.getcwd()+'/sem_seg/model/log_6cls/model.ckpt'

    wall_cloud, door_cloud, window_cloud = batch_inference.evaluate(model_path, out_filename, npy_list, min_list)
    generate_pointcloud = gpc.GeneratePointCloud(distance_threshold, epsilon_value, wall_cloud, [], [])
    generate_pointcloud.make_room_info()


def ply_to_CityGML(args):
    """Reading the .ply data starting the PinSout

    Dividing the entire PointCloud data into a certain size.
    Find the minimum value of each segmented data and subtract it to move the data to the origin.

    Args:
        args: Parameter list that user inputs
        args[1] : File path of PointCloud data
        args[2] : Distance threshold
        args[3] : Epsilon Value
    Returns:
        model_path: Path with the trained model needed to do Semantic segmentation.
        out_filename: The path where the 3D model will be saved
        npy_list: List of split PointCloud information
        min_list: Minimum value of each PointCloud data
    """


    distance_threshold = 0.05
    epsilon_value = 0.1

    logger.info("Starting Process")
    print args

    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    elif len(args) == 3:
        distance_threshold = float(args[2])
    elif len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])

    ply_path = args[1]
    # Setting the save path for result of PointNet
    folder_name = (ply_path.split("/")[-1]).split(".")[0]
    folder_name = folder_name + "_result"
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)
    filepath = os.path.join(dir, folder_name)

    PointCloudUtils.ROOT_PATH = filepath

    if os.path.exists(filepath) is not True:
        os.mkdir(filepath)

    # Checking the rgb value is existing

    has_color = False
    for prop in plydata.elements[0].properties:
        if prop.name == 'red' or prop.name == 'blue' or prop.name == 'green':
            has_color = True
    # Store the x, y, z points
    x = plydata.elements[0].data['x']
    y = plydata.elements[0].data['y']
    z = plydata.elements[0].data['z']
    point_value = np.stack([x, y, z], axis=1)
    if has_color:
        # Store the r, g, b value
        red = plydata.elements[0].data['red']
        green = plydata.elements[0].data['green']
        blue = plydata.elements[0].data['blue']
    else:
        # Store the r, g, b default value
        red = np.full(len(point_value), 0)
        green = np.full(len(point_value), 0)
        blue = np.full(len(point_value), 0)

    color_value = np.stack([red, green, blue], axis=1)
    voxelize = True
    if voxelize:
        # Running the voxelization using voxel size
        xyzrgb_value = PointCloudUtils.grid_subsampling(point_value, color_value, 0.02)

    else:
        xyzrgb_value = np.concatenate((point_value, color_value), axis=1)

    # Move the pointcloud data to (0,0,0) and add the label info


    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)


    # Setting trained model path
    model_path = os.getcwd()+'/sem_seg/model/log_6cls/model.ckpt'

    # wall_cloud, door_cloud, window_cloud = Predict.evaluate(model_path, out_filename, xyzrgb_value)
    # Starting PointNet semantic segmentation

    npy_list, min_list = make_point_label([xyzrgb_value])
    wall_cloud, door_cloud, window_cloud = batch_inference.evaluate(model_path, out_filename, npy_list, min_list)

    # Starting the generating solution to create Graph
    GeneratePCData = gpc.GeneratePointCloud(distance_threshold, epsilon_value, wall_cloud, door_cloud, window_cloud)
    GeneratePCData.make_room_info()



def make_point_label(data_list):
    """Find and subtract the minimum value of the segmented data

        If the number of points of each data in the list is 500 or more, the minimum value of the data is found and subtracted.

    Args:
        data_list: List of split point cloud data

    Returns:
        npy_list: The list after subtracting the minimum value from each PointCloud
        min_list: List of minimum values for each PointCloud
    """
    if len(data_list) != 0:
        npy_list = []
        min_list = []
        count = 0
        for data in data_list:
            count = count + 1

            if len(data) > 4096:
                points = np.asarray(data)
                min_value = np.amin(points, axis=0)[0:3]
                # Move original data to (0, 0, 0)
                points[:, 0:3] -= min_value
                # Making the default label info
                labels = np.zeros((points.shape[0], 1))
                # Adding the default label info to pointcloud data list
                points_with_label = np.concatenate((points, labels), axis=1)


                npy_list.append(points_with_label)
                min_list.append(min_value)
        return npy_list, min_list


if __name__ == '__main__':

    args = ["test", "../data/sample_data/original_data.ply", 0.052, 0.1]
    ply_to_CityGML(args)
