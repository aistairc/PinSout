from plyfile import PlyData
import os
import math
import numpy as np
# from src.SC_demo.sem_seg import batch_inference
from src.SC_demo.sem_seg import batch_inference

# /usr/local/cuda/lib64:$LD_LIBRARY_PATH

def ply_to_collada(ply_path='/home/dprt/Desktop/21/Untitled Folder 2/room2.ply'):
    """Reading the .ply data and divide space

    Dividing the entire PointCloud data into a certain size.
    Find the minimum value of each segmented data and subtract it to move the data to the origin.

    Args:
        ply_path: The path where A is located

    Returns:
        model_path: Path with the trained model needed to do Semantic segmentation.
        out_filename: The path where the 3D model will be saved
        npy_list: List of split PointCloud information
        min_list: Minimum value of each PointCloud data
    """
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)

    default_x = 10.0
    default_y = 10.0

    filepath = os.path.join(dir, 'npy_data2')
    if os.path.exists(filepath) is not True:
        os.mkdir(filepath)
    has_color = False
    for prop in plydata.elements[0].properties:
        if prop.name == 'red' or prop.name == 'blue' or prop.name == 'green':
            has_color = True

    lens = len(plydata.elements[0].data)

    data_list = list()
    for i in range(lens):
        z = plydata.elements[0].data['z'][i]
        if 10.0 > z:
            x = plydata.elements[0].data['x'][i]
            y = plydata.elements[0].data['y'][i]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                print "there is NaN value!!"
                continue

            if has_color:
                r = plydata.elements[0].data['red'][i]
                g = plydata.elements[0].data['green'][i]
                b = plydata.elements[0].data['blue'][i]
            else:
                r = 0
                g = 0
                b = 0
            # data_list.append([x, y, z, r, g, b])
            index_num = make_area_num(x, y, default_x, default_y)
            check_area = check_area_exist(data_list, index_num)

            if check_area == -1:
                index_info = []
                index_info.append([x, y, z, r, g, b])
                index_info.insert(0, index_num)
                data_list.append(index_info)
            else:
                data_list[check_area].append([x, y, z, r, g, b])

    npy_list, min_list = make_point_label(data_list)
    # print npy_list, min_list
    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)
    print out_filename, min_list
    # data = np.load('/root/pointnet/data/Stanford_5cls/Area_6_office_1.npy')
    # out_filename = "/home/dprt/Desktop/SC_DEMO/New Folder/test/st/npy_data2/office_1"
    model_path = '/home/dprt/Downloads/log_6cls_test16/model.ckpt'
    # model_path = '/root/pointnet/sem_seg/log_5cls_2/model.ckpt'
    # model_path = '/root/pointnet/sem_seg/log_5cls/model.ckpt'
    batch_inference.evaluate(model_path, out_filename, npy_list, min_list)


def get_range(point_cloud):
    """Making the bouning box information

           Finding the Minimum and Maximum points of X, Y and Z.

       Args:
           point_cloud: PointCloud data

       Returns:
           [point_max2, point_min2]: Minimum and Maximum points of X, Y and Z
    """

    ''' 0.1 10cm'''
    point_max = np.amax(np.asarray(point_cloud), axis=0)[0:2]
    point_min = np.amin(np.asarray(point_cloud), axis=0)[0:2]

    return [point_max, point_min]

def check_in_range(bbox, point):
    """Check whether the pointer is included in the bounding box

        Check the pointer's X and Y values are included in the bounding box

    Args:
        bbox: Information for bounding box
        point: Pointer to check

    Returns:
        True: The pointer is included in the bounding box
        False: The pointer is not included in the bounding box
    """
    min_point = bbox[1]
    max_point = bbox[0]
    check_list = list()
    if min_point[0] <= point[0] and point[0] <= max_point[0]:
        check_list.append(True)
    else:
        check_list.append(False)

    if min_point[1] <= point[1] and point[1] <= max_point[1]:
        check_list.append(True)
    else:
        check_list.append(False)

    if check_list.count(True) == 2:
        return True
    else:
        return False


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
            if len(data) > 500:
                points = np.asarray(data[1:])
                min_value = np.amin(points, axis=0)[0:3]
                # print min_value
                points[:, 0:3] -= min_value
                # npy_list.append(np.concatenate(points, 1))
                # print points.size
                npy_list.append(points)
                min_list.append(min_value)
                #
                # area_data = np.asarray(data[1:], dtype=np.float32)[:, 0:3]
                # cloud_data = pcl.PointCloud()
                # cloud_data.from_array(area_data)
                # print cloud_data.size
                # out_filename3 = "/home/dprt/Desktop/SC_DEMO/New Folder/test/room2" + "_" + str(count) + ".pcd"
                # pcl.save(cloud_data, out_filename3)
                # out_filename2 = "/home/dprt/Desktop/SC_DEMO/New Folder/test/room2" + "_" + str(count) + ".npy"
                # np.save(out_filename2, points)

        return npy_list, min_list

def make_area_num(x, y, default_x, default_y):
    """Check the segmented area containing points

        Create a grid map using the default area(default_x, default_y).
        Divide the point value by the default area, and set the area that contains the point.

    Args:
        x: The x coordinate of the point.
        y: The y coordinate of the point.
        default_x: The length of the area.
        default_y: The height of the area.
    Returns:
        [index_x, index_y]: Index of the area that contains the point
    """
    index_x = x // default_x
    index_y = y // default_y

    return [index_x, index_y]

def check_area_exist(data_list, index_area):
    """Check if a divided area exists

        The first value of each data in the list has area information that contains the data.
        If an existing segmented area exists, it returns the index value of the area containing data.

    Args
        data_list: List of split point cloud data
        index_area: The area that contains points.

    Returns:
        index: The index number of the area containing the data
        -1: When there is no area that contains points
    """
    if len(data_list) is 0:
        return -1
    else:
        for data in data_list:

            check_x = data[0][0] == index_area[0]
            check_y = data[0][1] == index_area[1]

            if check_x == True and check_y == True:
                return data_list.index(data)
        return -1

def txt2ply():
    print "hi"


if __name__ == '__main__':

    ply_to_collada("/home/dprt/Documents/dprt/pointnet_data/Area_2/Area_2_All_office.ply")
