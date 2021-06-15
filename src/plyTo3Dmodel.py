#!/root/anaconda3/envs/envname/bin/python2.7

from plyfile import PlyData
import random
import os
import sys
import numpy as np
import math
from sem_seg import batch_inference
#from sem_seg import batch_inference2
#from sem_seg import batch_inference3
# from plane_ransac import GeneratePointCloud
import sys
import logging
import pcl

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

    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    if len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])
    elif len(args) == 3:
        distance_threshold = float(args[2])
    ply_path = args[1]
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)

    default_x = 3.0
    default_y = 3.0

    filepath = os.path.join(dir, '8_floor_model_1127_1216')
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
        x = plydata.elements[0].data['x'][i]
        y = plydata.elements[0].data['y'][i]
        if has_color:
        	r = plydata.elements[0].data['red'][i]
        	g = plydata.elements[0].data['green'][i]
        	b = plydata.elements[0].data['blue'][i]
        else:
        	r = 0
        	g = 0
        	b = 0
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            print ("there is NaN value!!")
            continue

        index_num = make_area_num(x, y, default_x, default_y)
        check_area = check_area_exist(data_list, index_num)

        if check_area == -1:
            index_info = []
            # index_info.append([x, y, z, 0, 0, 0, 2])
            index_info.append([x, y, z, r, g, b, 2])
            index_info.insert(0, index_num)
            data_list.append(index_info)
        else:
            data_list[check_area].append([x, y, z, r, g, b, 2])

    npy_list, min_list = make_point_label(data_list)
    # print npy_list, min_list
    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)
    print (out_filename, min_list)

    model_path = os.getcwd()+'/sem_seg/model/log_5cls/model.ckpt'
    print(model_path)

    batch_inference.evaluate(model_path, out_filename, npy_list, min_list)


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
    epsilon_value = 0.5


    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    if len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])
    elif len(args) == 3:
        distance_threshold = float(args[2])
    ply_path = args[1]
    # original_point_cloud = pcl.load(ply_path)
    # visual_viewer([original_point_cloud])
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)
    filepath = os.path.join(dir, '8_floor_model_1127_1216_no')
    if os.path.exists(filepath) is not True:
        os.mkdir(filepath)
    has_color = False
    for prop in plydata.elements[0].properties:
         if prop.name == 'red' or prop.name == 'blue' or prop.name == 'green':
             has_color = True

    lens = len(plydata.elements[0].data)

    data_list = list()

    for i in range(lens):
        x = plydata.elements[0].data['x'][i]
        y = plydata.elements[0].data['y'][i]
        z = plydata.elements[0].data['z'][i]
        if has_color:
        	r = plydata.elements[0].data['red'][i]
        	g = plydata.elements[0].data['green'][i]
        	b = plydata.elements[0].data['blue'][i]
        else:
        	r = 0
        	g = 0
        	b = 0

        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            print ("there is NaN value!!")
            continue

        # if math.isnan(r) or math.isnan(g) or math.isnan(b):
        #     print ("there is rgb NaN value!!")
        #

        data_list.append([x, y, z, r, g, b, 0])
        # data_list.append([x, y, z, 0, 0, 0, 5])
    print "finish"


    npy_list, min_list = make_point_label([data_list])

    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)


    model_path = os.getcwd()+'/sem_seg/model/log_5cls/model.ckpt'
    # model_path = os.getcwd()+'/sem_seg/model/log_6cls/model.ckpt'
    #model_path = os.getcwd()+'/sem_seg/model/log6_1127/model.ckpt'
    seg_result = batch_inference.evaluate(model_path, out_filename, npy_list, min_list)
    # gp = GeneratePointCloud(distance_threshold, epsilon_value)
    # gp.make_wall_info(seg_result)

def ply_to_CityGML2(args):
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


    distance_threshold = 0.05
    epsilon_value = 0.5


    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    if len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])
    elif len(args) == 3:
        distance_threshold = float(args[2])
    ply_path = args[1]
    # original_point_cloud = pcl.load(ply_path)
    # visual_viewer([original_point_cloud])
    # plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)
    filepath = os.path.join(dir, 'npy_data2')
    if os.path.exists(filepath) is not True:
        os.mkdir(filepath)
    has_color = False
    # for prop in plydata.elements[0].properties:
    #     if prop.name == 'red' or prop.name == 'blue' or prop.name == 'green':
    #         has_color = True

    npy_list = np.load(ply_path)
    print npy_list[:, 0:6]
    # print npy_list, min_list
    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)


    model_path = os.getcwd()+'/sem_seg/model/log_5cls/model.ckpt'

    seg_result = batch_inference2.evaluate(model_path, out_filename, npy_list)
def ply_to_CityGML3(args):
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


    distance_threshold = 0.05
    epsilon_value = 0.5


    if len(args) == 1:
        logger.info("There is no path for reading the PointCloud")
        return
    if len(args) == 4:
        distance_threshold = float(args[2])
        epsilon_value = float(args[3])
    elif len(args) == 3:
        distance_threshold = float(args[2])
    ply_path = args[1]
    # original_point_cloud = pcl.load(ply_path)
    # visual_viewer([original_point_cloud])
    plydata = PlyData.read(ply_path)
    dir, ply_file = os.path.split(ply_path)
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
        x = plydata.elements[0].data['x'][i]
        y = plydata.elements[0].data['y'][i]
        z = plydata.elements[0].data['z'][i]

        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            print ("there is NaN value!!")
            continue

        data_list.append([x, y, z,0,0,0,0])
    print "finish"


    npy_list, min_list = make_point_label([data_list])
    # print npy_list, min_list
    npy_file = ply_file.split('.')[0]
    out_filename = os.path.join(filepath, npy_file)
    # print (out_filename, min_list)
    # data = np.load('/root/pointnet/data/Stanford_5cls/Area_6_office_1.npy')
    # out_filename = "/home/dprt/Desktop/SC_DEMO/New Folder/test/st/npy_data2/office_1"
    # model_path = '/home/dprt/Downloads/log_6cls_test16/model.ckpt'

    model_path = os.getcwd()+'/sem_seg/model/log_5cls/model.ckpt'
    print len(min_list)
    # model_path = os.getcwd()+'/sem_seg/model/log6class/model.ckpt'
    # model_path = os.getcwd()+'/sem_seg/model/log6/model.ckpt'
    # print(model_path)
    # model_path = '/root/pointnet/sem_seg/log_5cls_2/model.ckpt'
    # model_path = '/root/pointnet/sem_seg/log_5cls/model.ckpt'
    seg_result = batch_inference3.evaluate(model_path, out_filename, npy_list, min_list)

    # gp = GeneratePointCloud(distance_threshold, epsilon_value)
    # gp.make_wall_info(seg_result)
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
        path = 'temp'
        try: 
    		os.mkdir(path) 
	except OSError as error: 
    		pass
        for data in data_list:
            count = count + 1
            if len(data) > 200:
                points = np.asarray(data[1:])
                min_value = np.amin(points, axis=0)[0:3]
                # print min_value
                points[:, 0:3] -= min_value
                # npy_list.append(np.concatenate(points, 1))
                # print points.size
                npy_list.append(points)
                min_list.append(min_value)

                area_data = np.asarray(data[1:], dtype=np.float32)[:, 0:3]
                cloud_data = pcl.PointCloud()
                cloud_data.from_array(area_data)
                print cloud_data.size

                out_filename3 =  "./temp/"+str(count) + ".pcd"
                pcl.save(cloud_data, out_filename3)
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

def visual_viewer(cloud_list):
    """Visualizing Pointcloud data

    Args:
        cloud_list: list of PointClouds

    Returns:
        Visualizing Pointclouds data
    """
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)


    r_list = []
    b_list = []
    g_list = []
    r_num = random.randint(0, 256)
    b_num = random.randint(0, 256)
    g_num = random.randint(0, 256)

    for i in range(len(cloud_list)):
        while r_num in r_list:
            r_num = random.randint(0, 256)
        r_list.append(r_num)
        while b_num in b_list:
            b_num = random.randint(0, 256)
        b_list.append(b_num)
        while g_num in g_list:
            g_num = random.randint(0, 256)
        g_list.append(g_num)

    for index in range(len(cloud_list)):
        r = r_list[index]
        b = b_list[index]
        g = g_list[index]

        point_size = 3
        color_handle = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_list[index], r, b, g)
        id = b'inliers_cloud_' + str(index)
        viewer.AddPointCloud_ColorHandler(cloud_list[index], color_handle, id)
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, point_size, id)

    viewer.InitCameraParameters()
    while viewer.WasStopped() != True:
        viewer.SpinOnce(100)

def test(ply_path):
    plydata = PlyData.read(ply_path)
    lens = len(plydata.elements[0].data)
    point_x = plydata.elements[0].data['x']
    point_y = plydata.elements[0].data['Y']
    point_z = plydata.elements[0].data['Z']
    point_r = np.zeros(lens)
    point_g = np.zeros(lens)
    point_b = np.zeros(lens)

if __name__ == '__main__':
    args = sys.argv
    # args = [0, "/home/dprt/Documents/DataSet/ISPRS Benchmark Dataset/CaseStudy1/PointCloud_cs1_dec_x-y-z-localtime6precision - Cloud.ply"]
    # args = [0, "/home/dprt/Documents/office_1 - Cloud.ply"]
    # args = [0, "/home/dprt/Documents/DataSet/ISPRS Benchmark Dataset/CaseStudy1/1/original4.ply"]
    # args = [0, "/home/dprt/Documents/DataSet/ISPRS Benchmark Dataset/report_data/CaseStudy3-1.ply"]
    # args = [0, "/home/dprt/Documents/DataSet/original data/original_data_color.ply"]
    #args = [0, "/home/dprt/Documents/DataSet/AIST_8_floor/8_floor_grid_pointsize/colorized.ply"]
    # args = [0, "/home/dprt/Documents/DataSet/ISPRS Benchmark Dataset/CaseStudy1/1/original_data.ply"]
    args = [0, "../data/sample_data/original_data.ply"]
    #ply_to_CityGML(args)
    ply_to_CityGML_Grid(args)



