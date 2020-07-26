import numpy as np
import pcl
import random
import pcl.pcl_visualization
import math
from sympy import Symbol, solve, Eq
import matplotlib.pyplot as plt
import ring_sort as rs
import ply2obj as po
import time
import Queue
# cloud = pcl.load("/home/dprt/Desktop/stanford_data/sc_test_data/test.pcd")
# cloud = pcl.load("/home/dprt/Desktop/Demo/test/sc_test_data_Booth_3_wall.ply")
# cloud = pcl.load("/home/dprt/Desktop/sc_test_data_Booth_3_ceiling - Cloud.pcd")
# cloud = pcl.load("/home/dprt/Downloads/table_scene_lms400 (1).pcd")
cloud_value = pcl.PointCloud

def clustering(cloud, vg_size=0.1, tolerance=0.5, min_cluster_size=1000):
    cluster_list = list()
    global cloud_value
    cloud_value = cloud

    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(vg_size, vg_size, vg_size)
    cloud_filtered = vg.filter()


    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(cloud_value.size)
    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()

    for j, indices in enumerate(cluster_indices):

        cloud_cluster = pcl.PointCloud()

        points = np.zeros((len(indices), 3), dtype=np.float32)


        for i, indice in enumerate(indices):

            points[i][0] = cloud_filtered[indice][0]
            points[i][1] = cloud_filtered[indice][1]
            points[i][2] = cloud_filtered[indice][2]

        cloud_cluster.from_array(points)
        cluster_list.append(cloud_cluster)
    if len(cluster_list) != 0:

        return cluster_list
    else:
        cluster_list.append(cloud)
        return cluster_list

def visual_viewer(cloud_list):
    global cloud_value
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)

    viewer.AddPointCloud(cloud_value, b'sample cloud')
    viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')
    # viewer.AddPointCloud(ceiling_cloud, b'sample cloud1')
    # viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud1')
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
def visual_viewer2(cloud, cloud_list):

    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)

    viewer.AddPointCloud(cloud, b'sample cloud')
    viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')
    # viewer.AddPointCloud(ceiling_cloud, b'sample cloud1')
    # viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud1')
    # r_list = [255, 0,   255, 0,   255, 0]
    # b_list = [0  , 255, 0,   0,   255, 255]
    # g_list = [0  , 255, 255, 255, 0,   0]
    r_list = []
    b_list = []
    g_list = []
    r_num = random.randint(0, 256)
    b_num = random.randint(0, 256)
    g_num = random.randint(0, 256)
    print len(cloud_list)
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

        point_size = 10
        color_handle = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_list[index], r, b, g)
        id = b'inliers_cloud_' + str(index)
        viewer.AddPointCloud_ColorHandler(cloud_list[index], color_handle, id)
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, point_size, id)

    viewer.InitCameraParameters()
    while viewer.WasStopped() != True:
        viewer.SpinOnce(100)

def do_passthrough_filter( point_cloud, name_axis='z'):
    pass_filter = point_cloud.make_passthrough_filter()
    pass_filter.set_filter_field_name(name_axis)

    return pass_filter.filter()

def test(cloud):
    segmenter = cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)

    segmenter.set_normal_distance_weight(0.05)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(0.05)

    inlier_indices, coefficients = segmenter.segment()

    inliers = cloud.extract(inlier_indices, negative=False)

    outliers = cloud.extract(inlier_indices, negative=True)

    return inliers, outliers, coefficients, inlier_indices



def check_distance_plane(point_cloud, coeff):

    a = coeff[0]
    b = coeff[1]
    c = coeff[2]
    d = coeff[3]
    t = point_cloud.to_list()
    count = 0
    for index in t:
        x = index[0]
        y = index[1]
        z = index[2]

        point_distance = float(
            math.fabs(a * x + b * y + c * z + d) / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))

        if point_distance <= 0.1:
            count += 1

    if count == 0:
        distance_rate = 0.0
    else:
        distance_rate = round(float(count) / float(len(t)), 3)
    return distance_rate

def get_plane_list(cloud_list):
    plane_list = list()
    normal_vector = list()

    each_cloud = cloud_list

    min_size = 100
    max_size = each_cloud.size
    count = 0
    print "get_plane_list : "+str(max_size)
    while True:

        filtered_cloud = do_passthrough_filter(point_cloud=each_cloud, name_axis='z')
        print "filtered_cloud : "+ str(filtered_cloud.size)
        if count == 5 or filtered_cloud.size <= min_size:
            break

        inliers, outliers, coefficients, inlier_indices = test(filtered_cloud)

        default_vector = [0, 0, -1]
        model_value = [coefficients[0], coefficients[1], coefficients[2]]
        model_cos = np.dot(default_vector, model_value) / (
                np.linalg.norm(model_value) * np.linalg.norm(default_vector))

        if math.fabs(model_cos) >= 0.8:
            each_cloud = outliers
        else:

            vg = inliers.make_voxel_grid_filter()
            vg.set_leaf_size(0.01, 0.01, 0.01)
            cloud_filtered = vg.filter()



            tree = cloud_filtered.make_kdtree()
            ec = cloud_filtered.make_EuclideanClusterExtraction()
            ec.set_ClusterTolerance(0.5)
            ec.set_MinClusterSize(min_size)
            ec.set_MaxClusterSize(max_size)
            ec.set_SearchMethod(tree)
            cluster_indices = ec.Extract()


            if len(cluster_indices) == 0:
                count += 1
            else:
                count = 0

            temp = []

            for j, indices in enumerate(cluster_indices):

                cloud_cluster = pcl.PointCloud()

                temp += indices


                points = np.zeros((len(indices), 3), dtype=np.float32)

                for i, indice in enumerate(indices):

                    points[i][0] = cloud_filtered[indice][0]
                    points[i][1] = cloud_filtered[indice][1]
                    points[i][2] = cloud_filtered[indice][2]

                cloud_cluster.from_array(points)

                distance_p = check_distance_plane(cloud_cluster, coefficients)

                #
                if distance_p >= 0.75:
                    normal_vector.append(coefficients)
                    plane_list.append(cloud_cluster)
                elif cloud_cluster.size >= 500:
                    normal_vector.append(coefficients)
                    plane_list.append(cloud_cluster)
                else:
                    normal_vector.append(coefficients)
                    plane_list.append(cloud_cluster)

            each_cloud = outliers
    return plane_list, normal_vector


def make_wall_info(cloud):
    print "Make Wall Info"

    cluster_list = clustering(cloud)
    surface_point_list = list()

    # if len(cluster_list) == 1:
    #     cluster_list.insert(0, cloud)
    #     cluster_list.pop(len(cluster_list) - 1)
    ring_sort = rs.Point_sort()
    print "count of clustered data : ", len(cluster_list)
    # visual_viewer(cluster_list)
    room_list = list()
    all_area = []
    for clustered_cloud in cluster_list:
        each_area = []
        wall_point_list, wall_vector_list = get_plane_list(clustered_cloud)

        # if len(wall_vector_list) >= 3:
        #     print len(wall_vector_list)
            # room_list.append(clustered_cloud)

        new_plane_info = list()
        new_point_info = list()
        new_line_info = list()
        bbox_info = list()
        for wall_index in range(len(wall_point_list)):
            point_info, plane_info, line_info, each_bbox_info = make_side_line(wall_point_list[wall_index],
                                                               wall_vector_list[wall_index])
            pcl.save(wall_point_list[wall_index], '/home/dprt/Desktop/SC_DEMO/New Folder/test/each/each_'+str(cluster_list.index(clustered_cloud))+'_'+str(wall_index)+".pcd")
            if len(point_info) != 0:
                new_point_info.append(point_info)
            if len(plane_info) != 0:
                new_plane_info.append(plane_info)
            if len(line_info) != 0:
                new_line_info.append(line_info)
            if len(each_bbox_info) != 0:
                bbox_info.append(each_bbox_info)

        wall_surface_list = get_intersection_line(new_point_info, new_plane_info, new_line_info, bbox_info)
        # new_wall_surface_list = []
        # for s_i in wall_surface_list:
        #     new_wall_surface_list.append(s_i)
        print "Make surface of the clustered data  : ", str(cluster_list.index(clustered_cloud)+1)
        # surface_point_list.append(wall_surface_list)
        print
        print
        print wall_surface_list
        print
        print
        # surface_point_list.append(wall_surface_list)

        for wall_info in wall_surface_list:

            if len(wall_info) != 0:

                if len(wall_info) == 2:
                    line_1 = wall_info[0]
                    line_2 = wall_info[1]
                    rectangle_points = line_1 + line_2
                    rectangle_points = np.asarray(rectangle_points)

                    sorted_surface_info = ring_sort.SortPointsClockwise(rectangle_points.tolist(), False)
                    # sorted_surface_info_2 = ring_sort.SortPointsClockwise(rectangle_points.tolist(), True)

                    surface_point_list.append(sorted_surface_info)
                    # surface_point_list.append(sorted_surface_info_2)

                if len(wall_info) > 2:
                    rectangle_points = get_rectangle_points(wall_info)
                    rectangle_points = np.asarray(rectangle_points)

                    sorted_surface_info = ring_sort.SortPointsClockwise(rectangle_points.tolist(), False)
                    # sorted_surface_info_2 = ring_sort.SortPointsClockwise(rectangle_points.tolist(), True)

                    surface_point_list.append(sorted_surface_info)
                    # surface_point_list.append(sorted_surface_info_2)



    print all_area
    return surface_point_list

def get_center_point(side_points):
    center_point = (reduce(lambda x, y: np.array(x) + np.array(y), side_points)) / float(len(side_points))
    min_z = side_points[0][2]

    plane_abc = np.cross(np.asarray(side_points[1]) - np.asarray(side_points[0]),
                             np.asarray(side_points[2]) - np.asarray(side_points[0]))
    plane_d = np.asarray([np.sum(plane_abc * side_points[0]) * -1.0])
    a, b, c, d = np.concatenate([plane_abc, plane_d]).tolist()
    min_y = (a * center_point[0] + c * min_z + d) / b * (-1.0)
    center_xyz = np.asarray([center_point[0], min_y, min_z])

    return center_xyz

def get_rectangle_points(wall_info):
    side_1 = wall_info[0]
    side_2 = wall_info[len(wall_info) - 1]

    if len(wall_info) == 3:
        d_value_1 = np.asarray(side_1[0]) - np.asarray(wall_info[1][0])
        # d_1 = math.fabs(d_value_1[0]) + math.fabs(d_value_1[1])
        d_1 = math.sqrt(math.pow(d_value_1[0], 2) + math.pow(d_value_1[1], 2))

        d_value_2 = np.asarray(side_2[0]) - np.asarray(wall_info[1][0])
        # d_2 = math.fabs(d_value_2[0]) + math.fabs(d_value_2[1])
        d_2 = math.sqrt(math.pow(d_value_2[0], 2) + math.pow(d_value_2[1], 2))
        # print d_1, d_2
        if d_1 > d_2:
            return side_1 + wall_info[1]
        elif d_1 < d_2:
            return side_2 + wall_info[1]
        else:
            return side_1 + side_2
    else:
        distance_list = list()
        index_list = list()
        for value in range(1, len(wall_info) - 1):
            d_value = np.asarray(side_1[0]) - np.asarray(wall_info[value][0])
            # d = math.fabs(d_value[0]) + math.fabs(d_value[1])
            d = math.sqrt(math.pow(d_value[0], 2) + math.pow(d_value[1], 2))
            if d_value[2] != 0:
                print "The Z value is not same!!!! %s" % str(d_value[2])
            distance_list.append(d)
            index_list.append(value)

        center_point = get_center_point(side_1 + side_2)
        center_point -= side_1[0]
        # to_center_d1 = math.fabs(center_point[0]) + math.fabs(center_point[1])
        to_center_d1 = math.sqrt(math.pow(center_point[0], 2) + math.pow(center_point[1], 2))

        max_d = np.amax(np.asarray(distance_list))
        min_d = np.amin(np.asarray(distance_list))

        min_i = np.argmin(np.asarray(distance_list))
        max_i = np.argmax(np.asarray(distance_list))

        index_2 = index_list[int(max_i)]
        index_1 = index_list[int(min_i)]

        if math.fabs(min_d - max_d) <= 0.5:
            if index_2 != index_1:
                if max_d < to_center_d1:
                    return wall_info[index_1] + side_2
                elif min_d > to_center_d1:
                    return wall_info[index_2] + side_1
                else:
                    return wall_info[index_1] + wall_info[index_2]
            else:
                return side_1 + side_2
        else:
            return wall_info[index_1] + wall_info[index_2]
        if max_d < to_center_d1:
            return wall_info[index_1] + side_2
        elif min_d > to_center_d1:
            return wall_info[index_2] + side_1
        else:
            if index_1 == index_2:
                return side_1 + side_2
            else:
                return wall_info[index_1] + wall_info[index_2]


def get_point_from_line(line_info):

    point_set = list()
    max_z = line_info[6]
    min_z = line_info[7]

    temp_surface_info = list()
    max_z1 = max_z
    max_t = (max_z1 - line_info[5]) / line_info[2]
    max_x1 = max_t * line_info[0] + line_info[3]
    max_y1 = max_t * line_info[1] + line_info[4]

    temp_surface_info.append(float(max_x1))
    temp_surface_info.append(float(max_y1))
    temp_surface_info.append(float(max_z1))
    point_set.append(temp_surface_info)
    temp_surface_info = list()
    min_z1 = min_z
    min_t = (min_z1 - line_info[5]) / line_info[2]
    min_x1 = min_t * line_info[0] + line_info[3]
    min_y1 = min_t * line_info[1] + line_info[4]

    temp_surface_info.append(float(min_x1))
    temp_surface_info.append(float(min_y1))
    temp_surface_info.append(float(min_z1))
    point_set.append(temp_surface_info)

    return point_set


    ''' END Make Intersection Line'''
def make_chair_info(cloud, save_path):
    '''
    remove the z vlaue of chair data, if z value higher than 2.0
    :param cloud:
    :param xyz_min:
    :param save_path:
    :return:
    '''
    chair_range = get_range(cloud.to_list())
    cluster_list = clustering(cloud, min_cluter_size=100)
    main_area = get_area(chair_range)
    i = 0

    clustered_bbox = list()
    if len(cluster_list) == 0:
        temp_cloud = cloud.to_array()
        min_z = np.amin(temp_cloud, axis=0)[2:3]
        if min_z < 2.0:

            cloud_cluster = pcl.PointCloud()
            cloud_cluster.from_array(temp_cloud)
            clustered_chair_range = get_range(temp_cloud)
            clustered_bbox.append(clustered_chair_range)
        # out_data_chair_filename = save_path + "_clustered_chair_" + str(0) + ".pcd"
        # pcl.save(cloud_cluster, out_data_chair_filename)

    else:
        if len(cluster_list) == 1:

            temp_cloud = cluster_list[0].to_array()
            min_z = np.amin(temp_cloud, axis=0)[2:3]
            if min_z < 2.0:

                cloud_cluster = pcl.PointCloud()
                cloud_cluster.from_array(temp_cloud)
                clustered_chair_range = get_range(temp_cloud)
                clustered_bbox.append(clustered_chair_range)
            # out_data_chair_filename = save_path + "_clustered_chair_" + str(1) + ".pcd"
            # pcl.save(cloud_cluster, out_data_chair_filename)
        else:
            for clustered_cloud in cluster_list:
                i += 1
                temp_cloud = clustered_cloud.to_array()
                min_z = np.amin(temp_cloud, axis=0)[2:3]
                if min_z < 2.0:

                    clustered_chair_range = get_range(temp_cloud)
                    temp_area = get_area(clustered_chair_range)
                    print main_area, temp_area

                    if temp_area <= main_area / 2.0:

                        clustered_bbox.append(clustered_chair_range)
                        cloud_cluster = pcl.PointCloud()
                        cloud_cluster.from_array(temp_cloud)
                        out_data_chair_filename = save_path+"_clustered_chair_"+str(i)+".pcd"
                        pcl.save(cloud_cluster, out_data_chair_filename)
    print clustered_bbox
    return clustered_bbox

def make_table_info(cloud, save_path):


    table_range = get_range(cloud.to_list())
    cluster_list = clustering(cloud, min_cluter_size=100)
    main_area = get_area(table_range)
    i = 0

    clustered_bbox = list()
    if len(cluster_list) == 0:
        temp_cloud = cloud.to_array()

        cloud_cluster = pcl.PointCloud()
        cloud_cluster.from_array(temp_cloud)
        clustered_table_range = get_range(temp_cloud)
        clustered_bbox.append(clustered_table_range)

        # out_data_table_filename = save_path + "_clustered_table_" + str(0) + ".pcd"
        # pcl.save(cloud_cluster, out_data_table_filename)

    else:
        if len(cluster_list) == 1:

            temp_cloud = cluster_list[0].to_array()

            cloud_cluster = pcl.PointCloud()
            cloud_cluster.from_array(temp_cloud)
            clustered_table_range = get_range(temp_cloud)
            clustered_bbox.append(clustered_table_range)
            # out_data_table_filename = save_path + "_clustered_table_" + str(1) + ".pcd"
            # pcl.save(cloud_cluster, out_data_table_filename)
        else:
            for clustered_cloud in cluster_list:
                i += 1
                temp_cloud = clustered_cloud.to_array()

                clustered_table_range = get_range(temp_cloud)
                temp_area = get_area(clustered_table_range)
                print main_area, temp_area

                if temp_area <= main_area / 2.0:

                    clustered_bbox.append(clustered_table_range)
                    cloud_cluster = pcl.PointCloud()
                    cloud_cluster.from_array(temp_cloud)
                    out_data_table_filename = save_path+"_clustered_table_"+str(i)+".pcd"
                    pcl.save(cloud_cluster, out_data_table_filename)
    print clustered_bbox
    return clustered_bbox

def get_range(point_cloud, e=0.0):
    ''' 0.1 10cm'''
    point_max = np.amax(np.asarray(point_cloud), axis=0)[0:3]
    point_min = np.amin(np.asarray(point_cloud), axis=0)[0:3]

    point_max2 = np.asarray(point_max) + np.asarray([e, e, 0.0])
    point_min2 = np.asarray(point_min) - np.asarray([e, e, 0.0])

    return [point_max2, point_min2]

def get_area(bbox_range):

    min_point = bbox_range[1]
    max_point = bbox_range[0]
    side_point_1 = [max_point[0], min_point[1], min_point[2]]
    side_point_2 = [min_point[0], max_point[1], min_point[2]]
    a = np.cross(np.asarray(side_point_1)-np.asarray(min_point), np.asarray(side_point_2)-np.asarray(min_point))

    return np.linalg.norm(a)



def make_side_line(point_list, normal_vector):

    boundary_info = get_range(point_list.to_list(), 0.3)

    point_list2 = find_side_point(normal_vector, boundary_info)

    if len(point_list2) != 0:
        sorted_line_list = sorted(point_list2, key=cmp_to_key(z_point_sorting))



        match_i = list()
        for sorted_i in range(2):
            sorted_distance = list()

            for sorted_j in range(2, len(sorted_line_list)):
                distance_point = distance_to_point(sorted_line_list[sorted_i], sorted_line_list[sorted_j])
                sorted_distance.append(distance_point)

            min_i = sorted_distance.index(min(sorted_distance))
            match_i.append(min_i + 2)

        line_points = sorted_line_list[:2]
        line_points_2 = sorted_line_list[-2:]

        new_point_list2 = line_points + line_points_2


        plane_abc = np.cross(np.asarray(new_point_list2[1]) - np.asarray(new_point_list2[0]),
                             np.asarray(new_point_list2[2]) - np.asarray(new_point_list2[0]))
        plane_d = np.asarray([np.sum(plane_abc * new_point_list2[0]) * -1.0])
        new_plane_list = np.concatenate([plane_abc, plane_d]).tolist()

        point_1 = [sorted_line_list[0], sorted_line_list[match_i[0]]]
        point_2 = [sorted_line_list[1], sorted_line_list[match_i[1]]]
        line_points_list = list()
        line_points_list.append(point_1)
        line_points_list.append(point_2)


        return new_point_list2, new_plane_list, line_points_list, boundary_info
    else:
        return [], [], [], []

def make_straight(normal_vector, boundary_point, point_1, point_2, check):

    ''' Make Max Point '''
    t = Symbol('t')
    u = np.array(point_2) - np.array(point_1)

    equation = Eq(normal_vector[0] * (point_1[0] + u[0] * t) +
                  normal_vector[1] * (point_1[1] + u[1] * t) +
                  normal_vector[2] * (point_1[2] + u[2] * t) +
                  normal_vector[3], 0)
    value = solve(equation, t)[0]
    point_x = point_1[0] + (u[0] * value)
    point_y = point_1[1] + (u[1] * value)
    point_z = point_1[2] + (u[2] * value)

    point_list = [float(point_x), float(point_y), float(point_z)]

    # check_result = check_point_range_2(point_list, boundary_point, 0.075)
    check_result = check_point_range_2(point_list, boundary_point)
    # print check
    if check:
        if check_result:
            return point_list
        else:
            return []

def find_side_point(plane_vector, boundary_info):


    min_point_low   = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
    min_point_top   = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[0][2]) ]
    min_point_left  = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]
    min_point_right = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
    max_point_top   = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[0][2]) ]
    max_point_low   = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]
    max_point_right = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[0][2])]
    max_point_left  = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[0][2])]
    min_line_1 = make_straight(plane_vector, boundary_info, min_point_low, min_point_right, True)
    min_line_2 = make_straight(plane_vector, boundary_info, min_point_low, min_point_left, True)
    min_line_3 = make_straight(plane_vector, boundary_info, max_point_low, min_point_right, True)
    min_line_4 = make_straight(plane_vector, boundary_info, max_point_low, min_point_left, True)

    max_line_1 = make_straight(plane_vector, boundary_info, max_point_top, max_point_right, True)
    max_line_2 = make_straight(plane_vector, boundary_info, max_point_top, max_point_left, True)
    max_line_3 = make_straight(plane_vector, boundary_info, min_point_top, max_point_right, True)
    max_line_4 = make_straight(plane_vector, boundary_info, min_point_top, max_point_left, True)

    min_value = [min_line_1, min_line_2, min_line_3, min_line_4]
    max_value = [max_line_1, max_line_2, max_line_3, max_line_4]

    point_list = list()
    for max_index in max_value:
        if len(max_index) != 0:
            point_list.append(max_index)
    for min_index in min_value:
        if len(min_index) != 0:
            point_list.append(min_index)

    if len(point_list) == 4:
        return point_list
    else:
        return []


def check_point_range_2(point, wall_range):

    wall_range_max = wall_range[0]
    wall_range_min = wall_range[1]

    x = point[0]
    y = point[1]
    z = point[2]
    check_X = 0
    check_Z = 0
    check_Y = 0

    if x <= wall_range_max[0]:
        if x >= wall_range_min[0]:
            check_X += 1
    if y <= wall_range_max[1]:
        if y >= wall_range_min[1]:
            check_Y += 1
    if z <= wall_range_max[2]:
        if z >= wall_range_min[2]:
            check_Z += 1

    if (check_X + check_Z + check_Y) == 3:
        return True
    else:
        return False


def check_distance_plane(point_cloud, coeff):

    a = coeff[0]
    b = coeff[1]
    c = coeff[2]
    d = coeff[3]
    t = point_cloud.to_list()
    count = 0
    for index in t:
        x = index[0]
        y = index[1]
        z = index[2]

        point_distance = float(
            math.fabs(a * x + b * y + c * z + d) / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))

        if point_distance <= 0.05:
            count += 1

    if count == 0:
        distance_rate = 0.0
    else:
        distance_rate = round(float(count) / float(len(t)), 3)
    return distance_rate


def check_bbox(main_bbox, other_bbox, e=0.0):

    m_minX = main_bbox[1][0] + e
    m_minY = main_bbox[1][1] + e
    m_minZ = main_bbox[1][2]
    m_maxX = main_bbox[0][0] - e
    m_maxY = main_bbox[0][1] - e
    m_maxZ = main_bbox[0][2]

    o_minX = other_bbox[1][0] + e
    o_minY = other_bbox[1][1] + e
    o_minZ = other_bbox[1][2]
    o_maxX = other_bbox[0][0] - e
    o_maxY = other_bbox[0][1] - e
    o_maxZ = other_bbox[0][2]

    return (m_minX <= o_maxX and m_maxX >= o_minX) and (m_minY <= o_maxY and m_maxY >= o_minY) and (m_minZ <= o_maxZ and m_maxZ >= o_minZ)

def make_rectangle(rectangle_point):
    side1_bottom = rectangle_point[0][0]
    side1_top = rectangle_point[0][1]

    side2_bottom = rectangle_point[1][0]
    side2_top = rectangle_point[1][1]

    line_top = (np.asarray(side1_top) - np.asarray(side2_top)).tolist() + side2_top
    line_bottom = (np.asarray(side1_bottom) - np.asarray(side2_bottom)).tolist() + side2_bottom
    # line_side1 = (np.asarray(side1_top) - np.asarray(side1_bottom)).tolist() + side1_bottom
    # line_side2 = (np.asarray(side2_top) - np.asarray(side2_bottom)).tolist() + side2_bottom

    return [line_top, line_bottom]

def get_intersection_line(point_cloud, normal_vector, rectangle_point, bbox_info):
    '''

    :param point_cloud:
    :param normal_vector:
    :param line_point: (min point, max point), (min_point, max_point)
    :return:
    '''
    x = Symbol('x')
    y = Symbol('y')
    z = 0.0
    line_list = list()
    test_list = [[] for i in range(len(point_cloud))]
    check_bbox_index = list()
    for line_i in range(len(normal_vector)):
        line_temp = list()
        line_list.append(line_temp)
        line_list[line_i].append(rectangle_point[line_i][0])
        line_list[line_i].append(rectangle_point[line_i][1])
    for point_i in range(len(point_cloud)):
        temp_bbox = list()
        temp_bbox.append(point_i)
        for point_j in range(len(point_cloud)):
            if point_j != point_i:
                if check_bbox(bbox_info[point_i], bbox_info[point_j]):
                    temp_bbox.append(point_j)
        check_bbox_index.append(temp_bbox)
    # for point_i in range(len(point_cloud)):
    #     temp_bbox = list()
    #     for point_j in range(len(point_cloud)):
    #         if point_j != point_i:
    #             if check_bbox(get_range(point_cloud[point_i]), get_range(point_cloud[point_j])):
    #                 temp_bbox.append(point_j)
    #     check_bbox_index.append(temp_bbox)
    for index_i in range(len(normal_vector)):
        main_range = get_range(point_cloud[index_i])
        if len(check_bbox_index[index_i]) != 0:
            for check_i in check_bbox_index[index_i]:
                model_cos = np.dot(
                            [normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]],
                            [normal_vector[check_i][0], normal_vector[check_i][1], normal_vector[check_i][2]]) \
                                    / (np.linalg.norm(
                            [normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]])
                                       * np.linalg.norm(
                                    [normal_vector[check_i][0], normal_vector[check_i][1], normal_vector[check_i][2]]))
                if math.fabs(round(model_cos, 1)) != 1.0:

                    temp_list = np.cross(
                                        [normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]],
                                        [normal_vector[check_i][0], normal_vector[check_i][1], normal_vector[check_i][2]])
                    e1 = Eq(
                        normal_vector[index_i][0] * x + normal_vector[index_i][1] * y + normal_vector[index_i][3],
                        0)
                    e2 = Eq(
                        normal_vector[check_i][0] * x + normal_vector[check_i][1] * y + normal_vector[check_i][3],
                        0)
                    value_eq = solve([e1, e2], x, y)
                    temp_list = temp_list.tolist()
                    temp_list.append(value_eq[x])
                    temp_list.append(value_eq[y])
                    temp_list.append(z)

                    temp_point_list = list()
                    main_minZ = main_range[1][2]
                    main_maxZ = main_range[0][2]
                    minT = (z - temp_list[5]) / temp_list[2]
                    minX = (minT * temp_list[0]) + temp_list[3]
                    minY = (minT * temp_list[1]) + temp_list[4]

                    point_1 = [minX, minY, main_minZ]
                    point_2 = [minX, minY, main_maxZ]
                    # minT = (main_minZ - temp_list[5]) / temp_list[2]
                    # minX = (minT * temp_list[0]) + temp_list[3]
                    # minY = (minT * temp_list[1]) + temp_list[4]
                    #
                    # maxT = (main_maxZ - temp_list[5]) / temp_list[2]
                    # maxX = (maxT * temp_list[0]) + temp_list[3]
                    # maxY = (maxT * temp_list[1]) + temp_list[4]
                    # point_1 = [minX, minY, main_minZ]
                    # point_2 = [maxX, maxY, main_maxZ]

                    print check_point_range_2(point_1, main_range), check_point_range_2(point_2, main_range)
                    if check_point_range_2(point_1, main_range):
                        temp_point_list.append(point_1)
                    if check_point_range_2(point_2, main_range):
                        temp_point_list.append(point_2)

                    if len(temp_point_list) == 2:
                        sorted_point_list = sorted(temp_point_list, key=cmp_to_key(z_point_sorting))
                        rectangle_point[index_i].insert(1, sorted_point_list)
                        # line_list[index_i].insert(1, sorted_point_list)
                        test_list[index_i].append(sorted_point_list)

    return rectangle_point
    # return line_list


def visual_graph(point_list):
    x = []
    y = []
    # print len(e)
    for index in point_list:
        print index
        # for i in index:

        x.append(index[0])
        y.append(index[2])
    plt.scatter(x, y, label="stars", color="green",
                marker="*", s=50)
    plt.plot(x, y)
    plt.legend()
    plt.show()
    # x = []
    # y = []
    # # print len(e)
    # for index in point_list:
    #     x.append(index[0])
    #     y.append(index[2])
    # plt.scatter(x, y, label="stars", color="green",
    #             marker="*", s=50)
    # plt.plot(x, y)
    # plt.legend()
    # plt.show()
def distance_to_point(point1, point2):
    point1_x = point1[0]
    point1_y = point1[1]
    point1_z = point1[2]
    point2_x = point2[0]
    point2_y = point2[1]
    point2_z = point2[2]
    a = math.pow(point1_x - point2_x, 2)
    b = math.pow(point1_y - point2_y, 2)
    c = math.pow(point1_z - point2_z, 2)
    d = math.sqrt(a + b + c)

    return d

def z_point_sorting(x, y):
    if x[2] <= y[2]:
        return -1
    else:
        return 1
def cmp_to_key(mycmp):
    'Convert a cmp= function into a key= function'

    class K:
        def __init__(self, obj, *args):
            self.obj = obj

        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0

        def __gt__(self, other):
            return mycmp(self.obj, other.obj) >= 0
    return K

# def make_area(test_a):
#     print test_a
#
#     for i in test_a:
#         root_index = i.pop(0)
#         print i, test_a[]
#





if __name__ == "__main__":

    start_vect = time.time()

    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/merge_colorized_subsampled_wall.pcd")
    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/pointcloud_05_wall.pcd")
    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/pointcloud_05_01.ply")
    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/test_01_data.ply")
    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/conferenceRoom_1_wall.ply")

    # cloud = pcl.load("/home/dprt/Desktop/Demo/IOU_test/Untitled Folder/colorized_wall.ply")
    cloud = pcl.load("/home/dprt/Desktop/Sigspatial_LBJ/booth6.ply")
    # a = clustering(cloud)

    a = make_wall_info(cloud)

    first_process_runtime = (time.time() - start_vect) / 60
    # print first_process_runtime

    print("plane RANSAC Process Runtime: %0.2f Minutes" % (first_process_runtime))
    poly2obj = po.Ply2Obj(a)
    poly2obj.poly_2_obj('All')
    dae_filename = '/home/dprt/Desktop/SC_DEMO/New Folder/test/each/all.dae'
    poly2obj.output_dae(dae_filename)
    #
    #
    # a = [
    # [0, 20, 22, 24, 49, 57],
    # [1, 21, 43, 46],
    # [2, 11, 18, 26, 31],
    # [3],
    # [4, 21, 43],
    # [5, 16, 28, 33, 35, 37, 56],
    # [6, 9],
    # [7],
    # [8],
    # [9, 6, 34],
    # [10, 13, 60, 64],
    # [11, 2, 26, 31, 44],
    # [12, 25, 30, 36],
    # [13, 10, 17, 64],
    # [14, 22, 24, 50],
    # [15, 21, 46],
    # [16, 5, 35, 37, 56],
    # [17, 13, 36, 60],
    # [18, 2, 44],
    # [19],
    # [20, 0, 32, 39, 47],
    # [21, 1, 4, 15],
    # [22, 0, 14],
    # [23, 33, 45],
    # [24, 0, 14, 50, 57],
    # [25, 12, 30, 36],
    # [26, 2, 11, 31, 44],
    # [27, 38, 45, 54],
    # [28, 5, 33, 37, 52, 54],
    # [29, 47, 49, 53],
    # [30, 12, 25, 36],
    # [31, 2, 11, 26, 44],
    # [32, 20],
    # [33, 5, 23, 28, 37, 45],
    # [34, 9, 40],
    # [35, 5, 16, 37, 56],
    # [36, 12, 17, 25, 30],
    # [37, 5, 16, 28, 33, 35, 56],
    # [38, 27, 45],
    # [39, 20],
    # [40, 34],
    # [41, 42],
    # [42, 41],
    # [43, 1, 4, 46],
    # [44, 11, 18, 26, 31],
    # [45, 23, 27, 33, 38],
    # [46, 1, 15, 43],
    # [47, 20, 29],
    # [48],
    # [49, 0, 29],
    # [50, 14, 24],
    # [51, 59],
    # [52, 28, 54],
    # [53, 29],
    # [54, 27, 28, 52],
    # [55],
    # [56, 5, 16, 35, 37],
    # [57, 0, 24],
    # [58],
    # [59, 51],
    # [60, 10, 17, 64],
    # [61],
    # [62],
    # [63],
    # [64, 10, 13, 60]]
    #
    # b = [[1,2,3,6,7], [2,1,3], [3,2,4], [4,1,3], [5,6,7], [6,1,5], [7,1,5]]
    # make_area(b)
