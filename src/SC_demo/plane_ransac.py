import numpy as np
import pcl
import random
import pcl.pcl_visualization
import math
from sympy import Symbol, solve, Eq
import matplotlib.pyplot as plt
import Point_sort as ps
import Point_sort2 as ps2
import time
from shapely.geometry import Polygon
from shapely.geometry import box

from graph_cycles import MakingGraph
from graph_test import MakingGraph2
import ply2obj as po

count_index = 0


def clustering(cloud, vg_size=0.1, tolerance=0.5, min_cluster_size=1000):
    """Clustering the orginal point cloud data

        The voxelization of the original data for the downsampling.
        Extract clustered data from the voxelization data.

        Args:
            cloud: Original point cloud data
            vg_size: The voxel size to be voxelized
            tolerance: Cluster tolerance variable
            min_cluster_size: Minimum number of points in clustered data

        Returns:
            cluster_list: List of the clustered cloud point data
    """

    # voxelization using voxel_gird_filter() - downsampling
    # original points = 205620 -> after 24030


    cluster_list = list()
    cloud_value = cloud
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(vg_size, vg_size, vg_size)
    cloud_filtered = vg.filter()

    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tolerance) # 0.5 = 50cm
    ec.set_MinClusterSize(min_cluster_size) #  impose that the clusters found must have at least
    ec.set_MaxClusterSize(cloud_value.size) #  impose that the clusters found must have at Maximum
    ec.set_SearchMethod(tree)


    cluster_indices = ec.Extract() # return index number that is included in the result of clustering

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



# def do_passthrough_filter( point_cloud, name_axis='z'):
#
#     # Create a passthrough filter that removes points outside of the range 0 to 1.5 in the Z axis.
#     pass_filter = point_cloud.make_passthrough_filter()
#     pass_filter.set_filter_field_name(name_axis)
#
#     return pass_filter.filter()

def do_plane_ransac(cloud):
    """Finding the plane from point cloud data

        Calculate the surface normals for each point by fitting a plane to the nearest.

    Args:
        cloud: Clustered point cloud data

    Returns:
        inliers: Pointcloud data extracted by plane
        outliers: Pointcloud data that is not extracted by plane
        coefficients: Coefficient data of plane equation(a, b, c, d)
    """


    # Calculate the surface normals for each point by fitting a plane to the nearest
    # 50 neighbours to the candidate point.

    segmenter = cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE) # Fit a plane to the points.
    segmenter.set_optimize_coefficients(True)  # Do a little bit more optimisation once the plane has been fitted.
    segmenter.set_normal_distance_weight(0.05)
    segmenter.set_method_type(pcl.SAC_RANSAC)  # Use RANSAC for the sample consensus algorithm.
    segmenter.set_max_iterations(100000)  # Number of iterations for the RANSAC algorithm.
    segmenter.set_distance_threshold(0.05) # The max distance from the fitted model a point can be for it to be an inlier.
    #0.05 / 100000 / 0.05
    inlier_indices, coefficients = segmenter.segment() # Returns all the points that fit the model, and the parameters of the model.

    # Save all the inliers as a point cloud. This forms the table which the mug sits on.
    inliers = cloud.extract(inlier_indices, negative=False)

    # Save all the outliers as a point cloud.
    outliers = cloud.extract(inlier_indices, negative=True)

    return inliers, outliers, coefficients

def do_plane_ransac2(cloud):
    """Finding the plane from point cloud data

        Calculate the surface normals for each point by fitting a plane to the nearest.

    Args:
        cloud: Clustered point cloud data

    Returns:
        inliers: Pointcloud data extracted by plane
        outliers: Pointcloud data that is not extracted by plane
        coefficients: Coefficient data of plane equation(a, b, c, d)
    """


    # Calculate the surface normals for each point by fitting a plane to the nearest
    # 50 neighbours to the candidate point.

    segmenter = cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_model_type(pcl.SACMODEL_PLANE) # Fit a plane to the points.
    segmenter.set_optimize_coefficients(True)  # Do a little bit more optimisation once the plane has been fitted.
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)  # Use RANSAC for the sample consensus algorithm.
    segmenter.set_max_iterations(1000)  # Number of iterations for the RANSAC algorithm.
    segmenter.set_distance_threshold(0.1) # The max distance from the fitted model a point can be for it to be an inlier.
    #0.05 / 100000 / 0.05
    inlier_indices, coefficients = segmenter.segment() # Returns all the points that fit the model, and the parameters of the model.

    # Save all the inliers as a point cloud. This forms the table which the mug sits on.
    inliers = cloud.extract(inlier_indices, negative=False)

    # Save all the outliers as a point cloud.
    outliers = cloud.extract(inlier_indices, negative=True)

    return inliers, outliers, coefficients


def check_distance_plane(point_cloud, coeff, e=0.01):
    """Checking the distance between point and plane

    Find the number of points where the distance between the planes is 0.05 or less
    and calculate the percentage.

    Args:
        point_cloud: Pointcloud data extracted by plane
        coeff: plane: Coefficient data of plane equation(a, b, c, d)
    Returns:
        distance_rate: Percentage of points with a distance of 0.05 or less
    """
    # checking the rate between point cloud and plane information
    a = coeff[0]
    b = coeff[1]
    c = coeff[2]
    d = coeff[3]
    t = point_cloud
    count = 0
    for index in t:
        x = index[0]
        y = index[1]
        z = index[2]

        point_distance = float(
            math.fabs(a * x + b * y + c * z + d) / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))

        if point_distance <= e:
            count += 1

    if count == 0:
        distance_rate = 0.0
    else:
        distance_rate = round(float(count) / float(len(t)), 3)
    return distance_rate * 100, count

def plane_cluster(cloud_data):
    """Clustering the exported plane data

        Finding clustered points in the data extracted as planes.

    Args:
        cloud_data: Pointcloud data extracted by plane
    Returns:
        new_cloud_data: Clustered PointCloud data list
        normal_vector: Plane coefficient list of clustered PointCloud data
    """
    max_size = cloud_data.size
    min_size = 200
    new_cloud_data = list()
    normal_vector = list()
    outliers_data = pcl.PointCloud()
    tree = cloud_data.make_kdtree()
    segment = cloud_data.make_EuclideanClusterExtraction()
    segment.set_ClusterTolerance(0.5)
    segment.set_MinClusterSize(min_size)
    segment.set_MaxClusterSize(max_size)
    segment.set_SearchMethod(tree)
    cluster_indices = segment.Extract()
    if len(cluster_indices) != 0:
        inliers = cloud_data.extract(cluster_indices[0], negative=False)
        outliers = cloud_data.extract(cluster_indices[0], negative=True)
        if inliers > min_size:
            inliers_p, outliers_p, coeff_p = do_plane_ransac(inliers)
            new_cloud_data.append(inliers_p)
            normal_vector.append(coeff_p)
            outliers_data = outliers
    return new_cloud_data, normal_vector, outliers_data

    # while True:
    #     if cloud_data.size > min_size:
    #         tree = cloud_data.make_kdtree()
    #         segment = cloud_data.make_EuclideanClusterExtraction()
    #         segment.set_ClusterTolerance(0.5)
    #         segment.set_MinClusterSize(min_size)
    #         segment.set_MaxClusterSize(max_size)
    #         segment.set_SearchMethod(tree)
    #         cluster_indices = segment.Extract()
    #         if len(cluster_indices) != 0:
    #             inliers = cloud_data.extract(cluster_indices[0], negative=False)
    #             outliers = cloud_data.extract(cluster_indices[0], negative=True)
    #             if inliers > min_size:
    #                 inliers_p, outliers_p, coeff_p = do_plane_ransac(inliers)
    #                 new_cloud_data.append(inliers_p)
    #                 normal_vector.append(coeff_p)
    #                 cloud_data = outliers
    #             else:
    #                 if outliers.size > min_size:
    #                     cloud_data = outliers
    #                 else:
    #                     break
    #         else:
    #             break
    #     else:
    #         break
    # if len(cloud_point) >= 2:
    #     visual_viewer(cloud_point)
    # return new_cloud_data, normal_vector
def merge_dup_plane(plane_list, normal_vector):
    """Merging the parallel planes

        If the planes are parallel and the bounding boxes overlap, they are merged into one plane.

    Args:
        plane_list: PointCloud data list
        normal_vector: Coefficient data list of each plane

    Returns:
        new_plane_list: Data list of newly created PointClouds
        new_normal_vector: Coefficient data list of each planes
        new_bbox_list: Bounding box information of each PointCloud
    """
    new_plane_list = list()
    new_normal_vector = list()
    new_bbox_list = list()
    prev_bbox = [[] for i in range(len(plane_list))]
    dup_plane_index = list()
    for i in range(len(plane_list) - 1):
        for j in range(i + 1, len(plane_list)):
            if len(prev_bbox[i]) == 0:
                main_bbox = get_range(plane_list[i])
                prev_bbox[i].extend(main_bbox)
            else:
                main_bbox = prev_bbox[i]
            if len(prev_bbox[j]) == 0:
                sub_bbox = get_range(plane_list[j])
                prev_bbox[j].extend(sub_bbox)
            else:
                sub_bbox = prev_bbox[j]

            if check_bbox(main_bbox, sub_bbox):
                model_cos = np.dot(
                    [normal_vector[i][0], normal_vector[i][1], normal_vector[i][2]],
                    [normal_vector[j][0], normal_vector[j][1], normal_vector[j][2]]) \
                            / (np.linalg.norm(
                    [normal_vector[i][0], normal_vector[i][1], normal_vector[i][2]])
                               * np.linalg.norm(
                            [normal_vector[j][0], normal_vector[j][1], normal_vector[j][2]]))

                if math.fabs(round(model_cos, 1)) == 1.0:

                    main_point = plane_list[i].to_list()[0]
                    distance_bw_planes = math.fabs(normal_vector[j][0] * main_point[0] + normal_vector[j][1] * main_point[1] + normal_vector[j][2] * main_point[2] + normal_vector[j][3]) / \
                                        math.sqrt(math.pow(normal_vector[j][0],2) + math.pow(normal_vector[j][1],2) + math.pow(normal_vector[j][2],2))
                    # print model_cos, distance_bw_planes, i, j, check_distance_plane(plane_list[i], normal_vector[i]), check_distance_plane(plane_list[j], normal_vector[j])
                    if distance_bw_planes <= 0.05:
                        if len(dup_plane_index) == 0:
                            dup_plane_index.append([i, j])
                        else:
                            check_value = False
                            for dup_index in dup_plane_index:
                                if i in dup_index:
                                    if j not in dup_index:
                                        dup_index.append(j)
                                        check_value = True
                                        break
                            if check_value == False:
                                dup_plane_index.append([i, j])
    # print dup_plane_index
    if len(dup_plane_index) != 0:
        for dup_index in dup_plane_index:
            print dup_index
            max_i = dup_index[0]
            merge_point = plane_list[dup_index[0]].to_list()
            for each_i in range(1, len(dup_index)):
                if plane_list[max_i].size < plane_list[dup_index[each_i]].size:
                    max_i = dup_index[each_i]
                merge_point.extend(plane_list[dup_index[each_i]].to_list())
            merge_plane = pcl.PointCloud()
            merge_plane.from_list(merge_point)
            new_plane_list.append(merge_plane)
            new_normal_vector.append(normal_vector[max_i])
            new_bbox_list.append(prev_bbox[max_i])
        all_dup_index = reduce(lambda x, y: x+y, dup_plane_index)

        # print all_dup_index
        for each_plane_i in range(len(plane_list)):
            # print each_plane_i, each_plane_i not in all_dup_index
            if each_plane_i not in all_dup_index:
                new_plane_list.append(plane_list[each_plane_i])
                new_normal_vector.append(normal_vector[each_plane_i])
                new_bbox_list.append(prev_bbox[each_plane_i])
                # print len(new_plane_list), len(new_normal_vector), len(new_bbox_list)

        return new_plane_list, new_normal_vector, new_bbox_list
    else:
        return plane_list, normal_vector, prev_bbox



def get_plane_list(clustered_cloud):
    """Planar data extraction from clustered cloud

    Execute the loop statement until the cloud size is greater than
    or equal to the clustered_cloud size * min_percentage value.
    During loop statement execution, Plane RANSAC is executed.
    Maintain a loop statement by updating information that is not extracted by plane with cloud data.
    Args:
        clustered_cloud: Clustered data from original data

    Returns:
        new_plane_list: List of Pointcloud data extracted by plane
        new_normal_vector: Coefficient data list of each planes
        new_bbox_list: Bounding box information of each PointCloud

    """

    plane_list = list()
    normal_vector = list()

    cloud = clustered_cloud


    original_size = cloud.size
    min_percentage = 10

    while True:
        # print cloud.height * cloud.width < original_size * min_percentage / 100

        if cloud.height * cloud.width < 100:

            break

        inliers_p, outliers_p, coeff_p = do_plane_ransac(cloud)
        default_vector = [0, 0, -1]
        model_value = [coeff_p[0], coeff_p[1], coeff_p[2]]
        model_cos = np.dot(default_vector, model_value) / (
                np.linalg.norm(model_value) * np.linalg.norm(default_vector))
        #
        if math.fabs(model_cos) < 0.8:
            clustered_plane_list, clustered_plane_coeff, outliers_data = plane_cluster(inliers_p)
            if len(clustered_plane_list) != 0:
                plane_list.extend(clustered_plane_list)
                normal_vector.extend(clustered_plane_coeff)
                new_outliers_p = pcl.PointCloud()
                new_outliers_p.from_list(outliers_data.to_list() + outliers_p.to_list())
                cloud = new_outliers_p
            else:
                cloud = outliers_p
        else:
            cloud = outliers_p


    new_plane_list, new_normal_vector, new_bbox_list = merge_dup_plane(plane_list, normal_vector)
    new_plane_list2 = []
    new_bbox_list2 = []
    new_normal_vector2 = []
    for each_wall in new_plane_list:
        fil = each_wall.make_statistical_outlier_filter()
        fil.set_mean_k(50)
        fil.set_std_dev_mul_thresh(1.0)
        new_cloud = fil.filter()

        inliers_p, outliers_p, coeff_p = do_plane_ransac2(new_cloud)
        new_plane_list2.append(inliers_p)

        new_normal_vector2.append(coeff_p)
        new_bbox_list2.append(get_range(new_cloud))

    return new_plane_list2, new_normal_vector2, new_bbox_list2


def make_wall_info(cloud):
    """Making the wall information using PointCloud data

        Clustering the original point cloud data.
        Extracting the plane data from the clustered data through Plane RANSAC
        Find intersections between extracted plane data and generate surface data

    Args:
        cloud: Original PointCloud data

    Returns:
        surface_point_list: A list consisting of a list of points that make up each surface
    """

    print "Make Wall Info"

    # clustering the pointcloud data
    cluster_list = clustering(cloud)

    room_surface_list = list()
    p_s = ps.Point_sort()



    all_room = []
    for clustered_cloud in cluster_list:
        # Exporting the plane data from each clustered cloud data
        wall_point_list, wall_vector_list, wall_bbox_list = get_plane_list(clustered_cloud)
        # visual_viewer(wall_point_list)
        print len(wall_point_list), len(wall_vector_list), len(wall_bbox_list)

        # print "side_line_info"
        side_line_info = []
        for wall_index in range(len(wall_point_list)):

            #
            print wall_index
            # a, b = make_side_line(wall_bbox_list[wall_index], wall_vector_list[wall_index])
            side_line_info.append(make_side_line(wall_bbox_list[wall_index], wall_vector_list[wall_index]))


            # # print side_line_info
            # # for a_i in side_line_info:
            # #     a.extend(a_i)
            # # p_s.visual_graph(a)
            # # side_line_list.append(side_line_info)
            # # side_line_count.append(len(side_line_info))
            #
            # print "side line_length : ", len(side_line_info[wall_index]), side_line_info[wall_index]

            # print wall_bbox_list[wall_index]
            # box_info = box(wall_bbox_list[wall_index][1][0], wall_bbox_list[wall_index][1][1], wall_bbox_list[wall_index][0][0], wall_bbox_list[wall_index][0][1])
            # box_coords = box_info.exterior.coords
            #
            # test_box = []
            # for b_i in box_coords:
            #
            #     test_box.append([b_i[0], b_i[1], wall_bbox_list[wall_index][0][2]])
            #     test_box.append([b_i[0], b_i[1], wall_bbox_list[wall_index][1][2]])
            # print test_box
            # print b
            # a = pcl.PointCloud()
            # a.from_list(test_box)
            # pcl.save(a, "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/bbox_" + str(wall_index) + ".pcd")
        #     pcl.save(a, "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/s2_" + str(wall_index) + ".pcd")
        # print side_line_info

            # wall_point_list[wall_index]._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/" + str(wall_index) + ".ply")


            # pcl.save(fil.filter(), "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/f_" + str(wall_index) + ".pcd")
        # print side_line_list
        wall_surface_list = get_intersection_line(wall_bbox_list, wall_vector_list, side_line_info, wall_point_list)

        #
        # all_a = []
        # count = 0
        # for w_i in wall_surface_list:

            # t_w_i = len(w_i)
            # print "original size : ", t_w_i
            # if len(w_i) != 0:
            #
            #     w_i.pop(0)
            #     w_i.pop(0)
            #     all_a.append(w_i)
            #     test_point = pcl.PointCloud().from_list(w_i)
            #     test_point._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/"+str(count)+".ply")
            #
            # else:
            #     continue
        #
        #     # else:
        #     #     if len(w_i) == 2:
        #     #
        #
        #
        #     count = count + 1
        #

            # a = []


            # for w in w_i:
            #     a.extend(w)
            # print a
            # c = p_s.SortPointsClockwise(a, True)
            # surface_point_list.append(c)
        # print surface_point_list
        # print "surface_point_list count", len(surface_point_list), len(wall_surface_list)
    #     all_room.append(all_a)
    # print(all_room)

    # test_graph = MakingGraph(all_room)
    # test_graph.make_first_graph()
    #     #
        # for wall_i in range(len(wall_surface_list)):
        #     if side_line_count[wall_i] == 0:
        #         merge_wall_info = reduce(lambda x, y: x+y, wall_surface_list[wall_i])
        #     else:
        #         if len(wall_surface_list[wall_i]) >= 4:
        #             wall_surface_list[wall_i].pop(0)
        #             wall_surface_list[wall_i].pop(0)
        #         merge_wall_info = reduce(lambda x, y: x+y, wall_surface_list[wall_i])
        #     sorted_wall_info = p_s.SortPointsClockwise(merge_wall_info, True)
        #
        #     # p_s.visual_graph(sorted_wall_info)




    #     wall_surface_list = get_intersection_line(new_point_info, new_plane_info, new_line_info)
    #     print "Make surface of the clustered data  : ", str(cluster_list.index(clustered_cloud)+1)
    #     print wall_surface_list
    #     new_wall_surface = []
    #     for wall_info in wall_surface_list:
    #
    #         if len(wall_info) != 0:
    #
    #             if len(wall_info) == 2:
    #                 line_1 = wall_info[0]
    #                 line_2 = wall_info[1]
    #
    #                 rectangle_points = line_1 + line_2
    #                 rectangle_points = np.asarray(rectangle_points)
    #
    #                 sorted_surface_info = ring_sort.SortPointsClockwise(rectangle_points.tolist(), False)
    #                 sorted_surface_info_2 = ring_sort.SortPointsClockwise(rectangle_points.tolist(), True)
    #                 # new_wall_surface.extend(sorted_surface_info)
    #                 surface_point_list.append(sorted_surface_info)
    #                 surface_point_list.append(sorted_surface_info_2)
    #
    #                 # new_wall_surface.extend(rectangle_points.tolist())
    #             elif len(wall_info) > 2:
    #
    #                 rectangle_points = get_rectangle_points(wall_info)
    #                 rectangle_points = np.asarray(rectangle_points)
    #
    #
    #                 sorted_surface_info = ring_sort.SortPointsClockwise(rectangle_points.tolist(), False)
    #                 sorted_surface_info_2 = ring_sort.SortPointsClockwise(rectangle_points.tolist(), True)
    #                 # new_wall_surface.extend(sorted_surface_info)
    #                 surface_point_list.append(sorted_surface_info)
    #                 surface_point_list.append(sorted_surface_info_2)
    #
    #
    #
    #                 # new_wall_surface.extend(rectangle_points.tolist())
    #             else:
    #                 print wall_info
    #
    #

    # return surface_point_list

def make_chair_info(cloud, save_path):
    """Making the chair information

        Clustering the chair data.
        And returns the bounding box of each clustered data

    Args:
        cloud: PointCloud data of chair
        save_path: Save path of the clustered data
    Returns:
        clustered_bbox: The bounding box of each clustered data
    """
    chair_range = get_range(cloud.to_list())
    cluster_list = clustering(cloud, vg_size=0.05, tolerance=0.1, min_cluster_size=100)
    main_area = get_area(chair_range)
    i = 0

    clustered_bbox = list()
    if len(cluster_list) == 0:
        temp_cloud = cloud.to_array()

        cloud_cluster = pcl.PointCloud()
        cloud_cluster.from_array(temp_cloud)
        clustered_chair_range = get_range(temp_cloud)
        clustered_bbox.append(clustered_chair_range)
        # out_data_chair_filename = save_path + "_clustered_chair_" + str(0) + ".pcd"
        # pcl.save(cloud_cluster, out_data_chair_filename)

    else:
        if len(cluster_list) == 1:

            temp_cloud = cluster_list[0].to_array()

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

                clustered_chair_range = get_range(temp_cloud)
                temp_area = get_area(clustered_chair_range)

                if temp_area <= main_area / 2.0:
                    clustered_bbox.append(clustered_chair_range)
                    cloud_cluster = pcl.PointCloud()
                    cloud_cluster.from_array(temp_cloud)
                    out_data_chair_filename = save_path + "_clustered_chair_" + str(i) + ".pcd"
                    pcl.save(cloud_cluster, out_data_chair_filename)

    return clustered_bbox

def get_range(point_cloud, e=0.0):
    """Making the bouning box information

        Finding the Minimum and Maximum points of X, Y and Z
        and add the epsilon value
    Args:
        point_cloud: PointCloud data
        e: Epsilon value

    Returns:
        [point_max2, point_min2]: Minimum and Maximum points of X, Y and Z
    """
    ''' 0.1 10cm'''
    point_max = np.amax(point_cloud, axis=0)[0:3]
    point_min = np.amin(point_cloud, axis=0)[0:3]

    point_max2 = point_max + np.asarray([e, e, 0.0])
    point_min2 = point_min - np.asarray([e, e, 0.0])
    # point_max = np.amax(np.asarray(point_cloud), axis=0)[0:3]
    # point_min = np.amin(np.asarray(point_cloud), axis=0)[0:3]
    #
    # point_max2 = np.asarray(point_max) + np.asarray([e, e, 0.0])
    # point_min2 = np.asarray(point_min) - np.asarray([e, e, 0.0])

    return [point_max2, point_min2]

def get_area(bbox_range):
    """Calculate the size of the area

        Calculate the size of the area with data using a bounding box.

    Args:
        bbox_range: Bounding box information of PointCloud

    Returns:
        area_info: size of the area
    """
    min_point = bbox_range[1]
    max_point = bbox_range[0]
    side_point_1 = [max_point[0], min_point[1], min_point[2]]
    side_point_2 = [min_point[0], max_point[1], min_point[2]]
    a = np.cross(np.asarray(side_point_1) - np.asarray(min_point), np.asarray(side_point_2) - np.asarray(min_point))
    area_info = np.linalg.norm(a)
    return area_info

def make_side_line(bbox_info, normal_vector):
    """Making side lines of plane

        Using the bounding box and plane data, create straight lines at both ends of the plane.

    Args:
        bbox_info: Bounding box data of PointCloud
        normal_vector: Coefficient data of plane

    Returns:
        line_points_list: A list of the maximum and minimum points that can be made with straight lines at both ends
    """

    side_line_points = find_side_point(normal_vector, bbox_info)
    print "side_line_points : ", side_line_points
    return side_line_points
    # if len(side_line_points) != 0:
    #     sorted_line_list = sorted(side_line_points, key=cmp_to_key(z_point_sorting))
    #
    #     match_i = list()
    #     for sorted_i in range(2):
    #         sorted_distance = list()
    #
    #         for sorted_j in range(2, len(sorted_line_list)):
    #             distance_point = distance_to_point(sorted_line_list[sorted_i], sorted_line_list[sorted_j])
    #             sorted_distance.append(distance_point)
    #
    #         min_i = sorted_distance.index(min(sorted_distance))
    #         match_i.append(min_i + 2)
    #
    #     point_1 = [sorted_line_list[0], sorted_line_list[match_i[0]]]
    #     point_2 = [sorted_line_list[1], sorted_line_list[match_i[1]]]
    #     line_points_list = list()
    #     line_points_list.append(point_1)
    #     line_points_list.append(point_2)
    #
    #     return line_points_list
    # else:
    #     return []

def make_straight(normal_vector, point_1, point_2):
    """Making the straight information and finding the intersect point

        Generate straight line data using point_1 and point_2.
        Finding the intersection point using normal_vector and straight line.

    Args:
        normal_vector: Coefficient data of plane
        boundary_point: Bounding box data of PointCloud
        point_1: A point that generates a straight line
        point_2: A point that generates a straight line
    Returns:
        intersect_point: The intersection of a plane and a straight line
    """
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

    intersect_point = [point_x, point_y, point_z]

    # check_result = check_point_range_2(point_list, boundary_point, 0.075)
    # check_result = check_point_range_e(intersect_point, boundary_point, 0.0005)
    # print check
    return intersect_point
    # if check_result:
    #     return intersect_point
    # else:
    #     return []

def make_straight2(normal_vector, point_1, point_2, min_z, max_z):
    """Making the straight information and finding the intersect point

        Generate straight line data using point_1 and point_2.
        Finding the intersection point using normal_vector and straight line.

    Args:
        normal_vector: Coefficient data of plane
        boundary_point: Bounding box data of PointCloud
        point_1: A point that generates a straight line
        point_2: A point that generates a straight line
    Returns:
        intersect_point: The intersection of a plane and a straight line
    """
    t = Symbol('t')
    u = np.array(point_2) - np.array(point_1)

    equation = Eq(normal_vector[0] * (point_1[0] + u[0] * t) +
                  normal_vector[1] * (point_1[1] + u[1] * t) +
                  normal_vector[2] * min_z +
                  normal_vector[3], 0)
    value = solve(equation, t)[0]
    min_point_x = point_1[0] + (u[0] * value)
    min_point_y = point_1[1] + (u[1] * value)
    min_point_z = min_z
    min_intersect_point = [min_point_x, min_point_y, min_point_z]


    equation = Eq(normal_vector[0] * (point_1[0] + u[0] * t) +
                  normal_vector[1] * (point_1[1] + u[1] * t) +
                  normal_vector[2] * max_z +
                  normal_vector[3], 0)
    value = solve(equation, t)[0]
    max_point_x = point_1[0] + (u[0] * value)
    max_point_y = point_1[1] + (u[1] * value)
    max_point_z = max_z
    max_intersect_point = [max_point_x, max_point_y, max_point_z]

    return min_intersect_point, max_intersect_point

def find_side_point(plane_vector, boundary_info):
    """Create the intersection of the bounding box and the plane

        Creates the intersection of the edges and planes of the top and bottom faces of the bounding box

    Args:
        plane_vector: Coefficient data of plane
        boundary_info: Bounding box data of PointCloud

    Returns:
        point_list: List of points on both ends of a straight line
    """


    box_info = box(boundary_info[1][0], boundary_info[1][1], boundary_info[0][0], boundary_info[0][1])
    box_coords = box_info.exterior.coords
    min_z = boundary_info[1][2]
    max_z = boundary_info[0][2]
    min_value = []
    max_value = []
    min_h_distance = []
    min_l_distance = []
    max_h_distance = []
    max_l_distance = []

    for box_i in range(len(box_coords) - 1):
        min_result, max_result = make_straight2(plane_vector, box_coords[box_i], box_coords[box_i+1], min_z, max_z)
        # print box_i
        # b.append(min_result)
        # b.append(max_result)
        min_l_distance.append(math.fabs(boundary_info[1][0] - min_result[0]) + math.fabs(boundary_info[1][1] - min_result[1]))
        min_h_distance.append(math.fabs(boundary_info[1][0] - max_result[0]) + math.fabs(boundary_info[1][1] - max_result[1]))
        max_l_distance.append(math.fabs(boundary_info[0][0] - min_result[0]) + math.fabs(boundary_info[0][1] - min_result[1]))
        max_h_distance.append(math.fabs(boundary_info[0][0] - max_result[0]) + math.fabs(boundary_info[0][1] - max_result[1]))
        min_value.append(min_result)
        max_value.append(max_result)

    min_li = min_l_distance.index(min(min_l_distance))
    min_hi = min_h_distance.index(min(min_h_distance))
    max_li = max_l_distance.index(min(max_l_distance))
    max_hi = max_h_distance.index(min(max_h_distance))
    point_list = list()
    min_l = min_value[min_li]
    min_h = max_value[min_hi]
    max_l = min_value[max_li]
    max_h = max_value[max_hi]
    point_list.append(min_l)
    point_list.append(min_h)
    point_list.append(max_l)
    point_list.append(max_h)


    # min_value = [min_line_1, min_line_2, min_line_3, min_line_4]
    # max_value = [max_line_1, max_line_2, max_line_3, max_line_4]
    # for i in range(len(min_value)):
    #     math.fabs(min_value[i][0] - min_value[i+1][0]) + math.fabs(min_value[i][1] - min_value[i+1][1])
    # distance = math.fabs((boundary_info[0][0] - boundary_info[1][0]) + (boundary_info[0][1] - boundary_info[1][1]))

    # check_min = 0
    # check_max = 0
    # point_list = list()
    # used_min_index = []
    # used_max_index = []
    # print "min_value : ", min_value
    # print "max_value : ", max_value
    # print "boundary_info : ", boundary_info
    # e = 0.0
    # while check_min != 2:
    #
    #     for min_i in range(len(min_value)):
    #         if min_i not in used_min_index:
    #             if check_point_range_e(min_value[min_i], boundary_info, e):
    #                 if min_i not in used_min_index:
    #                     point_list.append(min_value[min_i])
    #                     used_min_index.append(min_i)
    #                     check_min = check_min + 1
    #     e = e + 0.00001
    #
    # e = 0.0
    # while check_max != 2:
    #
    #     for max_i in range(len(max_value)):
    #         if max_i not in used_max_index:
    #             if check_point_range_e(max_value[max_i], boundary_info, e):
    #                 if max_i not in used_max_index:
    #                     point_list.append(max_value[max_i])
    #                     used_max_index.append(max_i)
    #                     check_max = check_max + 1
    #     e = e + 0.00001

    # print "result : ", point_list
    # print "min_value", min_value
    # print "max_value", max_value
    # point_list = list()
    # for max_index in max_value:
    #     if len(max_index) != 0:
    #         point_list.append(max_index)
    # for min_index in min_value:
    #     if len(min_index) != 0:
    #         point_list.append(min_index)
    #
    # if len(point_list) == 4:
    #     return point_list
    # else:
    #     return []

    return point_list


def check_point_side(point, side_lines):

    x = point[0]
    y = point[1]

    if side_lines[0][0][0] > side_lines[1][0][0]:
        side_maxX = side_lines[0][0][0]
        side_minX = side_lines[1][0][0]
    else:
        side_maxX = side_lines[1][0][0]
        side_minX = side_lines[0][0][0]

    if side_lines[0][0][1] > side_lines[1][0][1]:
        side_maxY = side_lines[0][0][1]
        side_minY = side_lines[1][0][1]
    else:
        side_maxY = side_lines[1][0][1]
        side_minY = side_lines[0][0][1]

    check_count = 0
    if x <= side_maxX and x >= side_minX:
        check_count = check_count + 1
    if y <= side_maxY and x >= side_minY:
        check_count = check_count + 1

    if check_count == 2:
        return True
    else:
        return False


def check_point_range_2(point, wall_range):
    """Check whether the pointer is included in the bounding box

        Check the pointer's X, Y and Z values are included in the bounding box

    Args:
        point: Pointer to check
        wall_range: Information for bounding box

    Returns:
        True: The pointer is included in the bounding box
        False: The pointer is not included in the bounding box
    """
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

def check_point_range_e(point, wall_range, e=0.0):
    """Check whether the pointer is included in the bounding box

        Check the pointer's X, Y and Z values are included in the bounding box

    Args:
        point: Pointer to check
        wall_range: Information for bounding box

    Returns:
        True: The pointer is included in the bounding box
        False: The pointer is not included in the bounding box
    """
    # m_maxX = wall_range[0][0]
    # m_maxY = wall_range[0][1]
    # m_minX = wall_range[1][0]
    # m_minY = wall_range[1][1]


    wall_range_max = wall_range[0]
    wall_range_min = wall_range[1]

    x = point[0]
    y = point[1]
    z = point[2]
    check_X = 0
    check_Z = 0
    check_Y = 0

    if x <= wall_range_max[0] + e:
        if x >= wall_range_min[0] - e:
            check_X = check_X + 1
    if y <= wall_range_max[1] + e:
        if y >= wall_range_min[1] - e:
            check_Y = check_Y + 1
    # if z <= wall_range_max[2] + e:
    #     if z >= wall_range_min[2] - e:
    #         check_Z += 1
    # print check_X, check_Y
    if (check_X + check_Y) == 2:
        return True
    else:
        return False
def check_bbox(main_bbox, other_bbox):
    """Check if the bounding box intersects

        Check if the bounding box intersects using the min and max points of the bounding box.

    Args:
        main_bbox: Information of main bounding box
        other_bbox: Information of other bounding box

    Returns:
        True: Bounding boxes intersect
        False: Bounding boxes are not intersect
    """
    m_minX = main_bbox[1][0]
    m_minY = main_bbox[1][1]
    m_minZ = main_bbox[1][2]
    m_maxX = main_bbox[0][0]
    m_maxY = main_bbox[0][1]
    m_maxZ = main_bbox[0][2]

    s_minX = other_bbox[1][0]
    s_minY = other_bbox[1][1]
    s_minZ = other_bbox[1][2]
    s_maxX = other_bbox[0][0]
    s_maxY = other_bbox[0][1]
    s_maxZ = other_bbox[0][2]

    return (m_minX <= s_maxX and m_maxX >= s_minX) and (m_minY <= s_maxY and m_maxY >= s_minY) and (
                m_minZ <= s_maxZ and m_maxZ >= s_minZ)

def check_bbox2(main_bbox, other_bbox):
    """Check if the bounding box intersects

        Check if the bounding box intersects using the min and max points of the bounding box.

    Args:
        main_bbox: Information of main bounding box
        other_bbox: Information of other bounding box

    Returns:
        True: Bounding boxes intersect
        False: Bounding boxes are not intersect
    """
    m_minX = main_bbox[1][0]
    m_minY = main_bbox[1][1]
    m_maxX = main_bbox[0][0]
    m_maxY = main_bbox[0][1]


    s_minX = other_bbox[1][0]
    s_minY = other_bbox[1][1]
    s_maxX = other_bbox[0][0]
    s_maxY = other_bbox[0][1]

    m_box = box(m_minX, m_minY, m_maxX, m_maxY)
    s_box = box(s_minX, s_minY, s_maxX, s_maxY)

    # print m_box.exterior.coords
    # print s_box.exterior.coords
    # print m_box.intersects(s_box)

    return m_box.intersects(s_box)
    # check_value = 0
    # if (m_minX <= s_maxX and m_maxX >= s_minX) and (m_minY <= s_maxY and m_maxY >= s_minY):
    #     check_value = check_value + 1
    # if (s_minX <= m_maxX and s_maxX >= m_minX) and (s_minY <= m_maxY and s_maxY >= m_minY):
    #     check_value = check_value + 1
    #
    # if check_value >= 1:
    #     return True
    # else:
    #     return False

def extend_bbox(side_points, e):

    points = []
    max_points = side_points[2:]
    min_points = side_points[:2]
    for max_point in max_points:
        points.append([max_point[0]+e, max_point[1] + e, max_point[2]])
    for min_point in min_points:
        points.append([min_point[0]-e, min_point[1] - e, min_point[2]])
    new_range = get_range(points)

    return new_range




def get_intersection_line(bbox_list, normal_vector, side_line_info, wall_point_list):
    """Create intersection points between planes

        Search intersecting lines between planes where each bounding box intersects.
        Create intersection points using information from the intersection lines and Maximum and Minimum height value for each plane

    Args:
        bbox_list: Bounding box information of each plane
        normal_vector: Coefficient data list of each plane
        side_line_list: Side lines list of each plane
    Returns:
        side_line_list: side_line_list with each line intersection line information added
    """


    check_bbox_index = [[] for i in range(len(normal_vector))]

    check_wall_info = [[] for i in range(len(normal_vector))]
    each_wall_info = [[] for i in range(len(normal_vector))]



    for point_i in range(len(normal_vector)):
        for point_j in range(len(normal_vector)):
            if point_i != point_j and point_j not in check_wall_info[point_i]:
                model_cos = np.dot(
                    [normal_vector[point_i][0], normal_vector[point_i][1], normal_vector[point_i][2]],
                    [normal_vector[point_j][0], normal_vector[point_j][1], normal_vector[point_j][2]]) \
                            / (np.linalg.norm(
                    [normal_vector[point_i][0], normal_vector[point_i][1], normal_vector[point_i][2]])
                               * np.linalg.norm(
                            [normal_vector[point_j][0], normal_vector[point_j][1],
                             normal_vector[point_j][2]]))
                if math.fabs(round(model_cos, 1)) != 1.0:
                    if point_j not in check_wall_info[point_i]:
                        check_wall_info[point_i].append(point_j)
                    if point_i not in check_wall_info[point_j]:
                        check_wall_info[point_j].append(point_i)

                    if check_bbox2(bbox_list[point_i], bbox_list[point_j]):
                        if point_j not in each_wall_info[point_i]:
                            each_wall_info[point_i].append(point_j)
                        if point_i not in each_wall_info[point_j]:
                            each_wall_info[point_j].append(point_i)

    print check_wall_info
    print each_wall_info
    for check_wall_i in range(len(check_wall_info)):
        main_epsilon = 0.5
        sub_epsilon = 0.5
        while True:
            main_bbox = extend_bbox(side_line_info[check_wall_i], main_epsilon)
            for check_wall in check_wall_info[check_wall_i]:

                sub_bbox = extend_bbox(side_line_info[check_wall], sub_epsilon)
                # sub_bbox = bbox_list[check_wall]
                if check_wall not in check_bbox_index[check_wall_i]:
                    if check_bbox2(main_bbox, sub_bbox):
                        check_bbox_index[check_wall_i].append(check_wall)
                        check_bbox_index[check_wall].append(check_wall_i)


            # if len(each_wall_info[check_wall_i]) != 0:
            #     if len(each_wall_info[check_wall_i]) % 2 == 0:
            #         print main_epsilon, sub_epsilon
            #         break
            if main_epsilon > 0.5:
                break
            # if main_epslion > 0.3:
            #     print check_bbox_i, main_epslion
            #     break

            main_epsilon = main_epsilon + 0.005
            sub_epsilon = sub_epsilon + 0.005

    delete_node = []



    for each_i in range(len(check_bbox_index)):
        if len(check_bbox_index[each_i]) == 1:
            delete_node.append([each_i, check_bbox_index[each_i][0]])
    print delete_node
    if len(delete_node) != 0:
        for delete_v in delete_node:
            temp_i = check_bbox_index[delete_v[1]].index(delete_v[0])

            check_bbox_index[delete_v[1]].pop(temp_i)


    x = Symbol('x')
    y = Symbol('y')
    z = 0.0
    graph_list = []
    for check_value in check_bbox_index:
        graph_list.append(check_value)

    for main_i in range(len(check_bbox_index)):
        for match_i in check_bbox_index[main_i]:

            if len(check_bbox_index[match_i]) > 1 and len(check_bbox_index[main_i]) > 1:
                if main_i < match_i:
                    temp_list = np.cross(
                        [normal_vector[main_i][0], normal_vector[main_i][1], normal_vector[main_i][2]],
                        [normal_vector[match_i][0], normal_vector[match_i][1], normal_vector[match_i][2]])
                    e1 = Eq(
                        normal_vector[main_i][0] * x + normal_vector[main_i][1] * y + normal_vector[main_i][3],
                        0)
                    e2 = Eq(
                        normal_vector[match_i][0] * x + normal_vector[match_i][1] * y +
                        normal_vector[match_i][3],
                        0)

                    value_eq = solve([e1, e2], x, y)
                    temp_list = temp_list.tolist()
                    temp_list.append(value_eq[x])
                    temp_list.append(value_eq[y])
                    temp_list.append(z)


                    main_minZ = bbox_list[match_i][1][2]
                    main_maxZ = bbox_list[match_i][0][2]

                    minT = (main_minZ - temp_list[5]) / temp_list[2]
                    minX = (minT * temp_list[0]) + temp_list[3]
                    minY = (minT * temp_list[1]) + temp_list[4]

                    maxT = (main_maxZ - temp_list[5]) / temp_list[2]
                    maxX = (maxT * temp_list[0]) + temp_list[3]
                    maxY = (maxT * temp_list[1]) + temp_list[4]

                    main_point_bot = [minX, minY, main_minZ]
                    main_point_top = [maxX, maxY, main_maxZ]
                    main_points = [main_point_bot, main_point_top]
                    each_wall_info[main_i].append(main_points)
                    each_wall_info[match_i].append(main_points)



    print each_wall_info
    remove_index_list = []
    for i in range(len(each_wall_info)):
        if len(each_wall_info[i]) == 0:
            remove_index_list.append(i)

    if len(remove_index_list) != 0:
        remove_count = 0
        for i in remove_index_list:
            r_i = i - remove_count
            each_wall_info.pop(r_i)
            bbox_list.pop(r_i)
            normal_vector.pop(r_i)
            wall_point_list.pop(r_i)
            remove_count = remove_count + 1


    test_graph = MakingGraph2([each_wall_info])
    checked_list, G = test_graph.make_graph2()

    delete_value = check_point(checked_list, wall_point_list, normal_vector)

    test_graph.get_newGraph(G, delete_value)

    return each_wall_info

def check_point(checked_list, wall_point_list, normal_vector):
    delete_value = []
    print "hi"
    count = 0
    for i in checked_list:
        pointcloud_rate = get_poiontRate(wall_point_list[i[0]], normal_vector[i[0]], get_range(i[2]+i[3]))
        a = get_range(i[2] + i[3])
        check_a = []
        check_a.append(a[0])
        check_a.append(a[1])
        b = pcl.PointCloud()
        b.from_list(check_a)

        # pcl.save(b, "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/bbox2_" + str(count) + ".pcd")
        count = count + 1

        # result_rate = pointcloud_rate*i[1] *100 <= 20
        result_rate = pointcloud_rate*i[1] *100 <=0.01
        print pointcloud_rate == 0.0, pointcloud_rate, i[1], i[4]
        if pointcloud_rate == 0.0 or result_rate:

            delete_value.append(i[4])


    return delete_value

def get_poiontRate(pointcloud, normal_vector, bbox):

    points = pointcloud.to_list()
    pointcloud_size = pointcloud.size
    a, b = check_distance_plane(points, normal_vector, 0.1)
    count = 0
    temp_list = []
    for point in points:

        if check_point_range_e(point, bbox):
            temp_list.append(point)
            count = count + 1
    a, c = check_distance_plane(temp_list, normal_vector,0.1)
    # print "checked_count : ", float(count), float(pointcloud_size)
    # print b, c, float(c) / float(b)
    if c == 0:
        return 0.0
    else:

        return float(c) / float(b)


def get_intersection_line2(bbox_list, normal_vector, side_line_list):
    """Create intersection points between planes

        Search intersecting lines between planes where each bounding box intersects.
        Create intersection points using information from the intersection lines and Maximum and Minimum height value for each plane

    Args:
        bbox_list: Bounding box information of each plane
        normal_vector: Coefficient data list of each plane
        side_line_list: Side lines list of each plane
    Returns:
        side_line_list: side_line_list with each line intersection line information added
    """
    x = Symbol('x')
    y = Symbol('y')
    z = 0.0

    check_bbox_index = [[] for i in range(len(normal_vector))]

    for point_i in range(len(bbox_list) - 1):
        temp_bbox = list()
        for point_j in range(point_i + 1, len(bbox_list)):
            if check_bbox(bbox_list[point_i], bbox_list[point_j]):
                temp_bbox.append(point_j)
        check_bbox_index[point_i].extend(temp_bbox)

    for match_i in range(len(check_bbox_index)):
        if len(check_bbox_index[match_i]) != 0:
            for sub_index in check_bbox_index[match_i]:
                model_cos = np.dot(
                    [normal_vector[match_i][0], normal_vector[match_i][1], normal_vector[match_i][2]],
                    [normal_vector[sub_index][0], normal_vector[sub_index][1], normal_vector[sub_index][2]]) \
                            / (np.linalg.norm(
                    [normal_vector[match_i][0], normal_vector[match_i][1], normal_vector[match_i][2]])
                               * np.linalg.norm(
                            [normal_vector[sub_index][0], normal_vector[sub_index][1],
                             normal_vector[sub_index][2]]))
                if math.fabs(round(model_cos, 1)) != 1.0:
                    temp_list = np.cross(
                        [normal_vector[match_i][0], normal_vector[match_i][1], normal_vector[match_i][2]],
                        [normal_vector[sub_index][0], normal_vector[sub_index][1], normal_vector[sub_index][2]])
                    e1 = Eq(
                        normal_vector[match_i][0] * x + normal_vector[match_i][1] * y + normal_vector[match_i][3],
                        0)
                    e2 = Eq(
                        normal_vector[sub_index][0] * x + normal_vector[sub_index][1] * y +
                        normal_vector[sub_index][3],
                        0)

                    value_eq = solve([e1, e2], x, y)
                    temp_list = temp_list.tolist()
                    temp_list.append(value_eq[x])
                    temp_list.append(value_eq[y])
                    temp_list.append(z)

                    temp_point_list = list()

                    main_minZ = bbox_list[match_i][1][2]
                    main_maxZ = bbox_list[match_i][0][2]
                    sub_minZ = bbox_list[sub_index][1][2]
                    sub_maxZ = bbox_list[sub_index][0][2]

                    minT = (z - temp_list[5]) / temp_list[2]
                    minX = (minT * temp_list[0]) + temp_list[3]
                    minY = (minT * temp_list[1]) + temp_list[4]

                    main_point_bot = [minX, minY, main_minZ]
                    main_point_top = [minX, minY, main_maxZ]
                    sub_point_bot = [minX, minY, sub_minZ]
                    sub_point_top = [minX, minY, sub_maxZ]
                    main_points = [main_point_bot, main_point_top]
                    sub_points = [sub_point_bot, sub_point_top]
                    side_line_list[match_i].append(main_points)
                    side_line_list[sub_index].append(main_points)
                    # side_line_list[sub_index].append(sub_points)
                    # if included_bbox(bbox_list[match_i], bbox_list[sub_index]) == 2:
                    #     side_line_list[match_i].append(main_points)
                    #     side_line_list[match_i].append(sub_points)
                    #     side_line_list[sub_index].append(sub_points)
                    #
                    # elif included_bbox(bbox_list[match_i], bbox_list[sub_index]) == -2:
                    #     side_line_list[sub_index].append(sub_points)
                    #     side_line_list[sub_index].append(main_points)
                    #     side_line_list[match_i].append(main_points)
                    # else:
                    #     side_line_list[match_i].append(main_points)
                    #     side_line_list[sub_index].append(sub_points)

    return side_line_list


def get_intersection_line2(bbox_list, normal_vector, side_line_list):
    """Create intersection points between planes

        Search intersecting lines between planes where each bounding box intersects.
        Create intersection points using information from the intersection lines and Maximum and Minimum height value for each plane

    Args:
        bbox_list: Bounding box information of each plane
        normal_vector: Coefficient data list of each plane
        side_line_list: Side lines list of each plane
    Returns:
        side_line_list: side_line_list with each line intersection line information added
    """
    x = Symbol('x')
    y = Symbol('y')
    z = 0.0



    for main_i in range(len(normal_vector) - 1):
        for target_i in range(1, len(normal_vector)):
                model_cos = np.dot(
                    [normal_vector[main_i][0], normal_vector[main_i][1], normal_vector[main_i][2]],
                    [normal_vector[target_i][0], normal_vector[target_i][1], normal_vector[target_i][2]]) \
                            / (np.linalg.norm(
                    [normal_vector[main_i][0], normal_vector[main_i][1], normal_vector[main_i][2]])
                               * np.linalg.norm(
                            [normal_vector[target_i][0], normal_vector[target_i][1],
                             normal_vector[target_i][2]]))
                if math.fabs(round(model_cos, 1)) != 1.0:
                    temp_list = np.cross(
                        [normal_vector[main_i][0], normal_vector[main_i][1], normal_vector[main_i][2]],
                        [normal_vector[target_i][0], normal_vector[target_i][1], normal_vector[target_i][2]])
                    e1 = Eq(
                        normal_vector[main_i][0] * x + normal_vector[main_i][1] * y + normal_vector[main_i][3],
                        0)
                    e2 = Eq(
                        normal_vector[target_i][0] * x + normal_vector[target_i][1] * y +
                        normal_vector[target_i][3],
                        0)

                    value_eq = solve([e1, e2], x, y)
                    temp_list = temp_list.tolist()
                    temp_list.append(value_eq[x])
                    temp_list.append(value_eq[y])
                    temp_list.append(z)

                    temp_point_list = list()

                    main_minZ = bbox_list[main_i][1][2]
                    main_maxZ = bbox_list[main_i][0][2]
                    target_minZ = bbox_list[target_i][1][2]
                    target_maxZ = bbox_list[target_i][0][2]

                    minT = (z - temp_list[5]) / temp_list[2]
                    minX = (minT * temp_list[0]) + temp_list[3]
                    minY = (minT * temp_list[1]) + temp_list[4]

                    main_point_bot = [minX, minY, main_minZ]
                    main_point_top = [minX, minY, main_maxZ]
                    target_point_bot = [minX, minY, target_minZ]
                    target_point_top = [minX, minY, target_maxZ]
                    check_main = True
                    check_target = True
                    # if check_point_side(main_point_bot, side_line_list[main_i][0:2]) == False:
                    #     check_main = False
                    # if check_point_side(target_point_bot, side_line_list[target_i][0:2]) == False:
                    #     check_target = False
                    if check_point_range_2(main_point_bot, bbox_list[main_i]) == False:
                        check_main = False
                    if check_point_range_2(target_point_bot, bbox_list[target_i]) == False:
                        check_target = False

                    if check_main or check_target:

                        main_points = [main_point_bot, main_point_top]
                        target_points = [target_point_bot, target_point_top]
                        side_line_list[main_i].append(main_points)
                        side_line_list[target_i].append(target_points)
                    # if included_bbox(bbox_list[match_i], bbox_list[sub_index]) == 2:
                    #     side_line_list[match_i].append(main_points)
                    #     side_line_list[match_i].append(sub_points)
                    #     side_line_list[sub_index].append(sub_points)
                    #
                    # elif included_bbox(bbox_list[match_i], bbox_list[sub_index]) == -2:
                    #     side_line_list[sub_index].append(sub_points)
                    #     side_line_list[sub_index].append(main_points)
                    #     side_line_list[match_i].append(main_points)
                    # else:
                    #     side_line_list[match_i].append(main_points)
                    #     side_line_list[sub_index].append(sub_points)

    return side_line_list

def visual_graph(point_list):
    """Visualize point list as graph

    Args:
        point_list: list of Points

    Returns:
        Visualize point list as graph
    """
    x = []
    y = []
    # print len(e)
    for index in point_list:
        for i in index:
            x.append(i[0])
            y.append(i[2])
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
    """Calculate the distance between two points

    Args:
        point1: information of point (x, y, z)
        point2: information of point (x, y, z)

    Returns:
        d: the distance between two points
    """
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

def visual_viewer(cloud_list):
    """Visualizing Pointcloud data

    Args:
        cloud_list: list of PointClouds

    Returns:
        Visualizing Pointclouds data
    """
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)

    # viewer.AddPointCloud(cloud_value, b'sample cloud')
    # viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')
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
    """Visualizing Pointcloud data

       Args:
           cloud: Original PointCloud data
           cloud_list: list of PointClouds

       Returns:
           Visualizing PointClouds data
    """
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)

    viewer.AddPointCloud(cloud, b'sample cloud')
    viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')

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

def z_point_sorting(x, y):
    """Sorting tha data using z-value

    Args:
        x: Present point
        y: Next point

    Returns:
        -1: Present z-value of point is less than next z-value
        1: Present z-value of point is not less than next z-value
    """
    if x[2] <= y[2]:
        return -1
    else:
        return 1

def cmp_to_key(mycmp):
    """Convert a cmp= function into a key= function"""

    class K:
        def __init__(self, obj, *args):
            self.obj = obj

        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0

        def __gt__(self, other):
            return mycmp(self.obj, other.obj) >= 0

    return K
# def make_rectangle(rectangle_point):
#     side1_bottom = rectangle_point[0][0]
#     side1_top = rectangle_point[0][1]
#
#     side2_bottom = rectangle_point[1][0]
#     side2_top = rectangle_point[1][1]
#
#     line_top = (np.asarray(side1_top) - np.asarray(side2_top)).tolist() + side2_top
#     line_bottom = (np.asarray(side1_bottom) - np.asarray(side2_bottom)).tolist() + side2_bottom
#     # line_side1 = (np.asarray(side1_top) - np.asarray(side1_bottom)).tolist() + side1_bottom
#     # line_side2 = (np.asarray(side2_top) - np.asarray(side2_bottom)).tolist() + side2_bottom
#
#     return [line_top, line_bottom]
# def get_rectangle_points2(wall_info):
#     side_1 = wall_info[0]
#     side_2 = wall_info[len(wall_info) - 1]
#
#     if len(wall_info) == 3:
#         d_value_1 = np.asarray(side_1[0]) - np.asarray(wall_info[1][0])
#         # d_1 = math.fabs(d_value_1[0]) + math.fabs(d_value_1[1])
#         d_1 = math.sqrt(math.pow(d_value_1[0], 2) + math.pow(d_value_1[1], 2))
#
#         d_value_2 = np.asarray(side_2[0]) - np.asarray(wall_info[1][0])
#         # d_2 = math.fabs(d_value_2[0]) + math.fabs(d_value_2[1])
#         d_2 = math.sqrt(math.pow(d_value_2[0], 2) + math.pow(d_value_2[1], 2))
#         # print d_1, d_2
#         if d_1 > d_2:
#             return side_1 + wall_info[1]
#         elif d_1 < d_2:
#             return side_2 + wall_info[1]
#         else:
#             return side_1 + side_2
#     else:
#         distance_list = list()
#         index_list = list()
#         for value in range(1, len(wall_info) - 1):
#             d_value = np.asarray(side_1[0]) - np.asarray(wall_info[value][0])
#             # d = math.fabs(d_value[0]) + math.fabs(d_value[1])
#             d = math.sqrt(math.pow(d_value[0], 2) + math.pow(d_value[1], 2))
#             if d_value[2] != 0:
#                 print "The Z value is not same!!!! %s" % str(d_value[2])
#             distance_list.append(d)
#             index_list.append(value)
#
#         center_point = get_center_point(side_1 + side_2)
#         center_point -= side_1[0]
#         # to_center_d1 = math.fabs(center_point[0]) + math.fabs(center_point[1])
#         to_center_d1 = math.sqrt(math.pow(center_point[0], 2) + math.pow(center_point[1], 2))
#
#         max_d = np.amax(np.asarray(distance_list))
#         min_d = np.amin(np.asarray(distance_list))
#
#         min_i = np.argmin(np.asarray(distance_list))
#         max_i = np.argmax(np.asarray(distance_list))
#
#         index_2 = index_list[int(max_i)]
#         index_1 = index_list[int(min_i)]
#
#         if math.fabs(min_d - max_d) <= 0.5:
#             if index_2 != index_1:
#                 if max_d < to_center_d1:
#                     return wall_info[index_1] + side_2
#                 elif min_d > to_center_d1:
#                     return wall_info[index_2] + side_1
#                 else:
#                     return wall_info[index_1] + wall_info[index_2]
#             else:
#                 return side_1 + side_2
#         else:
#             return wall_info[index_1] + wall_info[index_2]
#         if max_d < to_center_d1:
#             return wall_info[index_1] + side_2
#         elif min_d > to_center_d1:
#             return wall_info[index_2] + side_1
#         else:
#             if index_1 == index_2:
#                 return side_1 + side_2
#             else:
#                 return wall_info[index_1] + wall_info[index_2]
# def get_center_point(side_points):
#     center_point = (reduce(lambda x, y: np.array(x) + np.array(y), side_points)) / float(len(side_points))
#     min_z = side_points[0][2]
#
#     plane_abc = np.cross(np.asarray(side_points[1]) - np.asarray(side_points[0]),
#                              np.asarray(side_points[2]) - np.asarray(side_points[0]))
#     plane_d = np.asarray([np.sum(plane_abc * side_points[0]) * -1.0])
#     a, b, c, d = np.concatenate([plane_abc, plane_d]).tolist()
#     min_y = (a * center_point[0] + c * min_z + d) / b * (-1.0)
#     center_xyz = np.asarray([center_point[0], min_y, min_z])
#
#     return center_xyz
#
# def get_rectangle_points(wall_info):
#     side_1 = wall_info[0]
#     side_2 = wall_info[len(wall_info) - 1]
#
#
#     if len(wall_info) == 3:
#         d_value_1 = np.asarray(side_1[0]) - np.asarray(wall_info[1][0])
#         # d_1 = math.fabs(d_value_1[0]) + math.fabs(d_value_1[1])
#         d_1 = math.sqrt(math.pow(d_value_1[0], 2) + math.pow(d_value_1[1], 2))
#
#         d_value_2 = np.asarray(side_2[0]) - np.asarray(wall_info[1][0])
#         # d_2 = math.fabs(d_value_2[0]) + math.fabs(d_value_2[1])
#         d_2 = math.sqrt(math.pow(d_value_2[0], 2) + math.pow(d_value_2[1], 2))
#         # print d_1, d_2
#         if d_1 > d_2:
#             return side_1 + wall_info[1]
#         elif d_1 < d_2:
#             return side_2 + wall_info[1]
#         else:
#             return side_1 + side_2
#     else:
#         distance_list = list()
#         index_list = list()
#         for value in range(1, len(wall_info) - 1):
#             d_value = np.asarray(side_1[0]) - np.asarray(wall_info[value][0])
#             # d = math.fabs(d_value[0]) + math.fabs(d_value[1])
#             d = math.sqrt(math.pow(d_value[0], 2) + math.pow(d_value[1], 2))
#             if d_value[2] != 0:
#                 print "The Z value is not same!!!! %s" % str(d_value[2])
#             distance_list.append(d)
#             index_list.append(value)
#
#         center_point = get_center_point(side_1 + side_2)
#         center_point -= side_1[0]
#         # to_center_d1 = math.fabs(center_point[0]) + math.fabs(center_point[1])
#         to_center_d1 = math.sqrt(math.pow(center_point[0], 2) + math.pow(center_point[1], 2))
#
#         max_d = np.amax(np.asarray(distance_list))
#         min_d = np.amin(np.asarray(distance_list))
#
#         min_i = np.argmin(np.asarray(distance_list))
#         max_i = np.argmax(np.asarray(distance_list))
#
#         index_2 = index_list[int(max_i)]
#         index_1 = index_list[int(min_i)]
#
#         if math.fabs(min_d - max_d) <= 0.5:
#             if index_2 != index_1:
#                 if max_d < to_center_d1:
#                     return wall_info[index_1] + side_2
#                 elif min_d > to_center_d1:
#                     return wall_info[index_2] + side_1
#                 else:
#                     return wall_info[index_1] + wall_info[index_2]
#             else:
#                 return side_1 + side_2
#         else:
#             return wall_info[index_1] + wall_info[index_2]
#         # if max_d < to_center_d1:
#         #     return wall_info[index_1] + side_2
#         # elif min_d > to_center_d1:
#         #     return wall_info[index_2] + side_1
#         # else:
#         #     if index_1 == index_2:
#         #         return side_1 + side_2
#         #     else:
#         #         return wall_info[index_1] + wall_info[index_2]


# def get_point_from_line(line_info):
#
#     point_set = list()
#     max_z = line_info[6]
#     min_z = line_info[7]
#
#     temp_surface_info = list()
#     max_z1 = max_z
#     max_t = (max_z1 - line_info[5]) / line_info[2]
#     max_x1 = max_t * line_info[0] + line_info[3]
#     max_y1 = max_t * line_info[1] + line_info[4]
#
#     temp_surface_info.append(float(max_x1))
#     temp_surface_info.append(float(max_y1))
#     temp_surface_info.append(float(max_z1))
#     point_set.append(temp_surface_info)
#     temp_surface_info = list()
#     min_z1 = min_z
#     min_t = (min_z1 - line_info[5]) / line_info[2]
#     min_x1 = min_t * line_info[0] + line_info[3]
#     min_y1 = min_t * line_info[1] + line_info[4]
#
#     temp_surface_info.append(float(min_x1))
#     temp_surface_info.append(float(min_y1))
#     temp_surface_info.append(float(min_z1))
#     point_set.append(temp_surface_info)
#
#     return point_set




if __name__ == "__main__":

    start_vect = time.time()
    # cloud = pcl.load("/home/dprt/Desktop/Untitled Folder 3/Untitled Folder 3/Untitled Folder 2/cloud_cluster_9459.pcd")
    # cloud = pcl.load("/home/dprt/Desktop/Demo/IOU_test/Test_part_area_wall.ply")
    # cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/SC/20.ply")
    path = "/home/dprt/Desktop/Sigspatial_LBJ/booth1.ply"
    file_name = (path.split('/')[-1]).split('.')[0]
    # cloud = pcl.load("/home/dprt/Desktop/pinsout_gitlab/pinsout/data/Sigspatial_LBJ/booth7.ply")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/Area_2/Area_2_All_office_wall_seg.ply")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/3d-model_wall_2.ply")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/3d-model_18-cloud_wall_.pcd")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/3d-model_wall_01-remove-3.ply")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/npy_data2/dump/sampling_in_d_wall_.pcd")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/sampling_in_d_wall_.ply")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/sampling_in_d_wall_.pcd")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/npy_data2/dump/test_pointcloud2_wall_.pcd")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/3d-model_wall_01-remove-3.ply")

    # cloud = pcl.load("/home/dprt/Desktop/lbj/merge2.ply")
    # cloud = pcl.load("/home/dprt/Desktop/lbj/npy_data2/dump/merge_subsample_nofloor_wall (copy).ply")
    # cloud = pcl.load("/home/dprt/Desktop/21/Untitled Folder 2/colorizedlast_wall2.ply")
    # cloud = pcl.load("/home/dprt/Desktop/Demo/test/sc_test_data_Booth_10_ceiling.ply")

    # cloud = pcl.load("/home/dprt/Desktop/Demo/IOU_test/Untitled Folder/colorized_wall.ply")
    # a = clustering(cloud)
    # print cloud.get_point(0)
    #
    #
    # cloud = pcl.load(
    #     "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/sampling_in_d_wall_.pcd")
    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_286/npy_data2/dump/sampling_in_d_wall_.pcd")
    cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_429/npy_data2/dump/sampling_in_d_wall_.pcd")
    a = make_wall_info(cloud)
    first_process_runtime = (time.time() - start_vect) / 60
    print("plane RANSAC Process Runtime: %0.2f Minutes" % (first_process_runtime))

    # [array([-0.21555801, -1.13197005, 2.95484996]), array([-0.21555804, -3.01844001, 0.0867425])]

    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/1000_143/npy_data2/dump/f2_13.pcd")
    # print cloud.make_cropbox()
    # a = [[-0.2155580073595047, -3.018440008163452, 2.9548499584198],
    #  [-0.2155580073595047, -3.018440008163452, 0.08674249798059464],
    #
    #  [-0.2155580073595047, -1.1319700479507446, 2.9548499584198],
    #  [-0.2155580073595047, -1.1319700479507446, 0.08674249798059464],
    #
    #  [-0.2155580371618271, -1.1319700479507446, 2.9548499584198],
    #  [-0.2155580371618271, -1.1319700479507446, 0.08674249798059464],
    #
    #  [-0.2155580371618271, -3.018440008163452, 2.9548499584198],
    #  [-0.2155580371618271, -3.018440008163452, 0.08674249798059464],
    #
    #  [-0.2155580073595047, -3.018440008163452, 2.9548499584198],
    #  [-0.2155580073595047, -3.018440008163452, 0.08674249798059464]]
    #
    # b = [[-0.215558007359505, -2.59912530755782, 0.08674249798059464],
    #      [-0.215558007359505, -1.78219608420728, 2.9548499584198],
    #
    #      [-0.215567739378466, -1.13197004795074, 0.08674249798059464],
    #      [-0.215562320476383, -1.13197004795074, 2.9548499584198],
    #
    #      [-0.215558037161827, -2.59463244392920, 0.08674249798059464],
    #      [-0.215558037161827, -1.77770322057866, 2.9548499584198],
    #
    #      [-0.215555225937027, -3.01844000816345, 0.08674249798059464],
    #      [-0.215549807034945, -3.01844000816345, 2.9548499584198]]
    # bb = [[-0.21555801, -1.13197005, 2.95484996],[-0.21555804, -3.01844001, 0.0867425]]
    # for b_i in b:
    #     print check_point_range_e(b_i, bb, 0.000001)
    # n_b = ps2.CalculateCentroid(b)
    # c = []
    # c.append(a)
    # c.append(n_b)
    #
    #
    # d = ps.Point_sort()
    # d.visual_graph3(c)

    # test_bbox = [[-0.0232066, -0.92826444, 2.98539662], [-0.0610235, -2.97115016, 0.11777067]]
    # m_box = box(test_bbox[1][0], test_bbox[1][1], test_bbox[0][0], test_bbox[0][1])
    # print m_box.exterior.coords[0][0]

        # print(p.intersects(q))  # True
    # print(p.intersection(q).area)  # 1.0
    # x = p.intersection(q)
    # print(x)

    # poly2obj = po.Ply2Obj(a)
    # poly2obj.poly_2_obj('All')
    # # print file_name
    # # dae_filename = '/home/dprt/Desktop/pinsout_gitlab/pinsout/data/sc_demo/'+file_name+'.dae'
    #
    # dae_filename = '/home/dprt/Desktop/pinsout_gitlab/pinsout/data/sc_demo/booth7.dae'
    # poly2obj.output_dae(dae_filename)
    # 0.05
    # rooms = [[[[4.69711197662828, 3.10132797859425, 0.01646059937775135], [4.69711197662828, 3.10132797859425, 3.0554399490356445]], [[-4.69822532075788, 3.12000233212000, 0.01646059937775135], [-4.69822532075788, 3.12000233212000, 3.0554399490356445]], [[2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 3.0554399490356445]], [[2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 3.0554399490356445]], [[1.03671818059138, 3.10860344786449, 0.01646059937775135], [1.03671818059138, 3.10860344786449, 3.0554399490356445]], [[1.17232344394907, 3.10833391622898, 0.01646059937775135], [1.17232344394907, 3.10833391622898, 3.0554399490356445]]], [[[4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 3.0621800422668457]], [[-4.69794659232765, -3.11232705634018, 0.03738019987940788], [-4.69794659232765, -3.11232705634018, 3.0621800422668457]], [[-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457]], [[-0.215555162439024, -3.11223542275595, 0.03738019987940788], [-0.215555162439024, -3.11223542275595, 3.0621800422668457]]], [[[4.69711197662828, 3.10132797859425, 0.01646059937775135], [4.69711197662828, 3.10132797859425, 3.0554399490356445]], [[4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 3.0621800422668457]]], [[[-4.69822532075788, 3.12000233212000, 0.01646059937775135], [-4.69822532075788, 3.12000233212000, 3.0554399490356445]], [[-4.69794659232765, -3.11232705634018, 0.03738019987940788], [-4.69794659232765, -3.11232705634018, 3.0621800422668457]], [[-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-4.69805035458894, -0.792217156192569, 3.0480549335479736]], [[-4.69804342456960, -0.947171436056617, 0.036078501492738724], [-4.69804342456960, -0.947171436056617, 3.0480549335479736]]], [[[-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-4.69805035458894, -0.792217156192569, 3.0480549335479736]], [[-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186]]], [[[2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 3.0554399490356445]], [[2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.98952686704443, -0.790252950611448, 3.038594961166382]], [[2.98956085812258, -0.658403856221184, 0.024747176095843315], [2.98956085812258, -0.658403856221184, 3.038594961166382]]], [[[2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 3.0554399490356445]], [[2.85417986314639, -0.790574479280031, 0.03525486961007118], [2.85417986314639, -0.790574479280031, 3.0363399982452393]], [[2.85423922802842, -0.658409047928616, 0.03525486961007118], [2.85423922802842, -0.658409047928616, 3.0363399982452393]]], [[[1.03671818059138, 3.10860344786449, 0.01646059937775135], [1.03671818059138, 3.10860344786449, 3.0554399490356445]], [[1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03899695814266, -0.794886605340243, 3.0378000736236572]], [[1.03891732599760, -0.658478694002224, 0.033170800656080246], [1.03891732599760, -0.658478694002224, 3.0378000736236572]]], [[[2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.98952686704443, -0.790252950611448, 3.038594961166382]], [[2.85417986314639, -0.790574479280031, 0.03525486961007118], [2.85417986314639, -0.790574479280031, 3.0363399982452393]], [[1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03899695814266, -0.794886605340243, 3.0378000736236572]], [[1.17151383917327, -0.794571799876543, 0.020686199888586998], [1.17151383917327, -0.794571799876543, 3.0344998836517334]]], [[[1.17232344394907, 3.10833391622898, 0.01646059937775135], [1.17232344394907, 3.10833391622898, 3.0554399490356445]], [[1.17151383917327, -0.794571799876543, 0.020686199888586998], [1.17151383917327, -0.794571799876543, 3.0344998836517334]], [[1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17154207089495, -0.658473605762648, 3.047840118408203]]], [[[2.98956085812258, -0.658403856221184, 0.024747176095843315], [2.98956085812258, -0.658403856221184, 3.038594961166382]], [[2.85423922802842, -0.658409047928616, 0.03525486961007118], [2.85423922802842, -0.658409047928616, 3.0363399982452393]], [[1.03891732599760, -0.658478694002224, 0.033170800656080246], [1.03891732599760, -0.658478694002224, 3.0378000736236572]], [[1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17154207089495, -0.658473605762648, 3.047840118408203]]], [[[-4.69804342456960, -0.947171436056617, 0.036078501492738724], [-4.69804342456960, -0.947171436056617, 3.0480549335479736]], [[-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982]], [[-0.215571228170187, -0.948702039242467, 0.02952166646718979], [-0.215571228170187, -0.948702039242467, 3.0423800945281982]]], [[[-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457]], [[-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186]], [[-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982]]], [[[-0.215555162439024, -3.11223542275595, 0.03738019987940788], [-0.215555162439024, -3.11223542275595, 3.0621800422668457]], [[-0.215571228170187, -0.948702039242467, 0.02952166646718979], [-0.215571228170187, -0.948702039242467, 3.0423800945281982]]]]
    #
    # print(len(rooms))
    #
    # for i in range(len(rooms)):
    #     list_a = []
    #     l = []
    #     for j in range(len(rooms[i])):
    #
    #         l.extend(rooms[i][j])
    #
    #     testa = pcl.PointCloud()
    #     testa.from_list(l)
    #     list_a.append(testa)
    #
    #     pcl.save(testa, "/home/dprt/Documents/dprt/pointnet_data/Untitled Folder/test"+str(i)+".ply")
        # visual_viewer2(cloud, list_a)

# /usr/local/cuda/lib64:$LD_LIBRARY_PATH



# 0
# side_line_points :  [[-4.55186843872070, 3.10371001810592, 0.030340079218149185], [-4.55186843872070, 3.11225950992093, 3.0321598052978516], [4.45410442352295, 3.10127483346018, 0.030340079218149185], [4.45410442352295, 3.10982432527518, 3.0321598052978516]]
# [[4.454104423522949, 3.0548300743103027, 3.0321598052978516], [4.454104423522949, 3.0548300743103027, 0.030340079218149185], [4.454104423522949, 3.113368511199951, 3.0321598052978516], [4.454104423522949, 3.113368511199951, 0.030340079218149185], [-4.551868438720703, 3.113368511199951, 3.0321598052978516], [-4.551868438720703, 3.113368511199951, 0.030340079218149185], [-4.551868438720703, 3.0548300743103027, 3.0321598052978516], [-4.551868438720703, 3.0548300743103027, 0.030340079218149185], [4.454104423522949, 3.0548300743103027, 3.0321598052978516], [4.454104423522949, 3.0548300743103027, 0.030340079218149185]]
# [[4.45410442352295, 3.10127483346018, 0.030340079218149185], [4.45410442352295, 3.10982432527518, 3.0321598052978516], [-40.2715937774248, 3.11336851119995, 0.030340079218149185], [-8.65325575673064, 3.11336851119995, 3.0321598052978516], [-4.55186843872070, 3.10371001810592, 0.030340079218149185], [-4.55186843872070, 3.11225950992093, 3.0321598052978516], [176.219412334848, 3.05483007431030, 0.030340079218149185], [207.837750355542, 3.05483007431030, 3.0321598052978516]]
# 1
# side_line_points :  [[-4.56954145431519, -3.11231038443798, 0.0582517646253109], [-4.56954145431519, -3.11224168972453, 3.029421091079712], [4.58844995498657, -3.11214909809773, 0.0582517646253109], [4.58844995498657, -3.11208040338429, 3.029421091079712]]
# [[4.588449954986572, -3.1218299865722656, 3.029421091079712], [4.588449954986572, -3.1218299865722656, 0.0582517646253109], [4.588449954986572, -3.066685438156128, 3.029421091079712], [4.588449954986572, -3.066685438156128, 0.0582517646253109], [-4.5695414543151855, -3.066685438156128, 3.029421091079712], [-4.5695414543151855, -3.066685438156128, 0.0582517646253109], [-4.5695414543151855, -3.1218299865722656, 3.029421091079712], [-4.5695414543151855, -3.1218299865722656, 0.0582517646253109], [4.588449954986572, -3.1218299865722656, 3.029421091079712], [4.588449954986572, -3.1218299865722656, 0.0582517646253109]]
# [[4.58844995498657, -3.11214909809773, 0.0582517646253109], [4.58844995498657, -3.11208040338429, 3.029421091079712], [2586.05819220303, -3.06668543815613, 0.0582517646253109], [2582.15764120381, -3.06668543815613, 3.029421091079712], [-4.56954145431519, -3.11231038443798, 0.0582517646253109], [-4.56954145431519, -3.11224168972453, 3.029421091079712], [-545.101581757990, -3.12182998657227, 0.0582517646253109], [-549.002132757210, -3.12182998657227, 3.029421091079712]]
# 2
# side_line_points :  [[4.69853812361037, -2.96558308601379, 0.1383921056985855], [4.69897316616935, -2.96558308601379, 2.9708728790283203], [4.69854666254270, 2.96572995185852, 0.1383921056985855], [4.69898170510169, 2.96572995185852, 2.9708728790283203]]
# [[4.698821544647217, -2.965583086013794, 2.9708728790283203], [4.698821544647217, -2.965583086013794, 0.1383921056985855], [4.698821544647217, 2.9657299518585205, 2.9708728790283203], [4.698821544647217, 2.9657299518585205, 0.1383921056985855], [4.698819637298584, 2.9657299518585205, 2.9708728790283203], [4.698819637298584, 2.9657299518585205, 0.1383921056985855], [4.698819637298584, -2.965583086013794, 2.9708728790283203], [4.698819637298584, -2.965583086013794, 0.1383921056985855], [4.698821544647217, -2.965583086013794, 2.9708728790283203], [4.698821544647217, -2.965583086013794, 0.1383921056985855]]
# [[4.69882154464722, 193.904332896191, 0.1383921056985855], [4.69882154464722, -108.284922312841, 2.9708728790283203], [4.69854666254270, 2.96572995185852, 0.1383921056985855], [4.69898170510169, 2.96572995185852, 2.9708728790283203], [4.69881963729858, 192.579450419939, 0.1383921056985855], [4.69881963729858, -109.609804789093, 2.9708728790283203], [4.69853812361037, -2.96558308601379, 0.1383921056985855], [4.69897316616935, -2.96558308601379, 2.9708728790283203]]
# 3
# side_line_points :  [[-4.69444554124203, -3.06785631179810, 0.1118827760219574], [-4.70041241561247, -3.06785631179810, 2.9761099815368652], [-4.69465201290213, 2.96834897994995, 0.1118827760219574], [-4.70061888727258, 2.96834897994995, 2.9761099815368652]]
# [[-4.649772644042969, -3.0678563117980957, 2.9761099815368652], [-4.649772644042969, -3.0678563117980957, 0.1118827760219574], [-4.649772644042969, 2.968348979949951, 2.9761099815368652], [-4.649772644042969, 2.968348979949951, 0.1118827760219574], [-4.698821544647217, 2.968348979949951, 2.9761099815368652], [-4.698821544647217, 2.968348979949951, 0.1118827760219574], [-4.698821544647217, -3.0678563117980957, 2.9761099815368652], [-4.698821544647217, -3.0678563117980957, 0.1118827760219574], [-4.649772644042969, -3.0678563117980957, 2.9761099815368652], [-4.649772644042969, -3.0678563117980957, 0.1118827760219574]]
# [[-4.64977264404297, -1309.08137084437, 0.1118827760219574], [-4.64977264404297, -1483.52312537373, 2.9761099815368652], [-4.69465201290213, 2.96834897994995, 0.1118827760219574], [-4.70061888727258, 2.96834897994995, 2.9761099815368652], [-4.69882154464722, 124.864736941660, 0.1118827760219574], [-4.69882154464722, -49.5770175877045, 2.9761099815368652], [-4.69444554124203, -3.06785631179810, 0.1118827760219574], [-4.70041241561247, -3.06785631179810, 2.9761099815368652]]
# 4
# side_line_points :  [[-4.56674480438232, -0.793317828750477, 0.0681283250451088], [-4.56674480438232, -0.793304224477492, 3.0405960083007812], [-0.0710121989250183, -0.793330632196779, 0.0681283250451088], [-0.0710121989250183, -0.793317027923794, 3.0405960083007812]]
# [[-0.07101219892501831, -0.8165006041526794, 3.0405960083007812], [-0.07101219892501831, -0.8165006041526794, 0.0681283250451088], [-0.07101219892501831, -0.785601794719696, 3.0405960083007812], [-0.07101219892501831, -0.785601794719696, 0.0681283250451088], [-4.566744804382324, -0.785601794719696, 3.0405960083007812], [-4.566744804382324, -0.785601794719696, 0.0681283250451088], [-4.566744804382324, -0.8165006041526794, 3.0405960083007812], [-4.566744804382324, -0.8165006041526794, 0.0681283250451088], [-0.07101219892501831, -0.8165006041526794, 3.0405960083007812], [-0.07101219892501831, -0.8165006041526794, 0.0681283250451088]]
# [[-0.0710121989250183, -0.793330632196779, 0.0681283250451088], [-0.0710121989250183, -0.793317027923794, 3.0405960083007812], [-2713.93303254046, -0.785601794719696, 0.0681283250451088], [-2709.15610200239, -0.785601794719696, 3.0405960083007812], [-4.56674480438232, -0.793317828750477, 0.0681283250451088], [-4.56674480438232, -0.793304224477492, 3.0405960083007812], [8135.70711613530, -0.816500604152679, 0.0681283250451088], [8140.48404667338, -0.816500604152679, 3.0405960083007812]]
# 5
# side_line_points :  [[2.99020722922247, -0.558140397071838, 0.12413974851369858], [2.99015895552621, -0.558140397071838, 2.967020034790039], [2.99017418939464, 2.96020555496216, 0.12413974851369858], [2.99012591569838, 2.96020555496216, 2.967020034790039]]
# [[2.9901602268218994, -0.5581403970718384, 2.967020034790039], [2.9901602268218994, -0.5581403970718384, 0.12413974851369858], [2.9901602268218994, 2.960205554962158, 2.967020034790039], [2.9901602268218994, 2.960205554962158, 0.12413974851369858], [2.990159511566162, 2.960205554962158, 2.967020034790039], [2.990159511566162, 2.960205554962158, 0.12413974851369858], [2.990159511566162, -0.5581403970718384, 2.967020034790039], [2.990159511566162, -0.5581403970718384, 0.12413974851369858], [2.9901602268218994, -0.5581403970718384, 2.967020034790039], [2.9901602268218994, -0.5581403970718384, 0.12413974851369858]]
# [[2.99016022682190, 4.44705232477931, 0.12413974851369858], [2.99016022682190, -0.693518161463317, 2.967020034790039], [2.99017418939464, 2.96020555496216, 0.12413974851369858], [2.99012591569838, 2.96020555496216, 2.967020034790039], [2.99015951156616, 4.52321849430619, 0.12413974851369858], [2.99015951156616, -0.617351991936430, 2.967020034790039], [2.99020722922247, -0.558140397071838, 0.12413974851369858], [2.99015895552621, -0.558140397071838, 2.967020034790039]]
# 6
# side_line_points :  [[2.85547109877646, -0.568999826908112, 0.12187949568033218], [2.85534718128652, -0.568999826908112, 2.9727442264556885], [2.85541008827689, 2.97615242004395, 0.12187949568033218], [2.85528617078695, 2.97615242004395, 2.9727442264556885]]
# [[2.855360746383667, -0.5689998269081116, 2.9727442264556885], [2.855360746383667, -0.5689998269081116, 0.12187949568033218], [2.855360746383667, 2.9761524200439453, 2.9727442264556885], [2.855360746383667, 2.9761524200439453, 0.12187949568033218], [2.8553600311279297, 2.9761524200439453, 2.9727442264556885], [2.8553600311279297, 2.9761524200439453, 0.12187949568033218], [2.8553600311279297, -0.5689998269081116, 2.9727442264556885], [2.8553600311279297, -0.5689998269081116, 0.12187949568033218], [2.855360746383667, -0.5689998269081116, 2.9727442264556885], [2.855360746383667, -0.5689998269081116, 0.12187949568033218]]
# [[2.85536074638367, 5.84327406092986, 0.12187949568033218], [2.85536074638367, -1.35723029470416, 2.9727442264556885], [2.85541008827689, 2.97615242004395, 0.12187949568033218], [2.85528617078695, 2.97615242004395, 2.9727442264556885], [2.85536003112793, 5.88483560364540, 0.12187949568033218], [2.85536003112793, -1.31566875198861, 2.9727442264556885], [2.85547109877646, -0.568999826908112, 0.12187949568033218], [2.85534718128652, -0.568999826908112, 2.9727442264556885]]
# 7
# side_line_points :  [[1.14024305343628, -0.793370035418793, 0.0672641471028328], [1.14024305343628, -0.794112878026645, 3.02361798286438], [2.96065473556519, -0.792087753350331, 0.0672641471028328], [2.96065473556519, -0.792830595958183, 3.02361798286438]]
# [[2.9606547355651855, -0.7933061122894287, 3.02361798286438], [2.9606547355651855, -0.7933061122894287, 0.0672641471028328], [2.9606547355651855, -0.7641681432723999, 3.02361798286438], [2.9606547355651855, -0.7641681432723999, 0.0672641471028328], [1.1402430534362793, -0.7641681432723999, 3.02361798286438], [1.1402430534362793, -0.7641681432723999, 0.0672641471028328], [1.1402430534362793, -0.7933061122894287, 3.02361798286438], [1.1402430534362793, -0.7933061122894287, 0.0672641471028328], [2.9606547355651855, -0.7933061122894287, 3.02361798286438], [2.9606547355651855, -0.7933061122894287, 0.0672641471028328]]
# [[2.96065473556519, -0.792087753350331, 0.0672641471028328], [2.96065473556519, -0.792830595958183, 3.02361798286438], [42.5971634230069, -0.764168143272400, 0.0672641471028328], [43.6517514848513, -0.764168143272400, 3.02361798286438], [1.14024305343628, -0.793370035418793, 0.0672641471028328], [1.14024305343628, -0.794112878026645, 3.02361798286438], [1.23099251824980, -0.793306112289429, 0.0672641471028328], [2.28558058009413, -0.793306112289429, 3.02361798286438]]
# 8
# side_line_points :  [[1.17214285470574, -0.351843476295471, 0.08374449610710144], [1.17218525859554, -0.351843476295471, 2.9593400955200195], [1.17219414244284, 2.98012161254883, 0.08374449610710144], [1.17223654633264, 2.98012161254883, 2.9593400955200195]]
# [[1.172200083732605, -0.3518434762954712, 2.9593400955200195], [1.172200083732605, -0.3518434762954712, 0.08374449610710144], [1.172200083732605, 2.980121612548828, 2.9593400955200195], [1.172200083732605, 2.980121612548828, 0.08374449610710144], [1.1721998453140259, 2.980121612548828, 2.9593400955200195], [1.1721998453140259, 2.980121612548828, 0.08374449610710144], [1.1721998453140259, -0.3518434762954712, 2.9593400955200195], [1.1721998453140259, -0.3518434762954712, 0.08374449610710144], [1.172200083732605, -0.3518434762954712, 2.9593400955200195], [1.172200083732605, -0.3518434762954712, 0.08374449610710144]]
# [[1.17220008373260, 3.36610413408401, 0.08374449610710144], [1.17220008373260, 0.611288101350367, 2.9593400955200195], [1.17219414244284, 2.98012161254883, 0.08374449610710144], [1.17223654633264, 2.98012161254883, 2.9593400955200195], [1.17219984531403, 3.35061500495868, 0.08374449610710144], [1.17219984531403, 0.595798972225040, 2.9593400955200195], [1.17214285470574, -0.351843476295471, 0.08374449610710144], [1.17218525859554, -0.351843476295471, 2.9593400955200195]]
# 9
# side_line_points :  [[1.03750890423369, -0.463309198617935, 0.07293400168418884], [1.03742335218583, -0.463309198617935, 2.9615142345428467], [1.03741465103389, 2.97259354591370, 0.07293400168418884], [1.03732909898602, 2.97259354591370, 2.9615142345428467]]
# [[1.0374001264572144, -0.4633091986179352, 2.9615142345428467], [1.0374001264572144, -0.4633091986179352, 0.07293400168418884], [1.0374001264572144, 2.9725935459136963, 2.9615142345428467], [1.0374001264572144, 2.9725935459136963, 0.07293400168418884], [1.0374000072479248, 2.9725935459136963, 2.9615142345428467], [1.0374000072479248, 2.9725935459136963, 0.07293400168418884], [1.0374000072479248, -0.4633091986179352, 2.9615142345428467], [1.0374000072479248, -0.4633091986179352, 0.07293400168418884], [1.0374001264572144, -0.4633091986179352, 2.9615142345428467], [1.0374001264572144, -0.4633091986179352, 0.07293400168418884]]
# [[1.03740012645721, 3.50207193973399, 0.07293400168418884], [1.03740012645721, 0.383360674180033, 2.9615142345428467], [1.03741465103389, 2.97259354591370, 0.07293400168418884], [1.03732909898602, 2.97259354591370, 2.9615142345428467], [1.03740000724792, 3.50641759087265, 0.07293400168418884], [1.03740000724792, 0.387706325318682, 2.9615142345428467], [1.03750890423369, -0.463309198617935, 0.07293400168418884], [1.03742335218583, -0.463309198617935, 2.9615142345428467]]
# 10
# side_line_points :  [[1.18335831165314, -0.658466502295604, 0.11546099931001663], [1.18335831165314, -0.658483422335079, 3.029949903488159], [2.79306006431580, -0.658523459749002, 0.11546099931001663], [2.79306006431580, -0.658540379788477, 3.029949903488159]]
# [[2.793060064315796, -0.6585110425949097, 3.029949903488159], [2.793060064315796, -0.6585110425949097, 0.11546099931001663], [2.793060064315796, -0.6571527719497681, 3.029949903488159], [2.793060064315796, -0.6571527719497681, 0.11546099931001663], [1.1833583116531372, -0.6571527719497681, 3.029949903488159], [1.1833583116531372, -0.6571527719497681, 0.11546099931001663], [1.1833583116531372, -0.6585110425949097, 3.029949903488159], [1.1833583116531372, -0.6585110425949097, 0.11546099931001663], [2.793060064315796, -0.6585110425949097, 3.029949903488159], [2.793060064315796, -0.6585110425949097, 0.11546099931001663]]
# [[2.79306006431580, -0.658523459749002, 0.11546099931001663], [2.79306006431580, -0.658540379788477, 3.029949903488159], [-35.9446014911255, -0.657152771949768, 0.11546099931001663], [-36.4227867956602, -0.657152771949768, 3.029949903488159], [1.18335831165314, -0.658466502295604, 0.11546099931001663], [1.18335831165314, -0.658483422335079, 3.029949903488159], [2.44213295096219, -0.658511042594910, 0.11546099931001663], [1.96394764642755, -0.658511042594910, 3.029949903488159]]
# 11
# side_line_points :  [[-4.57194995880127, -0.947819885006191, 0.13765360414981842], [-4.57194995880127, -0.947606948039557, 3.0333468914031982], [-0.242313340306282, -0.948083894248523, 0.13765360414981842], [-0.242313340306282, -0.947870957281889, 3.0333468914031982]]
# [[-0.24231334030628204, -0.9554921984672546, 3.0333468914031982], [-0.24231334030628204, -0.9554921984672546, 0.13765360414981842], [-0.24231334030628204, -0.9221389889717102, 3.0333468914031982], [-0.24231334030628204, -0.9221389889717102, 0.13765360414981842], [-4.5719499588012695, -0.9221389889717102, 3.0333468914031982], [-4.5719499588012695, -0.9221389889717102, 0.13765360414981842], [-4.5719499588012695, -0.9554921984672546, 3.0333468914031982], [-4.5719499588012695, -0.9554921984672546, 0.13765360414981842], [-0.24231334030628204, -0.9554921984672546, 3.0333468914031982], [-0.24231334030628204, -0.9554921984672546, 0.13765360414981842]]
# [[-0.242313340306282, -0.948083894248523, 0.13765360414981842], [-0.242313340306282, -0.947870957281889, 3.0333468914031982], [-425.727462866096, -0.922138988971710, 0.13765360414981842], [-422.235389331001, -0.922138988971710, 3.0333468914031982], [-4.57194995880127, -0.947819885006191, 0.13765360414981842], [-4.57194995880127, -0.947606948039557, 3.0333468914031982], [121.250650099296, -0.955492198467255, 0.13765360414981842], [124.742723634391, -0.955492198467255, 3.0333468914031982]]
# 12
# side_line_points :  [[-0.0270605486204793, -2.97115015983582, 0.11777067184448242], [-0.0262917258551813, -2.97115015983582, 2.98539662361145], [-0.0737659791723643, -0.928264439105988, 0.11777067184448242], [-0.0729971564070663, -0.928264439105988, 2.98539662361145]]
# [[-0.023206600919365883, -2.9711501598358154, 2.98539662361145], [-0.023206600919365883, -2.9711501598358154, 0.11777067184448242], [-0.023206600919365883, -0.9282644391059875, 2.98539662361145], [-0.023206600919365883, -0.9282644391059875, 0.11777067184448242], [-0.06102349981665611, -0.9282644391059875, 2.98539662361145], [-0.06102349981665611, -0.9282644391059875, 0.11777067184448242], [-0.06102349981665611, -2.9711501598358154, 2.98539662361145], [-0.06102349981665611, -2.9711501598358154, 0.11777067184448242], [-0.023206600919365883, -2.9711501598358154, 2.98539662361145], [-0.023206600919365883, -2.9711501598358154, 0.11777067184448242]]
# [[-0.0232066009193659, -3.13972102266723, 0.11777067184448242], [-0.0232066009193659, -3.10609287642299, 2.98539662361145], [-0.0737659791723643, -0.928264439105988, 0.11777067184448242], [-0.0729971564070663, -0.928264439105988, 2.98539662361145], [-0.0610234998166561, -1.48561780925751, 0.11777067184448242], [-0.0610234998166561, -1.45198966301326, 2.98539662361145], [-0.0270605486204793, -2.97115015983582, 0.11777067184448242], [-0.0262917258551813, -2.97115015983582, 2.98539662361145]]
# 13
# side_line_points :  [[-0.215555225937027, -3.01844000816345, 0.08674249798059464], [-0.215549807034945, -3.01844000816345, 2.9548499584198], [-0.215567739378466, -1.13197004795074, 0.08674249798059464], [-0.215562320476383, -1.13197004795074, 2.9548499584198]]
# [[-0.2155580073595047, -3.018440008163452, 2.9548499584198], [-0.2155580073595047, -3.018440008163452, 0.08674249798059464], [-0.2155580073595047, -1.1319700479507446, 2.9548499584198], [-0.2155580073595047, -1.1319700479507446, 0.08674249798059464], [-0.2155580371618271, -1.1319700479507446, 2.9548499584198], [-0.2155580371618271, -1.1319700479507446, 0.08674249798059464], [-0.2155580371618271, -3.018440008163452, 2.9548499584198], [-0.2155580371618271, -3.018440008163452, 0.08674249798059464], [-0.2155580073595047, -3.018440008163452, 2.9548499584198], [-0.2155580073595047, -3.018440008163452, 0.08674249798059464]]
# [[-0.215558007359505, -2.59912530755782, 0.08674249798059464], [-0.215558007359505, -1.78219608420728, 2.9548499584198], [-0.215567739378466, -1.13197004795074, 0.08674249798059464], [-0.215562320476383, -1.13197004795074, 2.9548499584198], [-0.215558037161827, -2.59463244392920, 0.08674249798059464], [-0.215558037161827, -1.77770322057866, 2.9548499584198], [-0.215555225937027, -3.01844000816345, 0.08674249798059464], [-0.215549807034945, -3.01844000816345, 2.9548499584198]]
# 14
# side_line_points :  [[3.08986993482890, -3.06476092338562, 0.1359969973564148], [3.06411148382826, -3.06476092338562, 3.0182700157165527], [3.08772427854637, -2.37992000579834, 0.1359969973564148], [3.06196582754573, -2.37992000579834, 3.0182700157165527]]
# [[3.0890004634857178, -3.06476092338562, 3.0182700157165527], [3.0890004634857178, -3.06476092338562, 0.1359969973564148], [3.0890004634857178, -2.37992000579834, 3.0182700157165527], [3.0890004634857178, -2.37992000579834, 0.1359969973564148], [3.0468053817749023, -2.37992000579834, 3.0182700157165527], [3.0468053817749023, -2.37992000579834, 0.1359969973564148], [3.0468053817749023, -3.06476092338562, 3.0182700157165527], [3.0468053817749023, -3.06476092338562, 0.1359969973564148], [3.0890004634857178, -3.06476092338562, 3.0182700157165527], [3.0890004634857178, -3.06476092338562, 0.1359969973564148]]
# [[3.08900046348572, -2.78724697237374, 0.1359969973564148], [3.08900046348572, -11.0087134591174, 3.0182700157165527], [3.08772427854637, -2.37992000579834, 0.1359969973564148], [3.06196582754573, -2.37992000579834, 3.0182700157165527], [3.04680538177490, 10.6803893455964, 0.1359969973564148], [3.04680538177490, 2.45892285885275, 3.0182700157165527], [3.08986993482890, -3.06476092338562, 0.1359969973564148], [3.06411148382826, -3.06476092338562, 3.0182700157165527]]
# 15
# side_line_points :  [[1.55005116219798, -3.05101633071899, 0.14438675343990326], [1.57209742164506, -3.05101633071899, 3.0426712036132812], [1.55143437606523, -2.37992000579834, 0.14438675343990326], [1.57348063551231, -2.37992000579834, 3.0426712036132812]]
# [[1.586609959602356, -3.051016330718994, 3.0426712036132812], [1.586609959602356, -3.051016330718994, 0.14438675343990326], [1.586609959602356, -2.37992000579834, 3.0426712036132812], [1.586609959602356, -2.37992000579834, 0.14438675343990326], [1.5487998723983765, -2.37992000579834, 3.0426712036132812], [1.5487998723983765, -2.37992000579834, 0.14438675343990326], [1.5487998723983765, -3.051016330718994, 3.0426712036132812], [1.5487998723983765, -3.051016330718994, 0.14438675343990326], [1.586609959602356, -3.051016330718994, 3.0426712036132812], [1.586609959602356, -3.051016330718994, 0.14438675343990326]]
# [[1.58660995960236, 14.6862802381398, 0.14438675343990326], [1.58660995960236, 3.99005744608262, 3.0426712036132812], [1.55143437606523, -2.37992000579834, 0.14438675343990326], [1.57348063551231, -2.37992000579834, 3.0426712036132812], [1.54879987239838, -3.65810682178884, 0.14438675343990326], [1.54879987239838, -14.3543296138460, 3.0426712036132812], [1.55005116219798, -3.05101633071899, 0.14438675343990326], [1.57209742164506, -3.05101633071899, 3.0426712036132812]]