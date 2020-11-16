import numpy as np
import pcl
import random
import pcl.pcl_visualization
import math
from sympy import Symbol, solve, Eq
import matplotlib.pyplot as plt
import Point_sort as ps
import time
from shapely.geometry import box
from graph_cycle import MakingGraph2
import citygml.PointCloud_To_CityGML as gml

count_index = 0
import sys
import logging

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
class GeneratePointCloud:

    def __init__(self, distance_threshold,epsilon_value, segmented_wall, segmented_door=[], segmented_window=[]):
        self.distance_threshold = distance_threshold
        self.epsilon_value = epsilon_value
        self.segmented_wall = segmented_wall
        self.segmented_door = segmented_door
        self.segmented_window = segmented_window

    def clustering(self, cloud, vg_size=0.1, tolerance=0.5, min_cluster_size=1000):
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

    def do_plane_ransac(self, cloud):
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
        segmenter.set_max_iterations(1000)  # Number of iterations for the RANSAC algorithm.
        segmenter.set_distance_threshold(self.distance_threshold) # The max distance from the fitted model a point can be for it to be an inlier.
        #0.05 / 100000 / 0.05 / 0.062
        inlier_indices, coefficients = segmenter.segment() # Returns all the points that fit the model, and the parameters of the model.

        # Save all the inliers as a point cloud. This forms the table which the mug sits on.
        inliers = cloud.extract(inlier_indices, negative=False)

        # Save all the outliers as a point cloud.
        outliers = cloud.extract(inlier_indices, negative=True)

        return inliers, outliers, coefficients

    def do_plane_ransac2(self, cloud):
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


    def check_distance_plane(self, point_cloud, coeff, e=0.01):
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

    def plane_cluster(self, cloud_data):
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
                inliers_p, outliers_p, coeff_p = self.do_plane_ransac(inliers)
                new_cloud_data.append(inliers_p)
                normal_vector.append(coeff_p)
                outliers_data = outliers
        return new_cloud_data, normal_vector, outliers_data

    def merge_dup_plane(self, plane_list, normal_vector):
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
                    main_bbox = self.get_range(plane_list[i])
                    prev_bbox[i].extend(main_bbox)
                else:
                    main_bbox = prev_bbox[i]
                if len(prev_bbox[j]) == 0:
                    sub_bbox = self.get_range(plane_list[j])
                    prev_bbox[j].extend(sub_bbox)
                else:
                    sub_bbox = prev_bbox[j]

                if self.check_bbox(main_bbox, sub_bbox):
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
    #                     # print model_cos, distance_bw_planes, i, j, check_distance_plane(plane_list[i], normal_vector[i]), check_distance_plane(plane_list[j], normal_vector[j])
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
    #     # print dup_plane_index
        if len(dup_plane_index) != 0:
            for dup_index in dup_plane_index:
    #             # print dup_index
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

    #         # print all_dup_index
            for each_plane_i in range(len(plane_list)):
    #             # print each_plane_i, each_plane_i not in all_dup_index
                if each_plane_i not in all_dup_index:
                    new_plane_list.append(plane_list[each_plane_i])
                    new_normal_vector.append(normal_vector[each_plane_i])
                    new_bbox_list.append(prev_bbox[each_plane_i])
    #                 # print len(new_plane_list), len(new_normal_vector), len(new_bbox_list)

            return new_plane_list, new_normal_vector, new_bbox_list
        else:
            return plane_list, normal_vector, prev_bbox



    def get_plane_list(self, clustered_cloud):
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

            if cloud.height * cloud.width < 100:

                break

            inliers_p, outliers_p, coeff_p = self.do_plane_ransac(cloud)
            default_vector = [0, 0, -1]
            model_value = [coeff_p[0], coeff_p[1], coeff_p[2]]
            model_cos = np.dot(default_vector, model_value) / (
                    np.linalg.norm(model_value) * np.linalg.norm(default_vector))
            #
            if math.fabs(model_cos) < 0.8:
                if inliers_p.size > 200:
                    plane_list.append(inliers_p)
                    normal_vector.append(coeff_p)
                    cloud = outliers_p
                else:
                    cloud = outliers_p
                # clustered_plane_list, clustered_plane_coeff, outliers_data = self.plane_cluster(inliers_p)
                # if len(clustered_plane_list) != 0:
                #     plane_list.extend(clustered_plane_list)
                #     normal_vector.extend(clustered_plane_coeff)
                #     new_outliers_p = pcl.PointCloud()
                #     new_outliers_p.from_list(outliers_data.to_list() + outliers_p.to_list())
                #     cloud = new_outliers_p
                # else:
                #     cloud = outliers_p
            else:
                cloud = outliers_p

        self.visual_viewer(plane_list)
        # new_plane_list, new_normal_vector, new_bbox_list = self.merge_dup_plane(plane_list, normal_vector)
        # new_plane_list2 = []
        # new_bbox_list2 = []
        # new_normal_vector2 = []
        # for each_wall in new_plane_list:
        #     fil = each_wall.make_statistical_outlier_filter()
        #     # fil.set_mean_k(50)
        #     # fil.set_std_dev_mul_thresh(0.5)
        #     new_cloud = fil.filter()
        #
        #     inliers_p, outliers_p, coeff_p = self.do_plane_ransac2(new_cloud)
        #     new_plane_list2.append(inliers_p)
        #
        #     new_normal_vector2.append(coeff_p)
        #     new_bbox_list2.append(self.get_range(new_cloud))
        #
        # return new_plane_list2, new_normal_vector2, new_bbox_list2


    def make_room_info(self):
        """Making the wall information using PointCloud data

            Clustering the original point cloud data.
            Extracting the plane data from the clustered data through Plane RANSAC
            Find intersections between extracted plane data and generate surface data

        Args:


        Returns:
            surface_point_list: A list consisting of a list of points that make up each surface
        """

        # print "Make Wall Info"
        logger.info("Starting the function to make the room information")
        # clustering the pointcloud data
        logger.info("Starting the Clustering function")
        cluster_list = self.clustering(self.segmented_wall)
        logger.info("Number of Clustered data : "+str(len(cluster_list)))
        room_surface_list = list()
        p_s = ps.Point_sort()

        count = 0

        all_room = []
        for clustered_cloud in cluster_list:
            count += 1
            logger.info("Starting to make the room information from each Cluster data : "+str(count))
            self.get_plane_list(clustered_cloud)
            # wall_point_list, wall_vector_list, wall_bbox_list = self.get_plane_list(clustered_cloud)
            # self.visual_viewer(wall_point_list)
            # side_line_info = []
            # for wall_index in range(len(wall_point_list)):
            #
            #     side_line_info.append(self.make_side_line(wall_bbox_list[wall_index], wall_vector_list[wall_index]))


            # result_ceiling, result_floor, result_wall = self.get_intersection_line(wall_bbox_list, wall_vector_list, side_line_info, wall_point_list)
            # for each_i in range(len(result_wall)) :
            #     make_CityGML = gml.PointCloudToCityGML([result_ceiling[each_i]], [result_floor[each_i]], result_wall[each_i], [], [])
            #     make_CityGML.MakeRoomObject()
    def make_chair_info(self, cloud, save_path):
        """Making the chair information

            Clustering the chair data.
            And returns the bounding box of each clustered data

        Args:
            cloud: PointCloud data of chair
            save_path: Save path of the clustered data
        Returns:
            clustered_bbox: The bounding box of each clustered data
        """
        chair_range = self.get_range(cloud.to_list())
        cluster_list = self.clustering(cloud, vg_size=0.05, tolerance=0.1, min_cluster_size=100)
        main_area = self.get_area(chair_range)
        i = 0

        clustered_bbox = list()
        if len(cluster_list) == 0:
            temp_cloud = cloud.to_array()

            cloud_cluster = pcl.PointCloud()
            cloud_cluster.from_array(temp_cloud)
            clustered_chair_range = self.get_range(temp_cloud)
            clustered_bbox.append(clustered_chair_range)


        else:
            if len(cluster_list) == 1:

                temp_cloud = cluster_list[0].to_array()

                cloud_cluster = pcl.PointCloud()
                cloud_cluster.from_array(temp_cloud)
                clustered_chair_range = self.get_range(temp_cloud)
                clustered_bbox.append(clustered_chair_range)

            else:
                for clustered_cloud in cluster_list:
                    i += 1
                    temp_cloud = clustered_cloud.to_array()

                    clustered_chair_range = self.get_range(temp_cloud)
                    temp_area = self.get_area(clustered_chair_range)

                    if temp_area <= main_area / 2.0:
                        clustered_bbox.append(clustered_chair_range)
                        cloud_cluster = pcl.PointCloud()
                        cloud_cluster.from_array(temp_cloud)
                        out_data_chair_filename = save_path + "_clustered_chair_" + str(i) + ".pcd"
                        pcl.save(cloud_cluster, out_data_chair_filename)

        return clustered_bbox

    def get_range(self, point_cloud, e=0.0):
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


        return [point_max2, point_min2]

    def get_area(self, bbox_range):
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

    def make_side_line(self, bbox_info, normal_vector):
        """Making side lines of plane

            Using the bounding box and plane data, create straight lines at both ends of the plane.

        Args:
            bbox_info: Bounding box data of PointCloud
            normal_vector: Coefficient data of plane

        Returns:
            line_points_list: A list of the maximum and minimum points that can be made with straight lines at both ends
        """

        side_line_points = self.find_side_point(normal_vector, bbox_info)

        return side_line_points


    def make_straight(self, normal_vector, point_1, point_2):
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


        return intersect_point


    def make_straight2(self, normal_vector, point_1, point_2, min_z, max_z):
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

    def find_side_point(self, plane_vector, boundary_info):
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
            min_result, max_result = self.make_straight2(plane_vector, box_coords[box_i], box_coords[box_i+1], min_z, max_z)

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

        return point_list


    def check_point_side(self, point, side_lines):

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


    def check_point_range_2(self, point, wall_range):
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
    def check_point_on_line(self, point, line_normal, e=0.0):
        a = line_normal[0]
        b = line_normal[1]
        c = line_normal[2]
        x1 = point[0]
        y1 = point[1]
        d = math.fabs(a*x1 + b*y1 + c) / math.sqrt(math.pow(a, 2)+math.pow(b, 2))

        if d <= e:
            return True
        else:
            return False
    def get_lineNormal(self, line_normal):
        print line_normal
        x1 = line_normal[0][0]
        y1 = line_normal[0][1]
        x2 = line_normal[1][0]
        y2 = line_normal[1][1]
        a = y2-y1
        b = x2-x1
        c = a*x1 - b*y1
        return [a, b, c]
    def check_point_range_e(self, point, wall_range, e=0.0):
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

        if x <= wall_range_max[0]:
            if x >= wall_range_min[0]:
                check_X = check_X + 1
        if y <= wall_range_max[1] + e:
            if y >= wall_range_min[1] - e:
                check_Y = check_Y + 1
        # if z <= wall_range_max[2] + e:
        #     if z >= wall_range_min[2] - e:
        #         check_Z += 1
    #     # print check_X, check_Y
        if (check_X + check_Y) == 2:
            return True
        else:
            return False
    def check_bbox(self, main_bbox, other_bbox):
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

    def check_bbox2(self, main_bbox, other_bbox):
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



        return m_box.intersects(s_box)


    def extend_bbox(self, side_points, e):

        points = []
        max_points = side_points[2:]
        min_points = side_points[:2]
        for max_point in max_points:
            points.append([max_point[0]+e, max_point[1] + e, max_point[2]])
        for min_point in min_points:
            points.append([min_point[0]-e, min_point[1] - e, min_point[2]])
        new_range = self.get_range(points)

        return new_range




    def get_intersection_line(self, bbox_list, normal_vector, side_line_info, wall_point_list):
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

                        if self.check_bbox2(bbox_list[point_i], bbox_list[point_j]):
                            if point_j not in check_bbox_index[point_i]:
                                check_bbox_index[point_i].append(point_j)
                            if point_i not in check_bbox_index[point_j]:
                                check_bbox_index[point_j].append(point_i)

        for check_wall_i in range(len(check_wall_info)):
            main_epsilon = 0.1
            sub_epsilon = 0.1
            max_e = self.epsilon_value
            count = 0
            while True:
                main_bbox = self.extend_bbox(side_line_info[check_wall_i], main_epsilon)
    #
                for check_wall in check_wall_info[check_wall_i]:

                    sub_bbox = self.extend_bbox(side_line_info[check_wall], sub_epsilon)

                    if check_wall not in check_bbox_index[check_wall_i]:
                        if self.check_bbox2(main_bbox, sub_bbox):
                            count = count + 1
                            check_bbox_index[check_wall_i].append(check_wall)
                            check_bbox_index[check_wall].append(check_wall_i)


                if main_epsilon > max_e:
                    break

                main_epsilon = main_epsilon + 0.005
                sub_epsilon = sub_epsilon + 0.005

        delete_node = []

        for each_i in range(len(check_bbox_index)):
            if len(check_bbox_index[each_i]) == 1:
                delete_node.append([each_i, check_bbox_index[each_i][0]])
    #     # print delete_node
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
                        sub_minZ = bbox_list[main_i][1][2]
                        main_maxZ = bbox_list[match_i][0][2]
                        sub_maxZ = bbox_list[main_i][0][2]

                        if main_minZ <= sub_minZ:
                            minZ = main_minZ
                        else:
                            minZ = sub_minZ

                        if main_maxZ >= sub_maxZ:
                            maxZ = main_maxZ
                        else:
                            maxZ = sub_maxZ
                        minT = (0.0 - temp_list[5]) / temp_list[2]
                        minX = (minT * temp_list[0]) + temp_list[3]
                        minY = (minT * temp_list[1]) + temp_list[4]
                        main_point_bot = [minX, minY, 0]
                        main_point_top = [minX, minY, maxZ]
                        main_points = [main_point_bot, main_point_top]
                        each_wall_info[main_i].append(main_points)
                        each_wall_info[match_i].append(main_points)


        remove_index_list = []
        for i in range(len(each_wall_info)):
            if len(each_wall_info[i]) == 0:
                remove_index_list.append(i)

    #     # print "print : ", remove_index_list
        if len(remove_index_list) != 0:
            remove_count = 0
            for i in remove_index_list:
                r_i = i - remove_count
                each_wall_info.pop(r_i)
                bbox_list.pop(r_i)
                normal_vector.pop(r_i)
                wall_point_list.pop(r_i)
                remove_count = remove_count + 1
        try:
            if len(each_wall_info) > 2:
                new_graph = MakingGraph2([each_wall_info])
                checked_list, G = new_graph.make_graph()

                delete_value = self.check_point(checked_list, wall_point_list, normal_vector)

                return new_graph.get_newGraph(G, delete_value)
        except TypeError or ValueError or None:

            pass


    def check_point(self, checked_list, wall_point_list, normal_vector):
        delete_value = []

        for i in checked_list:

            pointcloud_rate = self.get_pointRate(wall_point_list[i[0]], normal_vector[i[0]], i[2], i[3])
            a = self.get_range(i[2] + i[3])
            check_a = []
            check_a.append(a[0])
            check_a.append(a[1])
            b = pcl.PointCloud()
            b.from_list(check_a)

            result_rate = pointcloud_rate * 100 <= 10
            print result_rate, pointcloud_rate, i[1], pointcloud_rate*i[1], pointcloud_rate/i[1]

            if pointcloud_rate == 0.0 or result_rate:

                delete_value.append(i[4])


        return delete_value

    def get_pointRate(self, pointcloud, normal_vector, point1, point2):
        bbox = self.get_range(point1+point2)
        line_normal = self.get_lineNormal([point1[0], point2[0]])
        points = pointcloud.to_list()
        pointcloud_size = pointcloud.size
        e = self.distance_threshold
        a, b = self.check_distance_plane(points, normal_vector, e)
        count = 0
        temp_list = []
        for point in points:

            if self.check_point_range_e(point, bbox, e):
                    temp_list.append(point)
                    count = count + 1
        a, c = self.check_distance_plane(temp_list, normal_vector,e)

        print count, float(pointcloud_size)
        if c == 0:
            return 0.0
        else:
            return float(c) / float(b)


    def visual_graph(self, point_list):
        """Visualize point list as graph

        Args:
            point_list: list of Points

        Returns:
            Visualize point list as graph
        """
        x = []
        y = []
    #     # print len(e)
        for index in point_list:
            for i in index:
                x.append(i[0])
                y.append(i[2])
        plt.scatter(x, y, label="stars", color="green",
                    marker="*", s=50)
        plt.plot(x, y)
        plt.legend()
        plt.show()


    def distance_to_point(self, point1, point2):
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

    def visual_viewer(self, cloud_list):
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

    def visual_viewer2(self, cloud, cloud_list):
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
    #     # print len(cloud_list)
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

    def z_point_sorting(self, x, y):
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

    def cmp_to_key(self, mycmp):
        """Convert a cmp= function into a key= function"""

        class K:
            def __init__(self, obj, *args):
                self.obj = obj

            def __lt__(self, other):
                return mycmp(self.obj, other.obj) < 0

            def __gt__(self, other):
                return mycmp(self.obj, other.obj) >= 0

        return K

if __name__ == "__main__":


    # wall_cloud = pcl.load("/home/dprt/Desktop/PinSout_20201106_revise/PinSout/data/1000_168/npy_data2/dump/sampling_in_d2_wall_.pcd")
    wall_cloud = pcl.load("/home/dprt/Documents/DataSet/ISPRS Benchmark Dataset/CaseStudy1/PointCloud_result_wall.ply")
    a = GeneratePointCloud(0.05, 0.5, wall_cloud)
    a.make_room_info()
