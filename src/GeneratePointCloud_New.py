import numpy as np
import pcl
import random
import pcl.pcl_visualization
import math
import PointCloudUtils
from shapely.geometry import box
from graph_cycle_new import MakingGraph
import sys
import logging
from sympy import Symbol, solve, Eq

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
count_index = 0
CHECK_PLANE_DISTANCE = 0.05
class GeneratePointCloud:

    def __init__(self, distance_threshold, epsilon_value, segmented_wall, segmented_door, segmented_window):
        self.distance_threshold = distance_threshold
        self.epsilon_value = epsilon_value
        self.segmented_wall = segmented_wall
        self.segmented_door = segmented_door
        self.segmented_window = segmented_window
        self.wall_plane_count = 0
        self.__iterations = 1000
        self.checkCount = 0
    def get_plane_list(self, clustered_cloud):    # PointCloudUtils.gridMapForPointCloud(wall_cloud)
        """Planar data extraction from clustered cloud

        Execute the loop statement until the cloud size is greater than
        or equal to the clustered_cloud size * min_percentage value.
        During loop statement execution, Plane RANSAC is executed.
        Maintain a loop statement by updating information that is not extracted by plane with cloud data.
        Args:
            clustered_cloud: Clustered data from original data

        Returns:
            new_plane_list: List of Pointcloud data extracted by plane
        """

        plane_list = list()
        normal_vector = list()

        cloud = clustered_cloud

        original_size = cloud.size
        count = 0
        loopcount = 0
        while True:

            if cloud.height * cloud.width < original_size * 0.01:
                # Stop the loop when pointcloud size less than original_size * 0.01
                break
            # Get plane information using pcl ransac
            inliers_p, outliers_p, coeff_p = self.do_plane_ransac(cloud)
            default_vector = [0, 0, -1]
            model_value = [coeff_p[0], coeff_p[1], coeff_p[2]]
            model_cos = np.dot(default_vector, model_value) / (
                    np.linalg.norm(model_value) * np.linalg.norm(default_vector))

            if math.fabs(round(model_cos, 1)) <= 0.1:
                # In the case of a plane perpendicular to the floor surface
                # if inliers_p.size >= (1250 * 0.5): checking here
                if inliers_p.size >= 100:
                    count += 1
                    plane_list.append(inliers_p) # Add a list of points above the plane
                    normal_vector.append(coeff_p) # Plane equation [a, b, c, d]
                    cloud = outliers_p # Other points except inliers_p
                    loopcount = 0
                else:
                    #
                    loopcount += 1
                    cloud = outliers_p
            else:
                loopcount += 1
                cloud = outliers_p

            if loopcount == 10:
                # Stop when the number of points is continuously extracted below the standard
                break

        return plane_list

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

        segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  # Fit a plane to the points.
        segmenter.set_optimize_coefficients(True)  # Do a little bit more optimisation once the plane has been fitted.
        segmenter.set_normal_distance_weight(self.distance_threshold)
        # Use RANSAC for the sample consensus algorithm.
        segmenter.set_max_iterations(self.__iterations)  # Number of iterations for the RANSAC algorithm.
        segmenter.set_distance_threshold(
            self.distance_threshold)  # The max distance from the fitted model a point can be for it to be an inlier.
        # 0.05 / 100000 / 0.05 / 0.062
        inlier_indices, coefficients = segmenter.segment()  # Returns all the points that fit the model, and the parameters of the model.

        # Save all the inliers as a point cloud. This forms the table which the mug sits on.
        inliers = cloud.extract(inlier_indices, negative=False)

        # Save all the outliers as a point cloud.
        outliers = cloud.extract(inlier_indices, negative=True)

        return inliers, outliers, coefficients
    def clustering2(self, cloud, tolerance=1.5, min_cluster_size=2000):
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
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(tolerance)  # 0.5 = 50cm
        ec.set_MinClusterSize(min_cluster_size)  # impose that the clusters found must have at least
        ec.set_MaxClusterSize(cloud_value.size)  # impose that the clusters found must have at Maximum
        ec.set_SearchMethod(tree)

        cluster_indices = ec.Extract()  # return index number that is included in the result of clustering

        for j, indices in enumerate(cluster_indices):

            cloud_cluster = pcl.PointCloud()

            points = np.zeros((len(indices), 3), dtype=np.float32)

            for i, indice in enumerate(indices):
                points[i][0] = cloud[indice][0]
                points[i][1] = cloud[indice][1]
                points[i][2] = cloud[indice][2]

            cloud_cluster.from_array(points)
            cluster_list.append(cloud_cluster)

        if len(cluster_list) != 0:

            return cluster_list
        else:
            cluster_list.append(cloud)

            return cluster_list
    def plane_cluster(self, cloud_data):
        """Clustering the exported plane data

            Finding clustered points in the data extracted as planes.

        Args:
            cloud_data: Pointcloud data extracted by plane
        Returns:
            new_cloud_data: Clustered PointCloud data list
        """
        max_size = cloud_data.size # The number of points included in plane
        min_size = 100 # Minimum size of point included in each result of clustering
        new_data_st = list()
        count = 0
        check_count = 0
        while True:

            if cloud_data.size < min_size or count > 10:
                break
            tree = cloud_data.make_kdtree()
            segment = cloud_data.make_EuclideanClusterExtraction()
            segment.set_ClusterTolerance(0.1)  # distance of clustering
            segment.set_MinClusterSize(min_size)
            segment.set_MaxClusterSize(max_size)
            segment.set_SearchMethod(tree)
            cluster_indices = segment.Extract()

            if len(cluster_indices) != 0:
                inliers = cloud_data.extract(cluster_indices[0], negative=False) # Save all the inliers as a point cloudd
                outliers = cloud_data.extract(cluster_indices[0], negative=True) # Save all the outliers as a point cloud.

                if inliers.size >= min_size:
                    inliers_p, outliers_p, coeff_p = self.do_plane_ransac(inliers)

                    bbox_info = PointCloudUtils.get_range(inliers) # Getting the bounding box information [[min_x, min_y, min_z], [max_x, max_y, max_z]]

                    # Use of information with a height of 40 cm or more
                    check_height = True if math.fabs(bbox_info[1][2] - bbox_info[0][2]) > 1.0 else False
                    check_count += 1

                    m_minX = bbox_info[0][0]
                    m_minY = bbox_info[0][1]
                    m_maxX = bbox_info[1][0]
                    m_maxY = bbox_info[1][1]

                    make_box = box(m_minX, m_minY, m_maxX, m_maxY)
                    # Use of information with a area of 10 or more
                    if check_height is False:
                        check_height = True if math.fabs(bbox_info[0][2]) >= 1.5 else False
                    check_area = True if make_box.area > 0.1 else False

                    if check_height:
                        # 4 outer points of extracted result of clustering
                        side_points = PointCloudUtils.make_side_line(bbox_info, coeff_p)
                        # Adding the list of points, plane equation and outer points
                        new_data_st.append([inliers, coeff_p, side_points])

                        count = 0
                    cloud_data = outliers
                else:
                    cloud_data = outliers
                    count += 1
            else:
                break
        return new_data_st

    def merged_clustered_wall(self, all_wall):
        """Merging the same plane equation data

               Merging planes that are parallel and within a certain distance

            Args:
                all_wall: Clustered PointCloud List ( sorting the descending order)
            Returns:
                merged_point_cloud: List of merged PointCloud
                merged_normal_vector: List of each PointCloud's plane equation
                merged_bbox_info: List of each PointCloud's boundingbox info
       """
        used_v = [True for i in range(len(all_wall))]
        merged_point_cloud = []
        merged_normal_vector = []
        merged_bbox_info = []

        for i in range(len(all_wall)):
            new_wall_list = list()
            if used_v[i]:
                used_v[i] = False
                new_wall_list.append(all_wall[i][0])
                main_v = all_wall[i][1]
                for j in range(i + 1, len(all_wall)):

                    if used_v[j]:
                        sub_v = all_wall[j][1]

                        # Create an angle between each plane
                        model_cos = np.dot([main_v[0], main_v[1], main_v[2]], [sub_v[0], sub_v[1], sub_v[2]]) / (
                                np.linalg.norm([main_v[0], main_v[1], main_v[2]]) * np.linalg.norm(
                            [sub_v[0], sub_v[1], sub_v[2]]))
                        # Execute when the absolute value of the cos value is 0.9 to 1
                        if math.fabs(round(model_cos, 2)) > 0.9:

                            side_points = all_wall[j][2]
                            # Calculate the distance of the outside point of the base and mating face
                            avg_d = PointCloudUtils.avg_distance(all_wall[i][1], side_points)

                            if avg_d <= CHECK_PLANE_DISTANCE:
                                # Adding the information if the average distance is less than 10cm
                                used_v[j] = False
                                new_wall_list.append(all_wall[j][0])
                # Combined into one piece of data based on the added information
                m_point_cloud, m_bbox_info = PointCloudUtils.merge_one_file(new_wall_list)
                merged_point_cloud.append(m_point_cloud)
                merged_normal_vector.append(all_wall[i][1])
                merged_bbox_info.append(m_bbox_info)
            else:
                continue

        logger.info("Finish Merging")
        return merged_point_cloud, merged_normal_vector, merged_bbox_info

    def get_intersection_line(self, wallPointCloudList, wallCoeffList, wallBBoxList):
        """Finding the intersection points from each plane

                1) Finding the intersection points between each planes
                2) Sorting the point using bottom point
                3) Making the line segment (line segment with more than a certain amount of points)

            Args:
                wallPointCloudList: Merged PointCloud list
                wallCoeffList: List of plane equation for each merged plane
                wallBBoxList: List of bounding box information for each merged plane
            Returns:
                intersection_result2: All line segment included in each plane
        """
        each_wall_info = [[] for i in range(len(wallCoeffList))]

        x = Symbol('x')
        y = Symbol('y')
        z = 0.0

        intersection_result = []
        line_rate_list = []

        for point_i in range(len(wallPointCloudList)):

            for point_j in range(point_i + 1, len(wallPointCloudList)):
                # Create an angle between each plane
                model_cos = PointCloudUtils.get_cos(wallCoeffList[point_i], wallCoeffList[point_j])

                if math.fabs(round(model_cos, 1)) != 1.0:
                    # Run if two planes are not parallel
                    main_coeff = wallCoeffList[point_i]
                    match_coeff = wallCoeffList[point_j]
                    temp_list = np.cross(
                        [main_coeff[0], main_coeff[1], main_coeff[2]],
                        [match_coeff[0], match_coeff[1], match_coeff[2]])
                    e1 = Eq(
                        main_coeff[0] * x + main_coeff[1] * y + main_coeff[3],
                        0)
                    e2 = Eq(
                        match_coeff[0] * x + match_coeff[1] * y +
                        match_coeff[3],
                        0)
                    # Generate equations of intersection line using equations in plane
                    value_eq = solve([e1, e2], x, y)
                    temp_list = temp_list.tolist()
                    temp_list.append(value_eq[x])
                    temp_list.append(value_eq[y])
                    temp_list.append(z)
                    # Creating intersection points using intersecting line equations and bounding box information
                    main_minZ = wallBBoxList[point_i][0][2]
                    sub_minZ = wallBBoxList[point_j][0][2]
                    main_maxZ = wallBBoxList[point_i][1][2]
                    sub_maxZ = wallBBoxList[point_j][1][2]

                    main_minT = (main_minZ - temp_list[5]) / temp_list[2]
                    main_minX = (main_minT * temp_list[0]) + temp_list[3]
                    main_minY = (main_minT * temp_list[1]) + temp_list[4]

                    main_maxT = (main_maxZ - temp_list[5]) / temp_list[2]
                    main_maxX = (main_maxT * temp_list[0]) + temp_list[3]
                    main_maxY = (main_maxT * temp_list[1]) + temp_list[4]

                    sub_minT = (sub_minZ - temp_list[5]) / temp_list[2]
                    sub_minX = (sub_minT * temp_list[0]) + temp_list[3]
                    sub_minY = (sub_minT * temp_list[1]) + temp_list[4]

                    sub_maxT = (sub_maxZ - temp_list[5]) / temp_list[2]
                    sub_maxX = (sub_maxT * temp_list[0]) + temp_list[3]
                    sub_maxY = (sub_maxT * temp_list[1]) + temp_list[4]

                    if PointCloudUtils.check_range([main_minX, main_minY], wallBBoxList[point_i], self.epsilon_value):
                        # Check whether the created intersection is included in the bounding box of each plane.
                        if PointCloudUtils.check_range([sub_minX, sub_minY], wallBBoxList[point_j], self.epsilon_value):
                            # When the intersection point is contained, the values are added to each array.

                            main_point_bot = [main_minX, main_minY, main_minZ]
                            main_point_top = [main_maxX, main_maxY, main_maxZ]
                            sub_point_bot = [main_minX, main_minY, sub_minZ]
                            sub_point_top = [sub_maxX, sub_maxY, sub_maxZ]
                            main_points = [main_point_bot, main_point_top]
                            sub_points = [sub_point_bot, sub_point_top]
                            each_wall_info[point_i].append(main_points)
                            each_wall_info[point_j].append(sub_points)

            if len(each_wall_info[point_i]) > 1:
                # Align the points
                sorted_PointList = PointCloudUtils.sorted2Dpoints(each_wall_info[point_i])
                each_all_lines2, line_rate = PointCloudUtils.get_lengthRate(wallPointCloudList[point_i], wallCoeffList[point_i], sorted_PointList, self.distance_threshold)
                if len(each_all_lines2) != 0:
                    # Data is saved when the number of generated line segments is not 0
                    intersection_result.append(each_all_lines2)
                    line_rate_list.extend(line_rate)

        return intersection_result


    def make_room_info(self):
        logger.info("Starting the function to make the room information")
        logger.info("Starting the Clustering function")



        downsampling_result = PointCloudUtils.grid_subsampling_withoutRGB(self.segmented_wall.to_array(), 0.05)
        new_wall_data = pcl.PointCloud()
        new_wall_data.from_list(downsampling_result)
        cluster_list = self.clustering2(new_wall_data)

        logger.info("Number of Clustered data : " + str(len(cluster_list)))

        for clustered_i, clustered_cloud in enumerate(cluster_list):


            logger.info("Starting to make the room information from each Cluster data : " + str(clustered_i + 1))

            # Extract all planes information contained in PointCloud

            wallPointCloudList = self.get_plane_list(clustered_cloud)

            all_wall = list()
            for each_wall_cloud in wallPointCloudList:
                # Split each plane
                clustered_wall_data = self.plane_cluster(each_wall_cloud)
                all_wall.extend(clustered_wall_data)

            # Sort the divided planes in descending order based on the number of points contained
            all_wall = sorted(all_wall, key=lambda all_wall: all_wall[0].size, reverse=True)

            # Combine parallel and adjacent data
            merged_point_cloud, merged_normal_vector, merged_bbox_info = self.merged_clustered_wall(all_wall)

            logger.info("Finishing to merge the each wall data with the same plane equation into one: " + str(clustered_i + 1))

            if len(merged_normal_vector) > 2:

                logger.info("Starting to get intersection points between each wall " + str(clustered_i + 1))
                # Finding the intersection points and making the line segments
                intersection_result2 = self.get_intersection_line(merged_point_cloud, merged_normal_vector,
                                                                  merged_bbox_info)

                # Making the graph using line segments included in each planes

                Graph2CtiyGML = MakingGraph(intersection_result2)
                # Making the CityGML room information using cycles

                result_ceiling, result_floor, result_wall = Graph2CtiyGML.make_graph()
                if result_ceiling != None and result_floor != None and result_wall != None:
                    PointCloudUtils.saveAsWKT(clustered_i, result_ceiling, result_floor, result_wall)


if __name__ == "__main__":

    wall_cloud = pcl.load("../data/sample_data/original_data_result/dump/original_data_wall.ply")
    GeneratePointCloudData = GeneratePointCloud(0.052, 0.1, wall_cloud, [], "123")
    only_wall_list = GeneratePointCloudData.make_room_info()

