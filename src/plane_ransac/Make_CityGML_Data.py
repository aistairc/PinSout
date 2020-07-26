import pcl
import pcl.pcl_visualization
import math
from sympy import Symbol, solve, Eq
import numpy as np
import random
from ..citygml import PointCloud_To_CityGML as gml
from rdp import rdp
import matplotlib.pyplot as plt
import Point_sort as ps
''' boundary '''
# import fiona
# import shapely.geometry as geometry
# import pylab as pl

import matplotlib.pylab as plb
global_counter = 0
class MakeCityGMLData:

    def __init__(self, pred_data, ceiling_data, floor_data, wall_data, door_data, window_data):

        # self.pred_cloud = pcl.load(pred_data)
        # self.ceiling_cloud = pcl.load(ceiling_data)
        # self.floor_cloud = pcl.load(floor_data)
        # self.wall_cloud = pcl.load(wall_data)
        # self.door_cloud = pcl.load(door_data)
        # self.point_max, self.point_min = self.get_range(self.pred_cloud)
        self.pred_cloud = pred_data
        self.ceiling_cloud = ceiling_data
        self.floor_cloud = floor_data
        self.wall_cloud = wall_data
        self.door_cloud = door_data
        self.window_cloud = window_data
        # self.point_max, self.point_min = self.get_range(self.pred_cloud)
        # self.ceiling_point = list()
        # self.floor_point = list()
        # self.door_point = list()
        self.matching_index = 0
        self.distance_upsilon = 0.05



    def do_passthrough_filter(self, point_cloud, name_axis='z'):

        pass_filter = point_cloud.make_passthrough_filter()
        pass_filter.set_filter_field_name(name_axis)
        # cloud = pass_filter.filter()
        # fil_1 = cloud.make_statistical_outlier_filter()
        # fil_1.set_mean_k(10)
        # fil_1.set_std_dev_mul_thresh(1)

        # fil_1.filter()
        # pass_filter.filter()
        return pass_filter.filter()

    def do_ransac_plane_segmentation(self, point_cloud, ksearch):
        # ne = point_cloud.make_NormalEstimation()
        # tree = point_cloud.make_kdtree()
        # ne.set_SearchMethod(tree)
        # ne.set_KSearch(50)
        # ksearch = ksearch
        segmenter = point_cloud.make_segmenter_normals(ksearch=ksearch)
        segmenter.set_optimize_coefficients(True)
        segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        # segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_normal_distance_weight(0.1)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_max_iterations(100)
        segmenter.set_distance_threshold(0.1)


        inlier_indices, coefficients = segmenter.segment()

        inliers = point_cloud.extract(inlier_indices, negative=False)

        outliers = point_cloud.extract(inlier_indices, negative=True)

        return inliers, outliers, coefficients

    def do_ransac_plane_segmentation_2(self, point_cloud):
        # ne = point_cloud.make_NormalEstimation()
        # tree = point_cloud.make_kdtree()
        # ne.set_SearchMethod(tree)
        # ne.set_KSearch(50)
        segmenter = point_cloud.make_segmenter_normals(ksearch=50)
        segmenter.set_optimize_coefficients(True)

        '''
        CYLINDER - point_on_axis.x, y, z, axis_direction.x, y, z radius
        '''
        segmenter.set_model_type(pcl.SACMODEL_CYLINDER)
        segmenter.set_normal_distance_weight(0.1)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_max_iterations(10000)
        segmenter.set_distance_threshold(0.1)



        inlier_indices, coefficients = segmenter.segment()

        inliers = point_cloud.extract(inlier_indices, negative=False)

        outliers = point_cloud.extract(inlier_indices, negative=True)

        return inliers, outliers, coefficients

    def get_normal_vector(self, cloud, distance_rate=0.95, min_size=200, ksearch=50):
        global global_counter
        cloud_point = list()
        normal_vector = list()
        min_percentage = 1
        plane_list = list()
        # print ksearch
        # print min_size
        # cylinder_list = list()
        # while True:
        original_size = cloud.size
        distance_rate = distance_rate
        first_check = True
        while (cloud.height * cloud.width > original_size * min_percentage / 100):

            filtered_cloud = self.do_passthrough_filter(point_cloud=cloud, name_axis='z')
            inliers_p, outliers_p, coeff_p = self.do_ransac_plane_segmentation(filtered_cloud, ksearch)
            distance_p = self.check_distance_plane(inliers_p, coeff_p)
            # print "Plane"
            # print distance_p

            if distance_p >= distance_rate:
                # print distance_p
                # print "-----------"
                plane_list.append(inliers_p)
                # normal_vector.append(coeff_p)
                cloud = outliers_p
                # print cloud.height * cloud.width, original_size
                # first_check = False
            else:
                if first_check:
                    if distance_p >= 0.5:
                        # print distance_p
                        # print "-----------"
                        plane_list.append(inliers_p)

                        # normal_vector.append(coeff_p)
                        cloud = outliers_p
                    first_check = False
                else:
                    break
            if cloud.height * cloud.width < original_size * min_percentage / 100:
                break
        # self.visual_viewer(plane_list)
        for plane_list_index in plane_list:

            a = plane_list_index
            max_size = a.size
            min_size = min_size

            while a.size != 0:

                vg = a.make_voxel_grid_filter()
                vg.set_leaf_size(0.01, 0.01, 0.01)
                cloud_filtered = vg.filter()
                tree = cloud_filtered.make_kdtree()

                segment = cloud_filtered.make_EuclideanClusterExtraction()
                segment.set_ClusterTolerance(0.5)
                segment.set_MinClusterSize(min_size)
                segment.set_MaxClusterSize(max_size)
                segment.set_SearchMethod(tree)
                cluster_indices = segment.Extract()
                if len(cluster_indices) != 0:
                    inliers = cloud_filtered.extract(cluster_indices[0], negative=False)
                    outliers = cloud_filtered.extract(cluster_indices[0], negative=True)

                    filtered_cloud = self.do_passthrough_filter(point_cloud=inliers, name_axis='z')
                    inliers_p, outliers_p, coeff_p = self.do_ransac_plane_segmentation(filtered_cloud, ksearch)
                    cloud_point.append(inliers_p)
                    normal_vector.append(coeff_p)

                    a = outliers
                else:
                    break
        # self.visual_viewer(cloud_point)

        # if cloud.size > original_size * min_percentage / 100)
        if cloud.size / 2 > original_size * min_percentage / 100:
            while (cloud.height * cloud.width > original_size * min_percentage / 100):
                # print "Cylinder"
                filtered_cloud = self.do_passthrough_filter(point_cloud=cloud, name_axis='z')
                inliers_c, outliers_c, coeff_c = self.do_ransac_plane_segmentation_2(filtered_cloud)

                distance_c = self.check_distance_cylinder(inliers_c, coeff_c)

                if distance_c >= distance_rate:
                    # print distance_c
                    # print "-----------"
                    cloud_point.append(inliers_c)
                    normal_vector.append(coeff_c)
                    cloud = outliers_c
                else:
                    break
                if cloud.height * cloud.width < original_size * min_percentage / 100:
                    break

        return normal_vector, cloud_point


    def sorting_Z (self, point_cloud):

        vector_list, point_list = self.get_normal_vector(point_cloud)
        sorting_temp = list()
        cloud_vector_list = list()
        cloud_point_list = list()
        for index in range(len(point_list)):

            temp_matrix = np.asarray(point_list[index]).T[2]
            sum_index = sum(temp_matrix)
            ave_value = sum_index / len(temp_matrix)
            sorting_temp.append((vector_list[index], point_list[index], round(ave_value, 8), round(ave_value, 8)))
            # sorting_temp.append((vector_list[index], point_list[index], round(min(temp_matrix), 8), round(max(temp_matrix), 8)))

        for index in sorted(sorting_temp, key=lambda sorting_temp: sorting_temp[3], reverse=True):

            cloud_vector_list.append(index[0])
            cloud_point_list.append(index[1])

        return cloud_vector_list, cloud_point_list

    def check_distance_plane(self, point_cloud, coeff):

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

            point_distance = float(math.fabs(a*x + b*y + c*z + d) / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))

            if point_distance <= self.distance_upsilon:
                count += 1

        if count == 0:
            distance_rate = 0.0
        else:
            distance_rate = round(float(count) / float(len(t)), 3)
        return distance_rate

    def check_distance_cylinder(self, point_cloud, coeff):

        a = coeff[0]
        b = coeff[1]
        c = coeff[2]
        u = [coeff[3], coeff[4], coeff[5]]
        direction_vector = np.linalg.norm(u)
        r = coeff[6]
        t = point_cloud.to_list()

        count = 0
        for index in t:
            x = index[0]
            y = index[1]
            z = index[2]
            temp_vector = [x - a, y - b, z - c]

            cross_vector = np.linalg.norm(np.cross(temp_vector, u))
            point_distance = math.fabs(float(cross_vector / direction_vector) - float(r))

            if point_distance <= self.distance_upsilon:
                count += 1

        if count == 0:
            distance_rate = 0.0
        else:
            distance_rate = round(float(count) / float(len(t)), 3)
        return distance_rate


    def check_distance_point_1(self, point_list, coeff, epsilon=0.1):

        u = [float(coeff[0]), float(coeff[1]), float(coeff[2])]
        x1 = float(coeff[3])
        y1 = float(coeff[4])
        z1 = float(coeff[5])

        count = list()

        for point_list_index in point_list:

            max_xyz = point_list_index[0]
            min_xyz = point_list_index[1]

            max_xt = (max_xyz[0] - x1) / u[0]
            min_xt = (min_xyz[0] - x1) / u[0]
            max_yt = (max_xyz[1] - y1) / u[1]
            min_yt = (min_xyz[1] - y1) / u[1]
            max_zt = (max_xyz[2] - z1) / u[2]
            min_zt = (min_xyz[2] - z1) / u[2]

            max_list = [max_xt, max_yt, max_zt]
            min_list = [min_xt, min_yt, min_zt]
            temp_list = [math.fabs(max_xt) + math.fabs(min_xt),
                             math.fabs(max_yt) + math.fabs(min_yt),
                             math.fabs(max_zt) + math.fabs(min_zt)]
            min_value = min(temp_list)
            i = temp_list.index(min_value)

            max_t = max_list[i]
            min_t = min_list[i]

            if i == 2:
                max_zx = max_xyz[0] + epsilon >= x1 + u[0] * max_t
                max_zy = max_xyz[1] + epsilon >= y1 + u[1] * max_t
                min_zx = min_xyz[0] - epsilon <= x1 + u[0] * min_t
                min_zy = min_xyz[1] - epsilon <= y1 + u[1] * min_t
                temp_count = [max_zx, max_zy, min_zx, min_zy]
            elif i == 1:
                max_yx = max_xyz[0] + epsilon >= x1 + u[0] * max_t
                max_yz = max_xyz[2] + epsilon >= z1 + u[2] * max_t
                min_yx = min_xyz[0] - epsilon <= x1 + u[0] * min_t
                min_yz = min_xyz[2] - epsilon <= z1 + u[2] * min_t
                temp_count = [max_yx, max_yz, min_yx, min_yz]
            elif i == 0:
                max_xy = max_xyz[1] + epsilon >= y1 + u[1] * max_t
                max_xz = max_xyz[2] + epsilon >= z1 + u[2] * max_t
                min_xy = min_xyz[1] - epsilon <= y1 + u[1] * min_t
                min_xz = min_xyz[2] - epsilon <= z1 + u[2] * min_t
                temp_count = [max_xy, max_xz, min_xy, min_xz]

            if temp_count.count(True) >= 4:
                count.append(True)
            else:
                count.append(False)

        if count.count(True) == 2:
            return True
        else:
            return False
    def check_distance_door(self, wall_vector, door_line):

        distance_list = list()

        for wall_vector_index in wall_vector:
            a = wall_vector_index[0][0]
            b = wall_vector_index[0][1]
            c = wall_vector_index[0][2]
            d = wall_vector_index[0][3]
            temp_list = list()

            for door_index in door_line:
                x1 = float(door_index[3])
                y1 = float(door_index[4])
                z1 = float(door_index[5])
                d = math.fabs(a*x1 + b*y1 + c*z1 + d) / math.sqrt(math.pow(a, 2)+ math.pow(b, 2)+ math.pow(c, 2))
                temp_list.append(d)
            min_d = min(temp_list)
            distance_list.append(min_d)

        true_index = distance_list.index(min(distance_list))

        return true_index


    def check_distance_point_2(self, point_list, coeff):

        for coeff_index in coeff:
            u = [float(coeff_index[0]), float(coeff_index[1]), float(coeff_index[2])]
            a = float(coeff_index[3])
            b = float(coeff_index[4])
            c = float(coeff_index[5])

            direction_vector = np.linalg.norm(u)
            count = 0
            for index in point_list:
                x = float(index[0])
                y = float(index[1])
                z = float(index[2])
                temp_vector = [x - a, y - b, z - c]
                cross_vector = np.linalg.norm(np.cross(temp_vector, u))
                point_distance = math.fabs(float(cross_vector / direction_vector))
                # 1.0
                if point_distance < 0.5:

                    count += 1
                else:
                    count += -1

            if count >= 0:
                return True
            else:
                return False

    def get_range (self, point_cloud):

        point_max = list()
        point_min = list()

        for index in np.asarray(point_cloud).T:
            point_max.append(round(max(index), 8))
            point_min.append(round(min(index), 8))

        return [point_max, point_min]

    def make_cylinder_plane(self):

        return "1"



  
    def make_straight(self, normal_vector, boundary_point, point_1, point_2, check):


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

        point_list = [point_x, point_y, point_z]


        check_result = self.check_point_range_2(point_list, boundary_point)
        # print check
        if check:
            if check_result:
                return point_list
        else:
            if check_result:
                return True
            else:
                return False

    def make_straight_2(self, normal_vector, boundary_point, point_1, point_2, check, epsilon=0.1):
        ''' Make Max Point '''
        t = Symbol('t')
        u = np.array(point_2) - np.array(point_1)
        epsilon = epsilon
        equation = Eq(normal_vector[0] * (point_1[0] + u[0] * t) +
                      normal_vector[1] * (point_1[1] + u[1] * t) +
                      normal_vector[2] * (point_1[2] + u[2] * t) +
                      normal_vector[3], 0)
        value = solve(equation, t)[0]
        point_x = point_1[0] + (u[0] * value)
        point_y = point_1[1] + (u[1] * value)
        point_z = point_1[2] + (u[2] * value)

        point_list = [point_x, point_y, point_z]

        check_result = self.check_point_range_2(point_list, boundary_point, epsilon)
        # print check
        if check:
            if check_result:
                return point_list
        else:
            if check_result:
                return True
            else:
                return False

    def get_intersection_line (self, normal_vector, point_cloud):
        x = Symbol('x')
        y = Symbol('y')
        z = 0.0
        line_list = list()
        normal_list = list()

        for index_i in range(len(normal_vector)):

            temp_line_list = list()
            temp_line_list_2 = list()
            for index_j in range(len(normal_vector)):
                point_list = list()
                if index_i != index_j:

                    point_list.append(self.get_range(point_cloud[index_i]))
                    point_list.append(self.get_range(point_cloud[index_j]))

                    temp_list = np.cross([normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]],
                                         [normal_vector[index_j][0], normal_vector[index_j][1], normal_vector[index_j][2]])
                    model_cos = np.dot([normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]],
                                       [normal_vector[index_j][0], normal_vector[index_j][1], normal_vector[index_j][2]]) \
                                / (np.linalg.norm([normal_vector[index_i][0], normal_vector[index_i][1], normal_vector[index_i][2]])
                                   * np.linalg.norm([normal_vector[index_j][0], normal_vector[index_j][1], normal_vector[index_j][2]]))
                    ''' check cos '''
                    if math.fabs(round(model_cos, 1)) != 1.0:
                        # axis = temp_list / np.linalg.norm(temp_list)
                        # print round(axis[0], 1), round(axis[1], 1), round(axis[2], 1)

                        e1 = Eq(normal_vector[index_i][0] * x + normal_vector[index_i][1] * y + normal_vector[index_i][3], 0)
                        e2 = Eq(normal_vector[index_j][0] * x + normal_vector[index_j][1] * y + normal_vector[index_j][3], 0)
                        value_eq = solve([e1, e2], x, y)
                        temp_list = temp_list.tolist()
                        temp_list.append(value_eq[x])
                        temp_list.append(value_eq[y])
                        temp_list.append(z)
                        compare_wall = point_list[1]
                        main_wall = point_list[0]
                        compare_wall_z = (compare_wall[0][2] + compare_wall[1][2])/2
                        main_wall_z = (main_wall[0][2] + main_wall[1][2])/2

                        if (main_wall[0][2] - main_wall_z) <= (compare_wall[0][2] - compare_wall_z):
                            temp_list.append(main_wall_z)
                        else:
                            temp_list.append(compare_wall_z)
                        ''' START Checking Line Range '''
                        check = self.check_distance_point_1(point_list, temp_list)
                        # print check
                        ''' End Checking Line Range'''
                        if check:

                            temp_list.append(self.get_range(point_cloud[index_i]))
                            temp_line_list.append(temp_list)
                            temp_line_list_2.append(temp_list)
    
            if len(temp_line_list) == 1:
                # print "Line 1"
                line_info = temp_line_list[0]
                u = [float(line_info[0]), float(line_info[1]), float(line_info[2])]
                v = [float(line_info[3]), float(line_info[4]), float(line_info[5])]
     
                direction_vector = np.linalg.norm(u)
                temp_normal_vector = normal_vector[index_i]
                boundary_info = line_info[len(line_info) - 1]

                min_point_low = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
                min_point_top = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[0][2])]
                min_point_right = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
                min_point_left = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]

                max_point_top = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[0][2])]
                max_point_low = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]
                max_point_right = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[0][2])]
                max_point_left = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[0][2])]
                min_list = [min_point_top, min_point_right, min_point_left]
                max_list = [max_point_low, max_point_right, max_point_left]
                max_temp_vector = np.array(max_point_top) - np.array(v)
                max_cross_vector = np.linalg.norm(np.cross(max_temp_vector, u))
                max_point_distance = float(max_cross_vector / direction_vector)

                min_temp_vector = np.array(min_point_low) - np.array(v)
                min_cross_vector = np.linalg.norm(np.cross(min_temp_vector, u))
                min_point_distance = float(min_cross_vector / direction_vector)


                if min_point_distance < max_point_distance:
                    # print "min"
                    line_1 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_top, False)
                    line_2 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_right, False)
                    line_3 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_left, False)
                    line_4 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_low, False)
                    line_5 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_right, False)
                    line_6 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_left, False)

                    min_value = [line_1, line_2, line_3]
                    max_value = [line_4, line_5, line_6]
                    result_index = (np.array(min_value) & np.array(max_value)).tolist().index(True)

                    temp_point_list = list()
                    for i in range(len(max_value)):
                        for j in range(len(min_value)):
                            if i != j:
                                if i != result_index and j != result_index:
                                    check = self.make_straight(temp_normal_vector, boundary_info, max_list[i], min_list[j], False)

                                    if check:
                                        line_point_list_1 = self.make_straight(temp_normal_vector, boundary_info, max_list[i], min_list[j], True)
                                        line_point_list_2 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_value[result_index], True)
                                        if line_point_list_1 != None:
                                            temp_point_list.append(line_point_list_1)
                                        if line_point_list_2 != None:
                                            temp_point_list.append(line_point_list_2)
                        if len(temp_point_list) == 2:
                            break

                    # u3 = np.asarray(temp_point_list[0]) - np.asarray(temp_point_list[1])
                    temp_line_info = list()
                    temp_line_info.append(u[0])
                    temp_line_info.append(u[1])
                    temp_line_info.append(u[2])
                    temp_line_info.append(temp_point_list[0][0])
                    temp_line_info.append(temp_point_list[0][1])
                    temp_line_info.append(temp_point_list[0][2])
                    temp_line_info.append((boundary_info[0][2] + boundary_info[1][2])/2)
                    temp_line_info.append(boundary_info)
                    temp_line_list.append(temp_line_info)

                elif min_point_distance > max_point_distance:
                    # print "max"
                    line_1 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_top, False)
                    line_2 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_right, False)
                    line_3 = self.make_straight(temp_normal_vector, boundary_info, min_point_low, min_point_left, False)
                    line_4 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_low, False)
                    line_5 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_right, False)
                    line_6 = self.make_straight(temp_normal_vector, boundary_info, max_point_top, max_point_left, False)
                    min_value = [line_1, line_2, line_3]
                    max_value = [line_4, line_5, line_6]
                    result_index = (np.array(min_value) & np.array(max_value)).tolist().index(True)

                    temp_point_list = list()
                    for i in range(len(min_value)):
                        for j in range(len(max_value)):
                            if i != j:
                                if i != result_index and j != result_index:

                                    check = self.make_straight(temp_normal_vector, boundary_info, min_list[i], max_list[j], False)

                                    if check:
                                        line_point_list_1 = self.make_straight(temp_normal_vector, boundary_info,
                                                                                 min_list[i], max_list[j], True)
                                        line_point_list_2 = self.make_straight(temp_normal_vector, boundary_info,
                                                                                 min_point_low, min_list[result_index], True)
                                        if line_point_list_1 != None:
                                            temp_point_list.append(line_point_list_1)
                                        if line_point_list_2 != None:
                                            temp_point_list.append(line_point_list_2)

                        if len(temp_point_list) == 2:
                            break

                    temp_line_info = list()
                    temp_line_info.append(u[0])
                    temp_line_info.append(u[1])
                    temp_line_info.append(u[2])
                    temp_line_info.append(temp_point_list[0][0])
                    temp_line_info.append(temp_point_list[0][1])
                    temp_line_info.append(temp_point_list[0][2])
                    temp_line_info.append((boundary_info[0][2] + boundary_info[1][2]) / 2)
                    temp_line_info.append(boundary_info)
                    temp_line_list.append(temp_line_info)
       
            ''' More than 2 Intersection Line'''
            if len(temp_line_list) >= 2:
                line_list.append(temp_line_list)
                normal_list.append([normal_vector[index_i], self.get_range(point_cloud[index_i])])
                # print "ok"

            ''' More than 2 Intersection Line'''
 
        return line_list, normal_list

    def make_ceiling_info(self):
        print "Make Ceiling Info"
        ceiling_vector_list, ceiling_point_list = self.sorting_Z(self.ceiling_cloud)


        '''
        saperate point

        '''

        return ceiling_vector_list, ceiling_point_list


    def make_floor_info (self):
        print "Make Floor Info"
        floor_vector_list, floor_point_list = self.sorting_Z(self.floor_cloud)

        '''
        saperate point

        '''


        return floor_vector_list, floor_point_list


    def make_wall_info (self):
        print "Make Wall Info"
        wall_vector_list, wall_point_list = self.get_normal_vector(self.wall_cloud)



        ''' START Make Intersection Line '''
        wall_intersection_line, wall_normal_list = self.get_intersection_line(wall_vector_list, wall_point_list)

        ''' END Make Intersection Line'''

        return wall_intersection_line, wall_normal_list
    def check_boundary(self, point_list, epsilon=0.5):

        main_boundary = point_list[0]
        temp_boundary = point_list[1]
        epsilon = epsilon
        check_X = 0
        check_Y = 0
        check_Z = 0
        for index in temp_boundary:
            x = index[0]
            y = index[1]
            z = index[2]
            if x <= main_boundary[0][0] + epsilon:
                if x >= main_boundary[1][0] - epsilon:
                    check_X += 1
            if y <= main_boundary[0][1] + epsilon:
                if y >= main_boundary[1][1] - epsilon:
                    check_Y += 1
            if z <= main_boundary[0][2] + epsilon:
                if z >= main_boundary[1][2] - epsilon:
                    check_Z += 1
            # print check_X+check_Y+check_Z
            if (check_X + check_Z + check_Y) == 3:
                return True
            else:
                check_X = 0
                check_Y = 0
                check_Z = 0
        return False

    def search_point_bounding(self, point_list, wall_vector_list):
        boundary_info = point_list
        min_point_low = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
        min_point_top = [float(boundary_info[1][0]), float(boundary_info[1][1]), float(boundary_info[0][2])]
        min_point_right = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[1][2])]
        min_point_left = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]

        max_point_top = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[0][2])]
        max_point_low = [float(boundary_info[0][0]), float(boundary_info[0][1]), float(boundary_info[1][2])]
        max_point_right = [float(boundary_info[1][0]), float(boundary_info[0][1]), float(boundary_info[0][2])]
        max_point_left = [float(boundary_info[0][0]), float(boundary_info[1][1]), float(boundary_info[0][2])]
        min_list = [min_point_top, min_point_right, min_point_left]
        max_list = [max_point_low, max_point_right, max_point_left]
        bounding_box_line_list = list()
        ''' min '''
        for min_i in min_list:
            u = np.array(min_i) - np.array(min_point_low)
            x = min_point_low[0]
            y = min_point_low[1]
            z = min_point_low[2]
            temp_list = u.tolist()
            temp_list.append(x)
            temp_list.append(y)
            temp_list.append(z)
            bounding_box_line_list.append(temp_list)
        ''' max '''
        for max_i in max_list:
            u = np.array(max_i) - np.array(max_point_top)
            x = max_point_top[0]
            y = max_point_top[1]
            z = max_point_top[2]
            temp_list = u.tolist()
            temp_list.append(x)
            temp_list.append(y)
            temp_list.append(z)
            bounding_box_line_list.append(temp_list)
        ''' else '''
        for min_i in min_list:
            for max_i in max_list:
                if min_list.index(min_i) != max_list.index(max_i):
                    u = np.array(max_i) - np.array(min_i)
                    x = min_i[0]
                    y = min_i[1]
                    z = min_i[2]
                    temp_list = u.tolist()
                    temp_list.append(x)
                    temp_list.append(y)
                    temp_list.append(z)
                    bounding_box_line_list.append(temp_list)


        # door_to_wall = list()
        # door_point_list = list()
        for wall_index in wall_vector_list:

            door_point_list = list()
            door_to_wall = list()
            for bounding_index in bounding_box_line_list:

                t = Symbol('t')
                default_vector = [wall_index[0][1], wall_index[0][2], wall_index[0][3]]
                model_value = [bounding_index[0], bounding_index[1], bounding_index[2]]
                model_cos = np.dot(default_vector, model_value) / (np.linalg.norm(model_value) * np.linalg.norm(default_vector))
                if round(math.fabs(model_cos), 2) < 0.9:
                    # epsilon = epsilon
                    equation = Eq(wall_index[0][0] * (bounding_index[3] + (bounding_index[0] * t)) +
                                  wall_index[0][1] * (bounding_index[4] + (bounding_index[1] * t)) +
                                  wall_index[0][2] * (bounding_index[5] + (bounding_index[2] * t)) +
                                  wall_index[0][3], 0)
                    value = solve(equation, t)[0]
                    point_x = bounding_index[3] + (bounding_index[0] * value)
                    point_y = bounding_index[4] + (bounding_index[1] * value)
                    point_z = bounding_index[5] + (bounding_index[2] * value)

                    point_list_1 = [point_x, point_y, point_z]
                    check = self.check_point_range_2(point_list_1, boundary_info, 0.15)
                    # print check
                    if check is True:

                        door_point_list.append(point_list_1)
                        door_to_wall.append(wall_vector_list.index(wall_index))
            if len(door_to_wall) != 0:
                if door_to_wall.count(door_to_wall[0]) >= len(door_to_wall) - 1:
                    door_point_list.append(door_to_wall[0])
            if len(door_point_list) >= 4:

                return door_point_list

        # return True

    def make_door_info (self, wall_normal_vector):
        print "Make Door Info"

        door_vector_list, door_point_list = self.get_normal_vector(self.door_cloud, 0.85, 200, 500)
        wall_normal_vector = wall_normal_vector

        # self.visual_viewer(door_point_list)
        temp_list_a = list()
        temp_list_b = list()
        for index in door_point_list:
            temp_list_a.append(index)
            temp_list_b.append(index)
        # temp_list_a.sort(reverse=True)

        count = 0
        door_repoint_list = list()
        door_revector_list = list()

        for i in range(len(temp_list_a)):

            temp_list_c = list()
            temp_list_d = list()
            if temp_list_b[i] is not True:
                temp_list_c.append(temp_list_a[i])
                temp_list_d.append(door_vector_list[i])
                for j in range(len(temp_list_a)):
                    temp_list_b.pop(i)
                    temp_list_b.insert(i, True)
                    if i != j and temp_list_b[j] != True:
                        point_list = list()
                        point_list.append(self.get_range(temp_list_a[i]))
                        point_list.append(self.get_range(temp_list_a[j]))
                        check = self.check_boundary(point_list)

                        if check:
                            temp_list_c.append(temp_list_a[j])
                            temp_list_d.append(door_vector_list[j])
                            temp_list_b.pop(j)
                            temp_list_b.insert(j, True)

            if len(temp_list_c) > 2:
                for k in range(len(temp_list_c) - 1):
                    for l in range(len(temp_list_a)):
                        if temp_list_b[l] != True:
                            point_list = list()
                            point_list.append(self.get_range(temp_list_c[k+1]))
                            point_list.append(self.get_range(temp_list_a[l]))
                            check = self.check_boundary(point_list)
                            if check:
                                temp_list_c.append(temp_list_a[l])
                                temp_list_d.append(door_vector_list[l])
                                temp_list_b.pop(l)
                                temp_list_b.insert(l, True)

            count += len(temp_list_c)
            door_repoint_list.append(temp_list_c)
            door_revector_list.append(temp_list_d)
            if count == len(temp_list_a):
                break


        door_repoint_list_2 = list()
        door_revector_list_2 = list()


        for door_repoint_i in range(len(door_repoint_list)):
            temp_repoint_list = list()
            temp_revector_list = list()
            if len(door_repoint_list[door_repoint_i]) > 2:
                temp_list_a = list()
                temp_list_b = list()
                for index in door_repoint_list[door_repoint_i]:
                    temp_list_a.append(index.size)
                    temp_list_b.append(index.size)
                temp_list_b.sort(reverse=True)

                if temp_list_b[0] >= reduce(lambda x, y: x + y, temp_list_b[1:]):
                    temp_repoint_list.append(door_repoint_list[door_repoint_i][temp_list_a.index(temp_list_b[0])])
                    temp_revector_list.append(door_revector_list[door_repoint_i][temp_list_a.index(temp_list_b[0])])

                    temp_pcl_list = list()
                    one_time = True
                    default_vector = [0, 0, -1]
                    model = door_revector_list[door_repoint_i][0]
                    model_value = [model[0], model[1], model[2]]
                    model_cos = np.dot(default_vector, model_value) / (np.linalg.norm(model_value) * np.linalg.norm(default_vector))

                    temp_model_list = list()
                    temp_cos_list = list()
                    for index in temp_list_b[1:]:

                        if one_time:
                            main_i = temp_list_a.index(index)
                            temp_pcl_list = door_repoint_list[door_repoint_i][main_i].to_list()
                            model = door_revector_list[door_repoint_i][main_i]
                            model_value_1 = [model[0], model[1], model[2]]
                            temp_model_cos = np.dot(default_vector, model_value_1) / (
                                    np.linalg.norm(model_value_1) * np.linalg.norm(default_vector))
                            temp_model_list.append(model)
                            temp_cos_list.append(temp_model_cos)
                            one_time = False

                        else:
                            main_i = temp_list_a.index(index)
                            temp_pcl_list += door_repoint_list[door_repoint_i][main_i].to_list()
                            model = door_revector_list[door_repoint_i][main_i]
                            model_value_1 = [model[0], model[1], model[2]]
                            model_cos_1 = np.dot(default_vector, model_value_1) / (
                                    np.linalg.norm(model_value_1) * np.linalg.norm(default_vector))
                            temp_model_list.append(model)
                            temp_cos_list.append(model_cos_1)

                            if round(math.fabs(model_cos_1), 2) <= 0.8:
                                if model_cos_1 <= temp_model_cos:
                                    temp_model_cos = model_cos_1


                    temp_pcl = pcl.PointCloud(temp_pcl_list)
                    temp_repoint_list.append(temp_pcl)
                    temp_revector_list.append(temp_model_list[temp_cos_list.index(temp_model_cos)])
                door_repoint_list_2.append(temp_repoint_list)
                door_revector_list_2.append(temp_revector_list)

            else:

                temp_list_a = list()
                temp_list_b = list()
                for index in door_repoint_list[door_repoint_i]:
                    temp_list_a.append(index.size)
                    temp_list_b.append(index.size)
                temp_list_a.sort(reverse=True)
                for index in temp_list_a:
                    temp_repoint_list.append(door_repoint_list[door_repoint_i][temp_list_b.index(index)])
                    temp_revector_list.append(door_revector_list[door_repoint_i][temp_list_b.index(index)])

                door_repoint_list_2.append(temp_repoint_list)
                door_revector_list_2.append(temp_revector_list)
        '''new'''
        door_point_list = list()
        for door_index in door_repoint_list_2:

            if len(door_index) == 2:
               
                a = self.search_point_bounding(self.get_range(door_index[0]), wall_normal_vector)
                b = self.search_point_bounding(self.get_range(door_index[1]), wall_normal_vector)
                a_1 = a[:len(a) - 1]
                b_1 = b[:len(b) - 1]
                point_sort2 = ps.Point_sort()
                a_1 = point_sort2.SortPointsClockwise2(a_1, True)
                b_1 = point_sort2.SortPointsClockwise2(b_1, True)
                a_vector = np.cross(np.array(a_1[1]) - np.array(a_1[0]), np.array(a_1[2]) - np.array(a_1[1]))
                b_vector = np.cross(np.array(b_1[1]) - np.array(b_1[0]), np.array(b_1[2]) - np.array(b_1[1]))
                a_width = math.sqrt(math.pow(a_vector[0], 2) + math.pow(a_vector[1], 2) + math.pow(a_vector[2], 2))
                b_width = math.sqrt(math.pow(b_vector[0], 2) + math.pow(b_vector[1], 2) + math.pow(b_vector[2], 2))
                if a_width > b_width:
                    door_point_list.append(a)
                else:
                    door_point_list.append(b)
            else:
                door_point_list.append(self.search_point_bounding(self.get_range(door_index[0]), wall_normal_vector))

        return door_point_list
    

    def make_window_info (self, wall_normal_vector):
        print "Make Window Info"
        window_vector_list, window_point_list = self.get_normal_vector(self.window_cloud, 0.85, 200, 500)
        wall_normal_vector = wall_normal_vector

        temp_list_a = list()
        temp_list_b = list()
        for index in window_point_list:
            temp_list_a.append(index)
            temp_list_b.append(index)
        # temp_list_a.sort(reverse=True)

        count = 0
        window_repoint_list = list()
        window_revector_list = list()
        for i in range(len(temp_list_a)):

            temp_list_c = list()
            temp_list_d = list()
            if temp_list_b[i] is not True:
                temp_list_c.append(temp_list_a[i])
                temp_list_d.append(window_vector_list[i])
                for j in range(len(temp_list_a)):
                    temp_list_b.pop(i)
                    temp_list_b.insert(i, True)
                    if i != j and temp_list_b[j] != True:
                        point_list = list()
                        point_list.append(self.get_range(temp_list_a[i]))
                        point_list.append(self.get_range(temp_list_a[j]))
                        check = self.check_boundary(point_list)
                        if check:
                            temp_list_c.append(temp_list_a[j])
                            temp_list_d.append(window_vector_list[j])
                            temp_list_b.pop(j)
                            temp_list_b.insert(j, True)

            if len(temp_list_c) > 2:
                for k in range(len(temp_list_c) - 1):
                    for l in range(len(temp_list_a)):
                        if temp_list_b[l] != True:
                            point_list = list()
                            point_list.append(self.get_range(temp_list_c[k + 1]))
                            point_list.append(self.get_range(temp_list_a[l]))
                            check = self.check_boundary(point_list)
                            if check:
                                temp_list_c.append(temp_list_a[l])
                                temp_list_d.append(window_revector_list[l])
                                temp_list_b.pop(l)
                                temp_list_b.insert(l, True)
            count += len(temp_list_c)
            window_repoint_list.append(temp_list_c)
            window_revector_list.append(temp_list_d)
            if count == len(temp_list_a):
                break

        window_repoint_list_2 = list()
        window_revector_list_2 = list()

        for window_repoint_i in range(len(window_repoint_list)):
            temp_repoint_list = list()
            temp_revector_list = list()
            if len(window_repoint_list[window_repoint_i]) > 2:
                temp_list_a = list()
                temp_list_b = list()
                for index in window_repoint_list[window_repoint_i]:
                    temp_list_a.append(index.size)
                    temp_list_b.append(index.size)
                temp_list_b.sort(reverse=True)

                if temp_list_b[0] >= reduce(lambda x, y: x + y, temp_list_b[1:]):
                    temp_repoint_list.append(window_repoint_list[window_repoint_i][temp_list_a.index(temp_list_b[0])])
                    temp_revector_list.append(window_revector_list[window_repoint_i][temp_list_a.index(temp_list_b[0])])

                    temp_pcl_list = list()
                    one_time = True
                    default_vector = [0, 0, -1]
                    model = window_revector_list[window_repoint_i][0]
                    model_value = [model[0], model[1], model[2]]
                    model_cos = np.dot(default_vector, model_value) / (
                                np.linalg.norm(model_value) * np.linalg.norm(default_vector))

                    temp_model_list = list()
                    temp_cos_list = list()
                    for index in temp_list_b[1:]:

                        if one_time:
                            main_i = temp_list_a.index(index)
                            temp_pcl_list = window_repoint_list[window_repoint_i][main_i].to_list()
                            model = window_revector_list[window_repoint_i][main_i]
                            model_value_1 = [model[0], model[1], model[2]]
                            temp_model_cos = np.dot(default_vector, model_value_1) / (
                                    np.linalg.norm(model_value_1) * np.linalg.norm(default_vector))
                            temp_model_list.append(model)
                            temp_cos_list.append(temp_model_cos)
                            one_time = False

                        else:
                            main_i = temp_list_a.index(index)
                            temp_pcl_list += window_repoint_list[window_repoint_i][main_i].to_list()
                            model = window_revector_list[window_repoint_i][main_i]
                            model_value_1 = [model[0], model[1], model[2]]
                            model_cos_1 = np.dot(default_vector, model_value_1) / (
                                    np.linalg.norm(model_value_1) * np.linalg.norm(default_vector))
                            temp_model_list.append(model)
                            temp_cos_list.append(model_cos_1)

                            if round(math.fabs(model_cos_1), 2) <= 0.8:
                                if model_cos_1 <= temp_model_cos:
                                    temp_model_cos = model_cos_1

                    temp_pcl = pcl.PointCloud(temp_pcl_list)
                    temp_repoint_list.append(temp_pcl)
                    temp_revector_list.append(temp_model_list[temp_cos_list.index(temp_model_cos)])
                window_repoint_list_2.append(temp_repoint_list)
                window_revector_list_2.append(temp_revector_list)

            else:
                temp_list_a = list()
                temp_list_b = list()
                for index in window_repoint_list[window_repoint_i]:
                    temp_list_a.append(index.size)
                    temp_list_b.append(index.size)
                temp_list_a.sort(reverse=True)
                for index in temp_list_a:
                    temp_repoint_list.append(window_repoint_list[window_repoint_i][temp_list_b.index(index)])
                    temp_revector_list.append(window_revector_list[window_repoint_i][temp_list_b.index(index)])

                window_repoint_list_2.append(temp_repoint_list)
                window_revector_list_2.append(temp_revector_list)
        '''new'''
        window_point_list = list()
        for window_index in window_repoint_list_2:

            if len(window_index) == 2:
                # window_point_list.append(self.search_point_bounding(self.get_range(door_index[1]), wall_normal_vector))
                a = self.search_point_bounding(self.get_range(window_index[0]), wall_normal_vector)
                b = self.search_point_bounding(self.get_range(window_index[1]), wall_normal_vector)
                a_1 = a[:len(a) - 1]
                b_1 = b[:len(b) - 1]
                a_1 = ps.SortPointsClockwise2(a_1, True)
                b_1 = ps.SortPointsClockwise2(b_1, True)
                a_vector = np.cross(np.array(a_1[1]) - np.array(a_1[0]), np.array(a_1[2]) - np.array(a_1[1]))
                b_vector = np.cross(np.array(b_1[1]) - np.array(b_1[0]), np.array(b_1[2]) - np.array(b_1[1]))
                a_width = math.sqrt(math.pow(a_vector[0], 2) + math.pow(a_vector[1], 2) + math.pow(a_vector[2], 2))
                b_width = math.sqrt(math.pow(b_vector[0], 2) + math.pow(b_vector[1], 2) + math.pow(b_vector[2], 2))
                if a_width > b_width:
                    window_point_list.append(a)
                else:
                    window_point_list.append(b)
            else:
                window_point_list.append(self.search_point_bounding(self.get_range(window_index[0]), wall_normal_vector))

        return window_point_list


        ''' Make Concave Hull '''

        # return window_vector_list

    def make_point_surface (self):
        print "Make Point Surface"
        ceiling_vector_list, ceiling_point_list = self.make_ceiling_info()
        floor_vector_list, floor_point_list = self.make_floor_info()
        wall_interesction_line, wall_normal_vector = self.make_wall_info()
        door_surface_list = self.make_door_info(wall_normal_vector)
        window_surface_list = self.make_window_info(wall_normal_vector)
        print "door test start"


        #     print len(index)
        ceiling_test_list = list()
        floor_test_list = list()

        print "Ceiling Count : " + str(len(ceiling_point_list))
        for i in range(len(ceiling_vector_list)):
            ceiling_range = self.get_range(ceiling_point_list[i])
            temp_list = list()
            if len(ceiling_vector_list[i]) > 4:
                max = ceiling_range[0]
                min = ceiling_range[1]

                p1 = [float(min[0]), float(min[1]), float(min[2])]
                p2 = [float(min[0]), float(max[1]), float(min[2])]
                p3 = [float(max[0]), float(max[1]), float(min[2])]

                p2p1 = [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]]
                p2p3 = [p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]]

                n = np.cross(p2p1, p2p3)
                d = n[0] * (p2[0] * -1.0) + n[1] * (p2[1] * -1.0) + n[2] * (p2[2] * -1.0)
                n = n.tolist()
                n.append(d)
                temp_list.append(n)
                temp_list.append(ceiling_range)
                ceiling_test_list.append(temp_list)
            else:
                # temp_list.append(ceiling_vector_list[i])
                temp_list.append(ceiling_vector_list[i])
                temp_list.append(ceiling_range)
                ceiling_test_list.append(temp_list)

        print "Floor Count : " + str(len(floor_point_list))
        for i in range(len(floor_vector_list)):
            floor_range = self.get_range(floor_point_list[i])
            temp_list = list()
            if len(floor_vector_list[i]) > 4:
                max = floor_range[0]
                min = floor_range[1]

                p1 = [float(min[0]), float(min[1]), float(min[2])]
                p2 = [float(min[0]), float(max[1]), float(min[2])]
                p3 = [float(max[0]), float(max[1]), float(min[2])]

                p2p1 = [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]]
                p2p3 = [p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]]

                n = np.cross(p2p1, p2p3)
                d = n[0] * (p2[0] * -1.0) + n[1] * (p2[1] * -1.0) + n[2] * (p2[2] * -1.0)
                n = n.tolist()
                n.append(d)
                temp_list.append(n)
                temp_list.append(floor_range)
                floor_test_list.append(temp_list)
            else:
                # temp_list.append(ceiling_vector_list[i])
                temp_list.append(floor_vector_list[i])
                temp_list.append(floor_range)
                floor_test_list.append(temp_list)    


        up_down_surface_list = ceiling_test_list + floor_test_list
        wall_surface_list = list()

        cf_surface_list = list()
        for i in range(len(up_down_surface_list)):
            cf_surface_list.append([])

        for wall_interesction_index in wall_interesction_line:
            wall_surface_point = list()
            for index in wall_interesction_index:

                zero_a = index[0]
                zero_b = index[1]
                zero_c = index[2]
                zero_x = index[3]
                zero_y = index[4]
                zero_z = index[5]

                middle_z = index[len(index) - 2]
                middle_list = list()
                temp_middle_list_1 = list()
                temp_middle_list_2 = list()
                surface_width_list_c = list()
                surface_width_list_f = list()
                temp_width_list_1 = list()
                temp_width_list_2 = list()
                each_line_zero = list()

                for index_surface in ceiling_test_list:
                    surface_boundary = index_surface[len(index_surface) - 1]
                    surface_width = (surface_boundary[0][0] - surface_boundary[1][0]) * (surface_boundary[0][1] - surface_boundary[1][1])
                    max_z = surface_boundary[0][2]
                    min_z = surface_boundary[1][2]
                    middle_list.append((max_z + min_z) / 2)
                    temp_middle_list_1.append((max_z + min_z) / 2)
                    surface_width_list_c.append(surface_width)
                    temp_width_list_1.append(surface_width)

                temp_width_list_1.sort(reverse=False)
                ceiling_max = temp_width_list_1.pop()
                ceiling_max_i = surface_width_list_c.index(ceiling_max)
                temp_middle_list_1.pop(ceiling_max_i)

                for index_surface in floor_test_list:
                    surface_boundary = index_surface[len(index_surface) - 1]
                    surface_width = (surface_boundary[0][0] - surface_boundary[1][0]) * (
                                surface_boundary[0][1] - surface_boundary[1][1])
                    max_z = surface_boundary[0][2]
                    min_z = surface_boundary[1][2]
                    middle_list.append((max_z + min_z) / 2)
                    temp_middle_list_2.append((max_z + min_z) / 2)
                    surface_width_list_f.append(surface_width)
                    temp_width_list_2.append(surface_width)

                temp_width_list_2.sort(reverse=False)
                floor_max = temp_width_list_2.pop()
                floor_max_i = surface_width_list_f.index(floor_max)
                temp_middle_list_2.pop(floor_max_i)


                all_middle_list = temp_middle_list_1 + temp_middle_list_2

                surface_width_list = surface_width_list_c + surface_width_list_f
                all_middle_list.append(middle_z)
                all_middle_list.sort(reverse=True)

                ''' large -> small '''
                ''' model_list, surface_width_list '''

                i = all_middle_list.index(middle_z)
                up_count = 0
                down_count = 0

                up_list = all_middle_list[:i]
                down_list = all_middle_list[i+1:]

                for up_z in up_list:
                    middle_up_z = up_z
                    up_i = middle_list.index(middle_up_z)
                    up_surface = up_down_surface_list[up_i]

                    zero_t1 = (middle_up_z - zero_z) / zero_c
                    zero_x1 = zero_a * zero_t1 + zero_x
                    zero_y1 = zero_b * zero_t1 + zero_y


                    check_up = self.check_point_range_3([zero_x1, zero_y1, middle_up_z], up_surface[len(up_surface) - 1])

                    if check_up is True:
                        up_count += 1
                        each_line_zero.append([zero_x1, zero_y1, middle_up_z])
                        cf_surface_list[up_i].append([zero_x1, zero_y1, middle_up_z])

                for bottom_z in down_list:
                    middle_bottom_z = bottom_z
                    bottom_i = middle_list.index(middle_bottom_z)
                    bottom_surface = up_down_surface_list[bottom_i]
                    zero_t2 = (middle_bottom_z - zero_z) / zero_c
                    zero_x2 = zero_a * zero_t2 + zero_x
                    zero_y2 = zero_b * zero_t2 + zero_y

                    check_bottom = self.check_point_range_3([zero_x2, zero_y2, middle_bottom_z], bottom_surface[len(bottom_surface) - 1])
                    if check_bottom is True:
                        down_count += 1
                        each_line_zero.append([zero_x2, zero_y2, middle_bottom_z])
                        cf_surface_list[bottom_i].append([zero_x2, zero_y2, middle_bottom_z])

                if up_count + down_count is 0:

                    up_width_i = surface_width_list.index(ceiling_max)
                    middle_up_z = middle_list[up_width_i]
                    zero_t1 = (middle_up_z - zero_z) / zero_c
                    zero_x1 = zero_a * zero_t1 + zero_x
                    zero_y1 = zero_b * zero_t1 + zero_y
                    each_line_zero.append([zero_x1, zero_y1, middle_up_z])
                    cf_surface_list[up_width_i].append([zero_x1, zero_y1, middle_up_z])

                    bottom_width_i = surface_width_list.index(floor_max)
                    middle_bottom_z = middle_list[bottom_width_i]
                    zero_t2 = (middle_bottom_z - zero_z) / zero_c
                    zero_x2 = zero_a * zero_t2 + zero_x
                    zero_y2 = zero_b * zero_t2 + zero_y
                    each_line_zero.append([zero_x2, zero_y2, middle_bottom_z])
                    cf_surface_list[bottom_width_i].append([zero_x2, zero_y2, middle_bottom_z])
                elif up_count + down_count is 1:
                    ceiling_z = middle_list[surface_width_list.index(ceiling_max)]
                    floor_z = middle_list[surface_width_list.index(floor_max)]
                    up_width_i = surface_width_list.index(ceiling_max)
                    bottom_width_i = surface_width_list.index(floor_max)
                    if up_count is 0:
                        temp_list_c = [middle_z, ceiling_z, floor_z]
                        temp_list_c.sort(reverse=True)
                        temp_i = temp_list_c.index(middle_z)
                        temp_max = temp_list_c[temp_i - 1]
                        middle_up_z = temp_max
                        zero_t1 = (middle_up_z - zero_z) / zero_c
                        zero_x1 = zero_a * zero_t1 + zero_x
                        zero_y1 = zero_b * zero_t1 + zero_y
                        each_line_zero.append([zero_x1, zero_y1, middle_up_z])
                        cf_surface_list[up_width_i].append([zero_x1, zero_y1, middle_up_z])

                    if down_count is 0:
                        temp_list_f = [middle_z, ceiling_z, floor_z]
                        temp_list_f.sort(reverse=True)
                        temp_i = temp_list_f.index(middle_z)
                        temp_max = temp_list_f[temp_i + 1]
                        middle_bottom_z = temp_max
                        zero_t2 = (middle_bottom_z - zero_z) / zero_c
                        zero_x2 = zero_a * zero_t2 + zero_x
                        zero_y2 = zero_b * zero_t2 + zero_y
                        each_line_zero.append([zero_x2, zero_y2, middle_bottom_z])
                        cf_surface_list[bottom_width_i].append([zero_x2, zero_y2, middle_bottom_z])
                for index_zero in each_line_zero:
                    wall_surface_point.append(index_zero)

            

            print "Wall Surface point Count : " + str(len(wall_surface_point))
            wall_surface_list.append(wall_surface_point)


        print "Wall Surface point Count : " + str(len(wall_surface_list))
        ''' Extend the wall range using door range '''
        print "Door Info"
        print door_surface_list
        print "Window Info"
        print window_surface_list
      
        for door_info_index in door_surface_list:


            a = wall_surface_list[door_info_index[len(door_info_index)-1:][0]]
            b = wall_normal_vector[door_info_index[len(door_info_index)-1:][0]]
            normal_vector_1 = b[0]
            c = door_info_index[:len(door_info_index)-1]
            default_vector = [0, 0, -1]
            model_value = [normal_vector_1[0], normal_vector_1[1], normal_vector_1[2]]
            model_cos = np.dot(default_vector, model_value) / (np.linalg.norm(model_value) * np.linalg.norm(default_vector))
            axis = np.cross(default_vector, model_value) / np.linalg.norm(np.cross(default_vector, model_value))

            u_x = axis[0]
            u_y = axis[1]
            u_z = axis[2]

            model_sin = math.sqrt(1 - math.pow(model_cos, 2))
            model_cos = math.fabs(model_cos)
            model_cos2 = 1 - model_cos
            point_matrix = np.matrix([[model_cos2 * u_x * u_x + model_cos,
                                      model_cos2 * u_x * u_y - model_sin * u_z,
                                      model_cos2 * u_x * u_z + model_sin * u_y],

                                     [model_cos2 * u_y * u_x + model_sin * u_z,
                                      model_cos2 * u_y * u_y + model_cos,
                                      model_cos2 * u_y * u_z - model_sin * u_x],

                                     [model_cos2 * u_z * u_x - model_sin * u_y,
                                      model_cos2 * u_z * u_y + model_sin * u_x,
                                      model_cos2 * u_z * u_z + model_cos]])

            temp_list_1 = list()
            temp_list_2 = list()
            change_xyz = list()

            for a_i in a:
                temp_point = np.asmatrix([[a_i[0]], [a_i[1]], [a_i[2]]])
                temp_list_1.append(np.dot(point_matrix, temp_point).T.tolist()[0])
            for c_i in c:
                temp_point = np.asmatrix([[c_i[0]], [c_i[1]], [c_i[2]]])
                temp_list_2.append(np.dot(point_matrix, temp_point).T.tolist()[0])
            wall_surface_range = self.get_range(temp_list_1)
            for temp_2_i in temp_list_2:

                check = self.check_point_2Drange(temp_2_i, wall_surface_range)

                if check is not True:
                    door_i = temp_list_2.index(temp_2_i)
                    change_xyz.append([door_info_index[door_i][0], door_info_index[door_i][1], door_info_index[door_i][2]])

            if len(change_xyz) != 0:
                change_wall_range = self.get_range(wall_surface_list[door_info_index[len(door_info_index) - 1:][0]])
                for change_index in change_xyz:
                    x = change_index[0]
                    y = change_index[1]
                    z = change_index[2]
                    if math.fabs(change_wall_range[0][2] - z) >= math.fabs(change_wall_range[1][2] - z):
                        z = change_wall_range[1][2]
                    else:
                        z = change_wall_range[0][2]
                    wall_surface_list[door_info_index[len(door_info_index) - 1:][0]].append([x, y, z])
        for window_info_index in window_surface_list:

            a = wall_surface_list[window_info_index[len(window_info_index) - 1:][0]]
            b = wall_normal_vector[window_info_index[len(window_info_index) - 1:][0]]
            normal_vector_1 = b[0]
            c = window_info_index[:len(window_info_index) - 1]
            default_vector = [0, 0, -1]
            model_value = [normal_vector_1[0], normal_vector_1[1], normal_vector_1[2]]
            model_cos = np.dot(default_vector, model_value) / (
                        np.linalg.norm(model_value) * np.linalg.norm(default_vector))
            axis = np.cross(default_vector, model_value) / np.linalg.norm(np.cross(default_vector, model_value))

            u_x = axis[0]
            u_y = axis[1]
            u_z = axis[2]

            model_sin = math.sqrt(1 - math.pow(model_cos, 2))
            model_cos = math.fabs(model_cos)
            model_cos2 = 1 - model_cos
            point_matrix = np.matrix([[model_cos2 * u_x * u_x + model_cos,
                                       model_cos2 * u_x * u_y - model_sin * u_z,
                                       model_cos2 * u_x * u_z + model_sin * u_y],

                                      [model_cos2 * u_y * u_x + model_sin * u_z,
                                       model_cos2 * u_y * u_y + model_cos,
                                       model_cos2 * u_y * u_z - model_sin * u_x],

                                      [model_cos2 * u_z * u_x - model_sin * u_y,
                                       model_cos2 * u_z * u_y + model_sin * u_x,
                                       model_cos2 * u_z * u_z + model_cos]])

            temp_list_1 = list()
            temp_list_2 = list()
            change_xyz = list()

            for a_i in a:
                temp_point = np.asmatrix([[a_i[0]], [a_i[1]], [a_i[2]]])
                temp_list_1.append(np.dot(point_matrix, temp_point).T.tolist()[0])
            for c_i in c:
                temp_point = np.asmatrix([[c_i[0]], [c_i[1]], [c_i[2]]])
                temp_list_2.append(np.dot(point_matrix, temp_point).T.tolist()[0])
            wall_surface_range = self.get_range(temp_list_1)


            for temp_2_i in temp_list_2:

                check = self.check_point_2Drange(temp_2_i, wall_surface_range)

                if check is not True:
                    door_i = temp_list_2.index(temp_2_i)
                    change_xyz.append(
                        [window_info_index[door_i][0], window_info_index[door_i][1], window_info_index[door_i][2]])

            if len(change_xyz) != 0:
                change_wall_range = self.get_range(wall_surface_list[window_info_index[len(window_info_index) - 1:][0]])
                for change_index in change_xyz:
                    x = change_index[0]
                    y = change_index[1]
                    z = change_index[2]
                    if math.fabs(change_wall_range[0][2] - z) >= math.fabs(change_wall_range[1][2] - z):
                        z = change_wall_range[1][2]
                    else:
                        z = change_wall_range[0][2]
                    wall_surface_list[window_info_index[len(window_info_index) - 1:][0]].append([x, y, z])

        ceiling_surface_list = cf_surface_list[:len(ceiling_test_list)]
        floor_surface_list = cf_surface_list[len(ceiling_test_list):]

        return self.clockwise_sort(wall_surface_list, ceiling_surface_list, floor_surface_list, door_surface_list, window_surface_list)

    def check_point_2Drange(self, point, wall_range, epsilon=0.05):
        wall_range_max = wall_range[0]
        wall_range_min = wall_range[1]
        x = point[0]
        y = point[1]
        z = point[2]
        check_X = 0
        check_Y = 0
        check_Z = 0
        if x <= wall_range_max[0] + epsilon:
            if x >= wall_range_min[0] - epsilon:
                check_X += 1
        if y <= wall_range_max[1] + epsilon:
            if y >= wall_range_min[1] - epsilon:
                check_Y += 1
        if z <= wall_range_max[1] + epsilon:
            if z >= wall_range_min[1] - epsilon:
                check_Z += 1
        if (check_X + check_Y) == 2:
            return True
        else:
            return False

    def check_point_range_2(self, point, wall_range, epsilon=0.05):

        epsilon = epsilon
        wall_range_max = wall_range[0]
        wall_range_min = wall_range[1]

        x = point[0]
        y = point[1]
        z = point[2]
        check_X = 0
        check_Z = 0
        check_Y = 0

        if x <= wall_range_max[0] + epsilon:
            if x >= wall_range_min[0] - epsilon:
                check_X += 1
        if y <= wall_range_max[1] + epsilon:
            if y >= wall_range_min[1] - epsilon:
                check_Y += 1
        if z <= wall_range_max[2] + epsilon:
            if z >= wall_range_min[2] - epsilon:
                check_Z += 1

        if (check_X + check_Z + check_Y) == 3:
            return True
        else:
            return False

    def check_point_range_3(self, point, wall_range):

        epsilon = 0.1
        wall_range_max = wall_range[0]
        wall_range_min = wall_range[1]

        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        check_X = 0
        check_Y = 0
        check_Z = 0

        if x <= wall_range_max[0] + epsilon:
            if x >= wall_range_min[0] - epsilon:
                check_X += 1
        if y <= wall_range_max[1] + epsilon:
            if y >= wall_range_min[1] - epsilon:
                check_Y += 1
        if z <= wall_range_max[2] + epsilon:
            if z >= wall_range_min[2] - epsilon:
                check_Z += 1


        # if (check_X + check_Y + check_Z) == 3:
        if (check_X + check_Y) == 2:
            return True
        else:
            return False

    def check_point_range(self, point, wall_range, temp_range):

        epsilon = 0.1
        wall_range_max = wall_range[0]
        wall_range_min = wall_range[1]
        temp_range_max = temp_range[0]
        temp_range_min = temp_range[1]
        x = point[0]
        y = point[1]
        z = point[2]
        check_temp_X = 0
        check_temp_Y = 0
        check_temp_Z = 0
        check_wall_X = 0
        check_wall_Y = 0
        check_wall_Z = 0


        if x <= temp_range_max[0] + epsilon:
            if x >= temp_range_min[0] - epsilon:
                check_temp_X += 1
        if y <= temp_range_max[1] + epsilon:
            if y >= temp_range_min[1] - epsilon:
                check_temp_Y += 1
        if z <= temp_range_max[2] + epsilon:
            if z >= temp_range_min[2] - epsilon:
                check_temp_Z += 1

        if x <= wall_range_max[0] + epsilon:
            if x >= wall_range_min[0] - epsilon:
                check_wall_X += 1
        if y <= wall_range_max[1] + epsilon:
            if y >= wall_range_min[1] - epsilon:
                check_wall_Y += 1
        if z <= wall_range_max[2] + epsilon:
            if z >= wall_range_min[2] - epsilon:
                check_wall_Z += 1

        result = check_temp_X + check_temp_Y + check_temp_Z + check_wall_X + check_wall_Y + check_wall_Z
        if result == 6:
            return True

        else:
            return False


    def visual_viewer(self, cloud_list):


        viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

        viewer.SetBackgroundColor(0, 0, 0)

        viewer.AddPointCloud(self.wall_cloud, b'sample cloud')
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')
        viewer.AddPointCloud(self.ceiling_cloud, b'sample cloud1')
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud1')


        for index in range(len(cloud_list)):

            r = random.randrange(0, 256)
            b = random.randrange(0, 256)
            g = random.randrange(0, 256)
  

            point_size = 10
            color_handle = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_list[index], r, b, g)
            id = b'inliers_cloud_' + str(index)
            viewer.AddPointCloud_ColorHandler(cloud_list[index], color_handle, id)
            viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, point_size, id)
    
        viewer.InitCameraParameters()
        while viewer.WasStopped() != True:
            viewer.SpinOnce(100)

    def visual_graph(self, point_list):



        x = []
        y = []
        # print len(e)
        for index in point_list:
            x.append(index[0])
            y.append(index[2])
        plt.scatter(x, y, label="stars", color="green",
                    marker="*", s=50)
        plt.plot(x, y)
        plt.legend()
        plt.show()


    def delete_overlapping_data(self, node_list):

        for index in node_list:
            if node_list.count(index) != 1:
                node_list.pop(node_list.index(index))

        return node_list
    def clockwise_sort(self, wall_surface_list, ceiling_surface_list, floor_surface_list, door_surface_list, window_surface_list):
        wall_info = list()
        ceiling_info = list()
        floor_info = list()
        door_info = list()
        window_info = list()
        point_sort = ps.Point_sort()
        for index in wall_surface_list:
            a = point_sort.SortPointsClockwise(index, True)

            wall_info.append(a)

        for index in ceiling_surface_list:
            index = self.delete_overlapping_data(index)
            a = point_sort.SortPointsClockwise(index, True)

            ceiling_info.append(a)

        for index in floor_surface_list:
            index = self.delete_overlapping_data(index)
            a = point_sort.SortPointsClockwise(index, True)

            floor_info.append(a)
        if len(door_surface_list) != 0:
            for index in door_surface_list:
                a = point_sort.SortPointsClockwise(index[:len(index) - 1], True)

                # self.visual_graph(a)
                a.append(index[len(index) - 1])
                door_info.append(a)

        if len(window_surface_list) != 0:
            for index in window_surface_list:
                a = point_sort.SortPointsClockwise(index[:len(index) - 1], True)

                a.append(index[len(index) - 1])

                window_info.append(a)

        return wall_info, ceiling_info, floor_info, door_info, window_info
    
# if __name__ == "__main__":

#     pred_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_pred.ply")
#     ceiling_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_ceiling.ply")
#     floor_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_floor.ply")
#     wall_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_wall.ply")
#     door_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_door.ply")
#     window_cloud = pcl.load("/home/dprt/Desktop/dump2/Area_1_office_2_window.ply")

#     test = MakeCityGMLData(pred_cloud, ceiling_cloud, floor_cloud, wall_cloud, door_cloud, window_cloud)
#     test.get_normal_vector(wall_cloud)

#     # test.make_window_info()
#     wall_normal_vector, wall_surface, ceiling_surface, floor_surface, door_surface, window_surface = test.make_point_surface()
#     wall_info = list()
#     ceiling_info = list()
#     floor_info = list()
#     door_info = list()
#     window_info = list()


    # for index in wall_surface:
    #     a = ps.SortPointsClockwise(index, True)
    #     wall_info.append(a)
    #
    # for index in ceiling_surface:
    #     index = test.delete_overlapping_data(index)
    #     a = ps.SortPointsClockwise(index, True)
    #     ceiling_info.append(a)
    #
    # for index in floor_surface:
    #     index = test.delete_overlapping_data(index)
    #     a = ps.SortPointsClockwise(index, True)
    #
    #     floor_info.append(a)
    #
    # for index in door_surface:
    #     a = ps.SortPointsClockwise(index[:len(index)-1], True)
    #     a.append(index[len(index) - 1])
    #     door_info.append(a)
    #
    # for index in window_surface:
    #     a = ps.SortPointsClockwise(index[:len(index) - 1], True)
    #     a.append(index[len(index) - 1])
    #     window_info.append(a)

    # print floor_info
    # for index in door_info:
    #     print index[len(index) - 1]
    # print door_info
    # test_gml = gml.PointCloudToCityGML(ceiling_info, floor_info, wall_info, door_info, window_info)
    # test_gml.MakeRoomObject()
    # test_wall = a.make_wall_info()
    # test_door, normal= a.make_door_info()
    # print test_door
    # for index in test_door:
    #     for i in index:
    #         for j in i:
    # #             print j
    # for index in test_door:
    #     print index
    #     print ""
    # test_window = a.make_window_info()
    # test_cloud_list = list()
    # for index in wall_surface:
    #     test_cloud_list_2 = list()
    #     test_cloud = pcl.PointCloud()
    #     test_cloud.from_list(index)
    #     test_cloud_list.append(test_cloud)
    #     test_cloud_list_2.append(test_cloud)
    #     a.visual_viewer(test_cloud_list_2)
    # a.visual_viewer(test_cloud_list)

    # a.visual_graph(wall_surface)
    # test_cloud_list = list()
    # for index in test:
    #     test_cloud_list_2 = list()
    #     test_cloud = pcl.PointCloud()
    #     test_cloud.from_list(index)
    #     test_cloud_list.append(test_cloud)
    #     test_cloud_list_2.append(test_cloud)
    #     a.visual_viewer(test_cloud_list_2)
    # a.visual_viewer(test_cloud_list)

    # wall_info = list()
    # for index in wall_surface:
    #     a = ps.PointSort(index, True)
    #     b = a.get_sorted_point()
    #     wall_info.append(b)
    #
    # test_gml = gml.PointCloudToCityGML2(wall_info, wall_info, wall_info)
    # test_gml.MakeRoomObject()




