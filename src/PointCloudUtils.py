import pcl
import pcl.pcl_visualization
import random
import numpy as np
import math
import matplotlib.pyplot as plt
from shapely.geometry import box
from sympy import Symbol, solve, Eq
import os
import datetime as dt
ROOT_PATH = ""

def check_point_range2_e(s_point, e_point, point, e=0.0):
    """

    Args:
        s_point: Start point of line segment
        e_point: End point of line segment
        point: [x, y, z] point to check
        e: epsilon value

    Returns:
        if point is included in range return True
        else return False
    """
    # Making straight lines perpendicular to s_point and e_point
    x1 = s_point[0]
    y1 = s_point[1]
    x2 = e_point[0]
    y2 = e_point[1]
    px = point[0]
    py = point[1]
    # Making the equation for line segment
    a = y2 - y1
    b = x2 - x1
    ar1 = a / b
    ar2 = -(b / a)
    # Making the equation perpendicular straight lines
    c1 = y1 - ar2 * x1
    c2 = y2 - ar2 * x2
    check1 = False
    check2 = False

    # Checking the point is included between perpendicular straight lines
    if ar1 > 0:
        if x1 >= x2 and y1 >= y2:
            check1 = ar2 * px - py + c1 > 0
            check2 = ar2 * px - py + c2 < 0
        if x1 < x2 and y1 < y2:
        # else:
            check1 = ar2 * px - py + c2 > 0
            check2 = ar2 * px - py + c1 < 0
    elif ar1 < 0:
        if x1 >= x2 and y1 <= y2:
            check1 = ar2 * px - py + c1 < 0
            check2 = ar2 * px - py + c2 > 0
        if x1 < x2 and y1 > y2:
        # else:
            check1 = ar2 * px - py + c2 < 0
            check2 = ar2 * px - py + c1 > 0

    if check1 == True and check2 == True:
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

    wall_range_min = wall_range[0]
    wall_range_max = wall_range[1]

    x = point[0]
    y = point[1]

    check_X = 0

    check_Y = 0
    # Checking the included point in wall range
    if x <= wall_range_max[0] + e:
        if x >= wall_range_min[0] - e:
            check_X = check_X + 1
    if y <= wall_range_max[1] + e:
        if y >= wall_range_min[1] - e:
            check_Y = check_Y + 1

    if (check_X + check_Y) == 2:
        return True
    else:
        return False


def get_pointRate(pointcloud, normal_vector, point1, point2, distance_threshold):
    """Making the ratio of points that is included in certain area

    Args:
        pointcloud: [x, y, z] point list
        normal_vector: Equation of plane
        point1: Start point of line segment
        point2: End point of line segment
        distance_threshold: Epsilon value

    Returns:
        Ratio of points that is included in certain area
    """
    points = pointcloud.to_list()
    e = distance_threshold

    count = 0
    temp_list = []
    for point in points:
        # Checking the included points in line segment area
        if check_point_range2_e(point1, point2, point):
            temp_list.append(point)
            count = count + 1
    if count == 0:
        return 0.0
    else:
        return float(count) / float(pointcloud.size)


def get_lengthRate(p_cloud, normal_vector, sorted_PointList, distance_threshold):
    """Making the ration of line segment

    Args:
        p_cloud: [x, y, z] point list
        normal_vector: Equation of
        sorted_PointList:
        distance_threshold:

    Returns:

    """
    m_p1 = sorted_PointList[0][0]
    m_p2 = sorted_PointList[-1][0]

    point_rate_list = []
    each_all_lines2 = []
    m_length = math.fabs(m_p1[0] - m_p2[0]) + math.fabs(m_p1[1] - m_p2[1])

    s = sorted_PointList[0][0]
    for each_i in range(len(sorted_PointList) - 1):

        # Make each line segment information
        s_p1 = sorted_PointList[each_i][0]
        s_p2 = sorted_PointList[each_i + 1][0]
        s_length = math.fabs(s_p1[0] - s_p2[0]) + math.fabs(s_p1[1] - s_p2[1])
        line_rate = s_length / m_length
        # Make the ratio of points using existed in line segment area
        point_rate = get_pointRate(p_cloud, normal_vector, s_p1, s_p2, distance_threshold)

        # Need to checking here more clearly
        if s_length <= 0.05:
            point_rate_list.append(point_rate * 100)
            each_all_lines2.append([sorted_PointList[each_i], sorted_PointList[each_i + 1]])
        else:
            if point_rate * 100 > 5.0:

                point_rate_list.append(point_rate * 100)
                each_all_lines2.append([sorted_PointList[each_i], sorted_PointList[each_i + 1]])
            else:
                if line_rate / 2 < point_rate:
                    # Add the line segment information if line rate / 2 < point_rate
                    point_rate_list.append(point_rate * 100)
                    each_all_lines2.append([sorted_PointList[each_i], sorted_PointList[each_i + 1]])

    #         else:
    #             print line_rate * 100, point_rate * 100, s_p1, s_p2
    # print "finish check length"
    return each_all_lines2, point_rate_list



def sorted2Dpoints(point_list):
    """Function to align intersection point

        Since coordinates are created in one plane, align using the distance between points

        Args:
            point_list: Intersection point list on each side

        Returns:
            sorted_list: List of aligned points
            sorted_index: The current position list of indices moved from the existing array.
   """
    max = -1.0
    index = []
    sorted_list = []
    sorted_index = []
    index_list = [i for i in range(len(point_list))]

    # Find two points that have the longest distance
    for i in range(len(point_list) - 1):
        for j in range(i + 1, len(point_list)):
            distance = math.sqrt(
                ((point_list[i][0][0] - point_list[j][0][0]) ** 2) + ((point_list[i][0][1] - point_list[j][0][1]) ** 2))

            if distance > max:
                max = distance
                index = []
                # save index
                index.append(i)
                index.append(j)

    value0 = point_list[index[0]]
    value1 = point_list[index[1]]
    # Adding the start and end point information using the longest distance points
    if index[0] < index[1]:
        point_list.pop(index[0])
        point_list.pop(index[1] - 1)
        index_list.pop(index[0])
        index_list.pop(index[1] - 1)
    else:
        point_list.pop(index[1])
        point_list.pop(index[0])
        index_list.pop(index[1])
        index_list.pop(index[0])

    sorted_list.append(value0)
    sorted_list.append(value1)
    sorted_index.append(index[0])
    sorted_index.append(index[1])

    # Adding the points between start and end point
    while True:
        if len(point_list) == 0:
            break
        min = float("inf")
        index = -1
        for i in range(len(point_list)):
            distance = math.sqrt(
                ((sorted_list[0][0][0] - point_list[i][0][0]) ** 2) + ((sorted_list[0][0][1] - point_list[i][0][1]) ** 2))
            if distance < min:
                min = distance
                index = i

        sorted_list.insert(len(sorted_list) - 1, point_list[index])
        sorted_index.insert(len(sorted_index) - 1, index_list[index])
        index_list.pop(index)
        point_list.pop(index)
    # for i in range(len(sorted_list) - 1):
    #     distance = math.sqrt(
    #         ((sorted_list[i][0][0] - sorted_list[i+1][0][0]) ** 2) + ((sorted_list[i][0][1] - sorted_list[i+1][0][1]) ** 2))
    #     print distance

    return sorted_list


def check_range(points,bbox, e=0.0):
    """Checking the point included in certain area

    Args:
        points: [x, y, z] point
        bbox: Bounding box information [[minX, minY, minZ], [maxX, maxY, maxZ]]
        e: epsilon value

    Returns:
        True or False
    """
    wall_range_min = bbox[0]
    wall_range_max = bbox[1]

    x = points[0]
    y = points[1]

    check_X = 0
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
    #     # print check_X, check_Y
    if (check_X + check_Y) == 2:
        return True
    else:
        return False

def get_cos(coeff_1, coeff_2):
    """Make Cos value between 2 planes

    Args:
        coeff_1: Equation of plane
        coeff_2: Equation of plane

    Returns:
        model_cos: cos value
    """
    model_cos = np.dot(
        [coeff_1[0], coeff_1[1], coeff_1[2]],
        [coeff_2[0], coeff_2[1], coeff_2[2]]) \
                / (np.linalg.norm(
        [coeff_1[0], coeff_1[1], coeff_1[2]])
                   * np.linalg.norm(
                [coeff_2[0], coeff_2[1],
                 coeff_2[2]]))
    return model_cos
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

    return [point_min2, point_max2]

def grid_subsampling_withoutRGB(points, voxel_size):
    """PointCloud Voxelization
        Downsampling using voxelization
    Args:
        points: [x, y, z] data list
        colors: [r, g, b] data list
        voxel_size: Voxel size
    Returns:
        result_data: Voxelization result
        [[x, y, z], [x, ...]...]

    """

    # non_empty_voxel_keys, inverse, nb_pts_per_voxel= np.unique((points // voxel_size).astype(int), axis=0, return_inverse=True, return_counts=True)
    non_empty_voxel_keys, inverse, nb_pts_per_voxel= np.unique(((points - np.min(points, axis=0)) // voxel_size).astype(int), axis=0, return_inverse=True, return_counts=True)
    # return voxel keys [(0, 0)...] list, points list included in each voxel
    idx_pts_vox_sorted=np.argsort(inverse)
    voxel_grid={}
    grid_barycenter,grid_candidate_center=[],[]
    last_seen=0
    maxcount1 = 0
    maxcount2 = 0
    for idx,vox in enumerate(non_empty_voxel_keys):
        # Adding the points and color information to each voxel
        voxel_grid[tuple(vox)]=points[idx_pts_vox_sorted[last_seen:last_seen+nb_pts_per_voxel[idx]]]
        grid_barycenter.append(np.mean(voxel_grid[tuple(vox)],axis=0))
        grid_candidate_center.append(voxel_grid[tuple(vox)][np.linalg.norm(voxel_grid[tuple(vox)]-np.mean(voxel_grid[tuple(vox)],axis=0),axis=1).argmin()])
        last_seen+=nb_pts_per_voxel[idx]
    # print maxcount1
    # print maxcount2
    # print maxcount
    # np.savetxt("/home/dprt/Documents/20210414_PinSout/20210419_new_result/downsampling1.txt", grid_barycenter)
    # np.savetxt("/home/dprt/Documents/20210414_PinSout/20210419_new_result/downsampling2.txt", grid_candidate_center)
    # return grid_candidate_center
    return grid_barycenter


def grid_subsampling(points, colors, voxel_size):
    """PointCloud Voxelization
        Downsampling using voxelization
    Args:
        points: [x, y, z] data list
        colors: [r, g, b] data list
        voxel_size: Voxel size
    Returns:
        result_data: Voxelization result
        [[x, y, z, r, g, b], [x, ...]...]

    """

    non_empty_voxel_keys, inverse, nb_pts_per_voxel= np.unique(((points - np.min(points, axis=0)) // voxel_size).astype(int), axis=0, return_inverse=True, return_counts=True)
    # return voxel keys [(0, 0)...] list, points list included in each voxel
    idx_pts_vox_sorted=np.argsort(inverse)
    voxel_grid={}
    voxel_grid_color={}
    grid_barycenter,grid_candidate_center=[],[]
    cgrid_barycenter,cgrid_candidate_center=[],[]
    last_seen=0

    for idx,vox in enumerate(non_empty_voxel_keys):
        # Adding the points and color information to each voxel
        voxel_grid[tuple(vox)]=points[idx_pts_vox_sorted[last_seen:last_seen+nb_pts_per_voxel[idx]]]
        voxel_grid_color[tuple(vox)]=colors[idx_pts_vox_sorted[last_seen:last_seen+nb_pts_per_voxel[idx]]]

        grid_barycenter.append(np.mean(voxel_grid[tuple(vox)],axis=0))
        cgrid_barycenter.append(np.mean(voxel_grid_color[tuple(vox)],axis=0))

        # Making center point and color information
        grid_candidate_center.append(voxel_grid[tuple(vox)][np.linalg.norm(voxel_grid[tuple(vox)]-np.mean(voxel_grid[tuple(vox)],axis=0),axis=1).argmin()])
        cgrid_candidate_center.append(voxel_grid_color[tuple(vox)][np.linalg.norm(voxel_grid_color[tuple(vox)]-np.mean(voxel_grid_color[tuple(vox)],axis=0),axis=1).argmin()])

        last_seen+=nb_pts_per_voxel[idx]
    result_data = np.concatenate((grid_candidate_center, cgrid_candidate_center), axis=1)

    return result_data

def grid_subsampling2(points, grid_size):
    """Making the GridMap information
        Dividing the PointCloud data using grid_size
    Args:
        points: [x, y, z, r, g, b] data list
        grid_size: Each cell size of GridMap

    Returns:
        voxel_grid: GridMap result
        {
            [index_x, index_y]: [[x, y, z, r, g, b], [x, y, z, r, g, b]],
            ....
        }

    """
    # Making grid index without Z-value and points list
    non_empty_voxel_keys, inverse, nb_pts_per_voxel= np.unique(((points - np.min(points, axis=0))[:, :2] // grid_size).astype(int), axis=0, return_inverse=True, return_counts=True)
    idx_pts_vox_sorted=np.argsort(inverse)
    voxel_grid={}
    last_seen=0

    for idx,vox in enumerate(non_empty_voxel_keys):
        voxel_grid[tuple(vox)] = points[idx_pts_vox_sorted[last_seen:last_seen+nb_pts_per_voxel[idx]]]
        last_seen+=nb_pts_per_voxel[idx]

    return voxel_grid


def make_one_file(data_list, path, name):
    """Merging the data as file

      Args:
          data_list: PointCloud list

      Returns:
          Save merged PointCloud data
      """
    merged_data = list()
    for i in data_list:
        merged_data.extend(i.to_list())
    merged_cloud = pcl.PointCloud()
    merged_cloud.from_list(merged_data)
    save_path = path + "/"+str(name)+".ply"
    pcl.save(merged_cloud, save_path, "ply")

def merge_one_file(data_list):
    """Merging the data

    Args:
        data_list: PointCloud list

    Returns:
        merged_cloud: Merged PointCloud data
        bbox_info: Bounding box information of merged PointCloud
    """
    merged_data = list()
    for i in data_list:
        merged_data.extend(i.to_list())
    merged_cloud = pcl.PointCloud()
    merged_cloud.from_list(merged_data)
    bbox_info = get_range(merged_cloud)
    return merged_cloud, bbox_info

def make_straight(normal_vector, point_1, point_2, min_z, max_z):
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
    # Making the equation of line segment
    u = np.array(point_2) - np.array(point_1)
    # Finding intersection point between plane and line segment using min_z and max_z
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
    # return min and max intersection point
    return min_intersect_point, max_intersect_point

def make_side_line(bbox_info, normal_vector):
    """Making the Side Line using bounding box and plane equation

        Finding the intersection point between the equation of the plane and the bounding box

    Args:
        bbox_info: Bounding box of plane ( [[minX, minY, minZ], [maxX, maxY, maxZ]])
        normal_vector: plane equation

    Returns:
        intersection point list : [[x1, y1, minZ], [x1, y1, maxZ], [x2, y2, minZ], [x2, y2, maxZ]]
    """
    box_info = box(bbox_info[0][0], bbox_info[0][1], bbox_info[1][0], bbox_info[1][1])
    box_coords = box_info.exterior.coords
    # coords = [(maxX, minY), (maxX, maxY), (minX, maxY), (minX, minY)]
    min_z = bbox_info[1][2]
    max_z = bbox_info[0][2]
    min_value = []
    max_value = []
    min_h_distance = []
    min_l_distance = []
    max_h_distance = []
    max_l_distance = []

    # 3 - - - - 2
    #  \         \
    #   \         \
    #    4 - - - - 1
    # line segments : [1 - 2], [2 - 3], [3 - 4], [4 - 1]
    for box_i in range(len(box_coords) - 1):

        checking_box_info = (box_coords[box_i] == box_coords[box_i + 1])
        if checking_box_info != True:
            # Finding the intersection point using normal_vector and line segments
            min_result, max_result = make_straight(normal_vector, box_coords[box_i], box_coords[box_i + 1], min_z,
                                                         max_z)
            # Store distance between intersection point and bbox_info (check x, y point)
            # Need to check process (need update)
            min_l_distance.append(
                math.fabs(bbox_info[0][0] - min_result[0]) + math.fabs(bbox_info[0][1] - min_result[1]))
            min_h_distance.append(
                math.fabs(bbox_info[0][0] - max_result[0]) + math.fabs(bbox_info[0][1] - max_result[1]))
            max_l_distance.append(
                math.fabs(bbox_info[1][0] - min_result[0]) + math.fabs(bbox_info[1][1] - min_result[1]))
            max_h_distance.append(
                math.fabs(bbox_info[1][0] - max_result[0]) + math.fabs(bbox_info[1][1] - max_result[1]))
            min_value.append(min_result)
            max_value.append(max_result)
    # Finding the nearliest point value
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

def avg_distance(normal_vector, point_list):
    """Checking the outline point is near by main plane

    Args:
        normal_vector: Equation of main plane
        point_list: Outline point(4) of sub plane

    Returns:
        Average distance = d_sum / len(point_list)
    """
    d_sum = 0
    for each_point in point_list:
        d = math.fabs((normal_vector[0] * each_point[0]) + (normal_vector[1] * each_point[1]) + (normal_vector[2] * each_point[2]) + normal_vector[3]) / \
            math.sqrt(math.pow(normal_vector[0], 2) + math.pow(normal_vector[1], 2) + math.pow(normal_vector[2], 2))
        d_sum += d

    return d_sum / len(point_list)

def saveAsWKT(roomIndex, ceiling, floor, wall_list):
    """Creating the WKT information using room info

        Making the SOLID information usin each room information
    Args:
        roomIndex: Clustered area number from original data
        ceiling: The ceiling result of graph cycle
        floor: The floor result of graph cycle
        wall_list: The wall_list result of graph cycle

    Returns:
        Save the Text file that is included WKT SOLID data
    """
    global ROOT_PATH
    if len(ROOT_PATH) == 0:
        ROOT_PATH = os.path.join("../data/WKT_RESULT", str(dt.datetime.now()).replace(' ', 'T').split('.')[0])
        if os.path.isdir(ROOT_PATH) == False:
            os.mkdir(ROOT_PATH)
    else:
        ROOT_PATH = os.path.join(ROOT_PATH, "WKT_RESULT")
        if os.path.isdir(ROOT_PATH) == False:
            os.mkdir(ROOT_PATH)
    for room_i in range(len(wall_list)):
        solid_info = "SOLID(("
        ceiling_info = "(("
        floor_info = "(("
        for ceiling_i, points in enumerate(ceiling[room_i]):
            ceiling_info += str(points[0]) + " " + str(points[1]) + " " + str(points[2])
            if ceiling_i != len(ceiling) - 1:
                ceiling_info += ","
        ceiling_info += ")),"
        solid_info += ceiling_info
        for floor_i, points in enumerate(floor[room_i]):
            floor_info += str(points[0]) + " " + str(points[1]) + " " + str(points[2])
            if floor_i != len(floor) - 1:
                floor_info += ","
        floor_info += ")),"
        solid_info += floor_info
        for i, wall in enumerate(wall_list[room_i]):
            wall_info = "(("
            for wall_i, points in enumerate(wall):
                wall_info += str(points[0]) + " " + str(points[1]) + " " + str(points[2])
                if wall_i != len(wall):
                    wall_info += ","
            wall_info += "))"
            if i != len(wall_list) - 1:
                wall_info += ","
            solid_info += wall_info
        solid_info += "))"
        save_path = os.path.join(ROOT_PATH, str(roomIndex) + "_" +str(room_i) +".txt")
        with open(save_path, 'w') as f:
            f.writelines(solid_info)
        f.close()

def visual_graph3(point_list):

    for index in point_list:
        x = []
        y = []
        for each_v in index:
            x.append(each_v[0])
            y.append(each_v[1])
        plt.plot(x, y)

        plt.legend()
    plt.show()
def visual_graph3_with_text(point_list, rate_list):

    # print len(point_list[0]), len(rate_list[0])
    for i in range(len(point_list)):
        x = []
        y = []
        v = []


        for j in range(len(point_list[i])):
            x.append(point_list[i][j][0][0])
            y.append(point_list[i][j][0][1])
            if j < len(point_list[i]) - 1:
                v.append(round(rate_list[i], 2))
                # if rate_list >=10:
                #     v.append(10)
                # else:
                #     v.append(1)

        plt.plot(x, y)
        plt.legend()
        for k in range(len(x) - 1):

            n_x = (x[k] + x[k + 1]) / 2
            n_y = (y[k] + y[k + 1]) / 2
            if k < len(x) - 1:
                if v[k] < 5:
                    plt.text(n_x, n_y, str(v[k]))

    plt.show()


def visual_graph(point_list):
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

        x.append(index[0][0])
        y.append(index[0][2])
    plt.scatter(x, y, label="stars", color="green",
                marker="*", s=50)
    plt.plot(x, y)
    plt.legend()
    plt.show()

def visual_graph2(point_list):
    x = []
    y = []
    # print point_list
    for index in point_list:
        x.append(index[0])
        y.append(index[1])
    plt.scatter(x, y, label="stars", color="green",
                marker="*", s=50)
    plt.plot(x, y)
    plt.legend()
    plt.show()

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

if __name__ == "__main__":

    # wall_cloud = pcl.load("/home/dprt/Documents/20210414_PinSout/20210419_new_result/check_data.ply")
    wall_cloud = pcl.load("/home/dprt/Documents/20210414_PinSout/20210419_new.ply")
    grid_subsampling_withoutRGB(wall_cloud.to_array(), 0.02)