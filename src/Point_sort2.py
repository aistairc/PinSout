from Point_sort import *
import numpy as np

# point_list = [[0.0,0.0,0.0], [0.0,0.0,4.0],[2.0,0.0,4.0], [2.0,0.0,2.5], [5.0, 0.0, 2.5], [5.0,0.0,0.0], [2.0,0.0,0.0]]

def CalculateCentroid(old_point_list):

    a = Point_sort()
    point_list = a.SortPointsClockwise2(old_point_list, True)


    # centre_point = [0.0, 0.0, 0.0]
    # area_total = 0.0
    #
    # p1 = point_list[0]
    # p2 = point_list[1]
    #
    # for i in range(2, len(point_list)):
    #
    #     p3 = point_list[i]
    #     edge1 = np.asarray(p3) - np.asarray(p1)
    #     edge2 = np.asarray(p3) - np.asarray(p2)
    #
    #     crossProduct = np.cross(edge1, edge2)
    #     area = np.linalg.norm(crossProduct) / 2.0
    #
    #     centre_point[0] = centre_point[0] + (area * (p1[0] + p2[0] + p3[0]) / float(len(old_point_list)))
    #     centre_point[1] = centre_point[1] + (area * (p1[1] + p2[1] + p3[1]) / float(len(old_point_list)))
    #     centre_point[2] = centre_point[2] + (area * (p1[2] + p2[2] + p3[2]) / float(len(old_point_list)))
    #
    #     area_total = area_total + area
    #     p2 = p3
    #
    # result = [centre_point[0] / area_total, centre_point[1] / area_total, centre_point[2] / area_total]
    #
    # new_point_list = a.SortPointsClockwise3(point_list, True)
    new_point_list = a.SortPointsClockwise3(point_list, True)
    # a.visual_graph2(point_list)
    # a.visual_graph2(new_point_list)

    return new_point_list

def CalculateCentroid2(old_point_list):

    a = Point_sort()
    point_list = a.SortPointsClockwise2(old_point_list, True)
    # print point_list

    centre_point = [0.0, 0.0, 0.0]
    area_total = 0.0

    p1 = point_list[0]
    p2 = point_list[1]

    for i in range(2, len(point_list)):

        p3 = point_list[i]
        edge1 = np.asarray(p3) - np.asarray(p1)
        edge2 = np.asarray(p3) - np.asarray(p2)

        crossProduct = np.cross(edge1, edge2)
        area = np.linalg.norm(crossProduct) / 2.0

        centre_point[0] = centre_point[0] + (area * (p1[0] + p2[0] + p3[0]) / float(len(old_point_list)))
        centre_point[1] = centre_point[1] + (area * (p1[1] + p2[1] + p3[1]) / float(len(old_point_list)))
        centre_point[2] = centre_point[2] + (area * (p1[2] + p2[2] + p3[2]) / float(len(old_point_list)))

        area_total = area_total + area
        p2 = p3

    result = [centre_point[0] / area_total, centre_point[1] / area_total, centre_point[2] / area_total]

    # a = np.asarray(point_list)
    # x = np.mean(a.T[0])
    # y = np.mean(a.T[1])
    # z = np.mean(a.T[2])




    # print type(result[0])

    new_point_list = a.SortPointsClockwise3(point_list, True)

    # a.visual_graph2(point_list)
    # a.visual_graph2(new_point_list)

    return new_point_list

def CalculateCentroid2D(old_point_list):

    a = Point_sort()
    point_list = a.SortPointsClockwise2D(old_point_list, True)


    # centre_point = [0.0, 0.0]
    # area_total = 0.0

    # p1 = point_list[0]
    # p2 = point_list[1]
    #
    # for i in range(2, len(point_list)):
    #
    #     p3 = point_list[i]
    #     edge1 = np.asarray(p3) - np.asarray(p1)
    #     edge2 = np.asarray(p3) - np.asarray(p2)
    #
    #     crossProduct = np.cross(edge1, edge2)
    #     area = np.linalg.norm(crossProduct) / 2.0
    #
    #     centre_point[0] = centre_point[0] + (area * (p1[0] + p2[0] + p3[0]) / float(len(old_point_list)))
    #     centre_point[1] = centre_point[1] + (area * (p1[1] + p2[1] + p3[1]) / float(len(old_point_list)))
    #
    #
    #     area_total = area_total + area
    #     p2 = p3
    #
    # if area_total == 0.0:
    #     result_x = 0.0
    #     result_y = 0.0
    # else:
    #     result_x = centre_point[0] / area_total
    #     result_y = centre_point[1] / area_total
    # a = np.asarray(point_list)
    # print np.mean(a.T[0]), np.mean(a.T[1]), np.mean(a.T[2])
    # result = [np.mean(a.T[0]), np.mean(a.T[1])]
    # new_point_list = a.SortPointsClockwise2DClose(result, point_list, True)
    new_point_list = a.SortPointsClockwise2DClose(point_list, True)

    # a.visual_graph(point_list)
    # a.visual_graph(new_point_list)

    return new_point_list