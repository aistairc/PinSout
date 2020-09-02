# def intersection_over_union(box1, box2):
#     x1 = max(box1[0], box2[0])
#     y1 = max(box1[1], box2[1])
#     x2 = min(box1[2], box2[2])
#     y2 = min(box1[3], box2[3])
#
#
#     area_intersection = (x2 - x1) * (y2 - y1)
#     area_box1 = (box1[2] - box1[0]) * (box1[3] - box1[2])
#     area_box2 = (box2[2] - box2[0]) * (box2[3] - box2[2])
#     area_union = area_box1 + area_box2 - area_intersection
#
#     iou = area_intersection / area_union
#     return iou
#
# box1 = [0.0, 0.0, 2.0, 2.0]
# box2 = [0.0, 0.0, 2.0, 2.0]
# # box2 = [1.0, 1.0, 3.0, 3.0]
#
# result = intersection_over_union(box1, box2)
# print result


#


# include <stdio.h>
# include <string.h>

# define MIN(a,b) (((a)<(b))?(a):(b))
# define MAX(a,b) (((a)>(b))?(a):(b))

# import rtree.index
# from shapely.affinity import rotate, translate
# from shapely.geometry import Polygon
# import numpy as np
#
# def rect_polygon(x, y, width, height, angle):
#     """Return a shapely Polygon describing the rectangle with centre at
#     (x, y) and the given width and height, rotated by angle quarter-turns.
#
#     """
#     w = width / 2
#     h = height / 2
#     p = Polygon([(-w, -h), (w, -h), (w, h), (-w, h)])
#     return translate(rotate(p, angle * 90), x, y)
#
#
# def intersection_over_union(rects_a, rects_b):
#     """Calculate the intersection-over-union for every pair of rectangles
#     in the two arrays.
#
#     Arguments:
#     rects_a: array_like, shape=(M, 5)
#     rects_b: array_like, shape=(N, 5)
#         Rotated rectangles, represented as (centre x, centre y, width,
#         height, rotation in quarter-turns).
#
#     Returns:
#     iou: array, shape=(M, N)
#         Array whose element i, j is the intersection-over-union
#         measure for rects_a[i] and rects_b[j].
#
#     """
#     m = len(rects_a)
#     n = len(rects_b)
#     if m > n:
#         # More memory-efficient to compute it the other way round and
#         # transpose.
#         return intersection_over_union(rects_b, rects_a).T
#
#     # Convert rects_a to shapely Polygon objects.
#     polys_a = [rect_polygon(*r) for r in rects_a]
#
#     # Build a spatial index for rects_a.
#     index_a = rtree.index.Index()
#     for i, a in enumerate(polys_a):
#         index_a.insert(i, a.bounds)
#
#     # Find candidate intersections using the spatial index.
#     iou = np.zeros((m, n))
#     for j, rect_b in enumerate(rects_b):
#         b = rect_polygon(*rect_b)
#         for i in index_a.intersection(b.bounds):
#             a = polys_a[i]
#             intersection_area = a.intersection(b).area
#             if intersection_area:
#                 iou[i, j] = intersection_area / a.union(b).area
#
#     return iou
import Point_sort as rs
import Point_sort2 as rs2
from plyfile import PlyData, PlyElement
import numpy as np
import pcl

def dist(p1, p2):

    return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1])


def ccw(p1, p2, p3):

    cross_product = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])

    if cross_product > 0:
        return 1

    elif cross_product < 0:
        return -1
    else:
        return 0


def get_IntersectionPoint(original_line, new_line):
    p1 = original_line[0]
    p2 = original_line[1]
    p3 = new_line[0]
    p4 = new_line[1]
    rect = []
    x = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[0] - p4[0]) - (p1[0] - p2[0]) * (p3[0] * p4[1] - p3[1] * p4[0])) / (
                (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]))

    y = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] * p4[1] - p3[1] * p4[0])) / (
                (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]))

    rect.append(x)
    rect.append(y)

    return rect

def Max(a, b):

    return a if a > b else b

def Min(a, b):

    return a if a < b else b

def check_PolygonInOut(p1, check_points):

    rect = 0

    for check_i in range(len(check_points)-1):

        if ccw(check_points[check_i], check_points[check_i+1], p1) == 0:
            min_x = Min(check_points[check_i][0], check_points[check_i+1][0])
            min_y = Min(check_points[check_i][1], check_points[check_i+1][1])
            max_x = Max(check_points[check_i][0], check_points[check_i+1][0])
            max_y = Max(check_points[check_i][1], check_points[check_i+1][1])

            if ((min_x <= p1[0] and p1[0] <= max_x) and (min_y <= p1[1] and p1[1] <= max_y)):
                return 1


    outside_point = []
    outside_point.append(1.0)
    outside_point.append(1234567.0)

    outside_line = []
    outside_line.append(outside_point)
    outside_line.append(p1)

    for check_i in range(len(check_points) - 1):
        check_line = []
        check_line.append(check_points[check_i])
        check_line.append(check_points[check_i+1])
        rect = rect + get_LineIntersection(outside_line, check_line)


    rect = rect % 2.0

    return rect

def get_PolygonArea(intersection_points):

    rect = 0.0

    for i in range(len(intersection_points) - 1):

        rect = rect + (intersection_points[i][0] * intersection_points[i+1][1]) - (intersection_points[i][1] * intersection_points[i+1][0])

    rect = rect if rect > 0 else -rect

    return rect / 2.0



def get_LineIntersection(original_line, new_line):

    l1_l2 = ccw(original_line[0], original_line[1], new_line[0]) * ccw(original_line[0], original_line[1], new_line[1])
    l2_l1 = ccw(new_line[0], new_line[1], original_line[0]) * ccw(new_line[0], new_line[1], original_line[1])

    return (l1_l2 < 0) and (l2_l1 < 0)


def get_intersection(original_points, new_points):

    intersection_points = []


    for original_i in range(len(original_points)-1):
        original_line = []
        original_line.append(original_points[original_i])
        original_line.append(original_points[original_i + 1])

        for new_i in range(len(new_points)-1):
            new_line = []
            new_line.append(new_points[new_i])
            new_line.append(new_points[new_i + 1])

            if get_LineIntersection(original_line, new_line):

                intersection_points.append(get_IntersectionPoint(original_line, new_line))

    for original_i in range(len(original_points)-1):

        if check_PolygonInOut(original_points[original_i], new_points):
            intersection_points.append(original_points[original_i])

    for new_i in range(len(new_points) - 1):

        if check_PolygonInOut(new_points[new_i], original_points):
            intersection_points.append(new_points[new_i])

    if len(intersection_points) > 3:
        if check_closed(intersection_points) == -1:

            intersection_points = rs2.CalculateCentroid2D(intersection_points)

        rect = get_PolygonArea(intersection_points)

        return rect
    else:
        return -1


def get_IOU(original_points, new_points):


    intersection_area = get_intersection(original_points, new_points)
    if intersection_area != -1:
        original_area = get_PolygonArea(original_points)
        new_area = get_PolygonArea(new_points)

        union_area = original_area + new_area - intersection_area

        rect = intersection_area / union_area

        return rect
    else:
        return -1


###################################

def check_closed(point_list):

    point_list_len = len(point_list) - 1

    if (point_list[0][0] != point_list[point_list_len][0]) or (point_list[0][1] != point_list[point_list_len][1]):
        # print "Last vertex and first vertex are not connected."
        return -1
    else:
        # print "Last vertex and first vertex are connected."
        return 1
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





if __name__ == '__main__':

    list_original = ["a_01","a_02","a_03","a_04",
                     "b_01","b_02","b_03","b_04",
                     "b_05","b_06","b_07","b_08",
                     "b_09","b_10",
                     "c_01","c_02","c_03","c_04"]
    test_path = "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/"
    # new_points = [[[-0.215571228170187, -0.948702039242467, 3.0423800945281982], [-4.6980434245696, -0.947171436056617, 3.0480549335479736], [-4.6980434245696, -0.947171436056617, 0.036078501492738724], [-0.215571228170187, -0.948702039242467, 0.02952166646718979], [-0.215571228170187, -0.948702039242467, 3.0423800945281982]], [[-0.215555162439024, -3.11223542275595, 3.0621800422668457], [-0.215571228170187, -0.948702039242467, 3.0423800945281982], [-0.215571228170187, -0.948702039242467, 0.02952166646718979], [-0.215555162439024, -3.11223542275595, 0.03738019987940788], [-0.215555162439024, -3.11223542275595, 3.0621800422668457]], [[-4.69794659232765, -3.11232705634018, 3.0621800422668457], [-0.215555162439024, -3.11223542275595, 3.0621800422668457], [-0.215555162439024, -3.11223542275595, 0.03738019987940788], [-4.69794659232765, -3.11232705634018, 0.03738019987940788], [-4.69794659232765, -3.11232705634018, 3.0621800422668457]], [[-4.69794659232765, -3.11232705634018, 0.03738019987940788], [-4.6980434245696, -0.947171436056617, 0.036078501492738724], [-4.6980434245696, -0.947171436056617, 3.0480549335479736], [-4.69794659232765, -3.11232705634018, 3.0621800422668457], [-4.69794659232765, -3.11232705634018, 0.03738019987940788]], [[1.17154207089495, -0.658473605762648, 3.047840118408203], [2.85423922802842, -0.658409047928616, 3.0363399982452393], [2.85423922802842, -0.658409047928616, 0.03525486961007118], [1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17154207089495, -0.658473605762648, 3.047840118408203]], [[1.17232344394907, 3.10833391622898, 3.0554399490356445], [1.17154207089495, -0.658473605762648, 3.047840118408203], [1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17232344394907, 3.10833391622898, 0.01646059937775135], [1.17232344394907, 3.10833391622898, 3.0554399490356445]], [[1.17232344394907, 3.10833391622898, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 3.0554399490356445], [1.17232344394907, 3.10833391622898, 3.0554399490356445], [1.17232344394907, 3.10833391622898, 0.01646059937775135]], [[2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85423922802842, -0.658409047928616, 0.03525486961007118], [2.85423922802842, -0.658409047928616, 3.0363399982452393], [2.85592963672819, 3.10498754810542, 3.0554399490356445], [2.85592963672819, 3.10498754810542, 0.01646059937775135]], [[-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186], [-0.0736667534394775, -0.792109886945326, 0.02443454973399639]], [[-0.0243317252695328, -3.11223151357283, 3.0621800422668457], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982], [-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457]], [[4.69940499681119, -3.11213494616376, 3.0621800422668457], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457], [-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 3.0621800422668457]], [[4.69711197662828, 3.10132797859425, 3.0554399490356445], [4.69940499681119, -3.11213494616376, 3.0621800422668457], [4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69711197662828, 3.10132797859425, 0.01646059937775135], [4.69711197662828, 3.10132797859425, 3.0554399490356445]], [[4.69711197662828, 3.10132797859425, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 3.0554399490356445], [4.69711197662828, 3.10132797859425, 3.0554399490356445], [4.69711197662828, 3.10132797859425, 0.01646059937775135]], [[2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.98952686704443, -0.790252950611448, 3.038594961166382], [2.99053100234752, 3.10472001183641, 3.0554399490356445], [2.99053100234752, 3.10472001183641, 0.01646059937775135]], [[2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.85417986314639, -0.790574479280031, 0.03525486961007118], [2.85417986314639, -0.790574479280031, 3.0363399982452393], [2.98952686704443, -0.790252950611448, 3.038594961166382], [2.98952686704443, -0.790252950611448, 0.024747176095843315]], [[1.03899695814266, -0.794886605340243, 3.0378000736236572], [2.85417986314639, -0.790574479280031, 3.0363399982452393], [2.85417986314639, -0.790574479280031, 0.03525486961007118], [1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03899695814266, -0.794886605340243, 3.0378000736236572]], [[1.03671818059138, 3.10860344786449, 3.0554399490356445], [1.03899695814266, -0.794886605340243, 3.0378000736236572], [1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03671818059138, 3.10860344786449, 0.01646059937775135], [1.03671818059138, 3.10860344786449, 3.0554399490356445]], [[-4.69822532075788, 3.12000233212, 3.0554399490356445], [1.03671818059138, 3.10860344786449, 3.0554399490356445], [1.03671818059138, 3.10860344786449, 0.01646059937775135], [-4.69822532075788, 3.12000233212, 0.01646059937775135], [-4.69822532075788, 3.12000233212, 3.0554399490356445]], [[-4.69822532075788, 3.12000233212, 0.01646059937775135], [-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-4.69805035458894, -0.792217156192569, 3.0480549335479736], [-4.69822532075788, 3.12000233212, 3.0554399490356445], [-4.69822532075788, 3.12000233212, 0.01646059937775135]], [[-0.0736667534394775, -0.792109886945326, 3.0496299266815186], [-4.69805035458894, -0.792217156192569, 3.0480549335479736], [-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186]]]
    # sampling 1000 / k 1000
    new_points = [[[4.85258445844372, 3.26737701297628, 0.05875932052731514], [4.85259324246169, 3.09481363662704, 0.0557435005903244], [4.85259324246169, 3.09481363662704, 3.0445098876953125], [4.85258445844372, 3.26737701297628, 3.0591700077056885], [4.85258445844372, 3.26737701297628, 0.05875932052731514]], [[4.85259324246169, 3.09481363662704, 0.0557435005903244], [4.69759616397057, 3.09533036759260, 0.0557435005903244], [4.69759616397057, 3.09533036759260, 3.0445098876953125], [4.85259324246169, 3.09481363662704, 3.0445098876953125], [4.85259324246169, 3.09481363662704, 0.0557435005903244]], [[4.69756442362172, 3.26734319976325, 3.0591700077056885], [4.69759616397057, 3.09533036759260, 3.0445098876953125], [4.69759616397057, 3.09533036759260, 0.0557435005903244], [4.69756442362172, 3.26734319976325, 0.05875932052731514], [4.69756442362172, 3.26734319976325, 3.0591700077056885]], [[4.69756442362172, 3.26734319976325, 0.05875932052731514], [4.85258445844372, 3.26737701297628, 0.05875932052731514], [4.85258445844372, 3.26737701297628, 3.0591700077056885], [4.69756442362172, 3.26734319976325, 3.0591700077056885], [4.69756442362172, 3.26734319976325, 0.05875932052731514]], [[4.85291706511409, -3.26673189232133, 3.0660300254821777], [4.85290913535968, -3.11095065434166, 3.05641508102417], [4.85290913535968, -3.11095065434166, 0.03781680017709732], [4.85291706511409, -3.26673189232133, 0.021199500188231468], [4.85291706511409, -3.26673189232133, 3.0660300254821777]], [[4.85291706511409, -3.26673189232133, 0.021199500188231468], [-0.214608106762168, -3.26625288692056, 0.021199500188231468], [-0.214608106762168, -3.26625288692056, 3.0660300254821777], [4.85291706511409, -3.26673189232133, 3.0660300254821777], [4.85291706511409, -3.26673189232133, 0.021199500188231468]], [[-0.214608106762168, -3.26625288692056, 0.021199500188231468], [-0.214745433375814, -3.11238374795242, 0.03781680017709732], [-0.214745433375814, -3.11238374795242, 3.05641508102417], [-0.214608106762168, -3.26625288692056, 3.0660300254821777], [-0.214608106762168, -3.26625288692056, 0.021199500188231468]], [[-0.214745433375814, -3.11238374795242, 0.03781680017709732], [-4.69633638853741, -3.11365110729525, 0.03781680017709732], [-4.69633638853741, -3.11365110729525, 3.05641508102417], [-0.214745433375814, -3.11238374795242, 3.05641508102417], [-0.214745433375814, -3.11238374795242, 0.03781680017709732]], [[-4.69633531015145, -3.26582925379346, 3.0660300254821777], [-4.69633638853741, -3.11365110729525, 3.05641508102417], [-4.69633638853741, -3.11365110729525, 0.03781680017709732], [-4.69633531015145, -3.26582925379346, 0.021199500188231468], [-4.69633531015145, -3.26582925379346, 3.0660300254821777]], [[-4.85362003542260, -3.26581438652979, 3.0660300254821777], [-4.69633531015145, -3.26582925379346, 3.0660300254821777], [-4.69633531015145, -3.26582925379346, 0.021199500188231468], [-4.85362003542260, -3.26581438652979, 0.021199500188231468], [-4.85362003542260, -3.26581438652979, 3.0660300254821777]], [[-4.85362003542260, -3.26581438652979, 0.021199500188231468], [-4.85310388112334, 3.12717059648017, 0.0557435005903244], [-4.85310388112334, 3.12717059648017, 3.0445098876953125], [-4.85362003542260, -3.26581438652979, 3.0660300254821777], [-4.85362003542260, -3.26581438652979, 0.021199500188231468]], [[-4.85310388112334, 3.12717059648017, 0.0557435005903244], [-4.69638060941402, 3.12664811071267, 0.0557435005903244], [-4.69638060941402, 3.12664811071267, 3.0445098876953125], [-4.85310388112334, 3.12717059648017, 3.0445098876953125], [-4.85310388112334, 3.12717059648017, 0.0557435005903244]], [[-4.69635264605458, -0.819445757436463, 3.0691800117492676], [-4.69638060941402, 3.12664811071267, 3.0445098876953125], [-4.69638060941402, 3.12664811071267, 0.0557435005903244], [-4.69635264605458, -0.819445757436463, 0.05283840000629425], [-4.69635264605458, -0.819445757436463, 3.0691800117492676]], [[-4.69635264605458, -0.819445757436463, 0.05283840000629425], [-0.216780360466777, -0.832326947329312, 0.04095960035920143], [-0.216780360466777, -0.832326947329312, 3.0555601119995117], [-4.69635264605458, -0.819445757436463, 3.0691800117492676], [-4.69635264605458, -0.819445757436463, 0.05283840000629425]], [[-0.216780360466777, -0.832326947329312, 0.04095960035920143], [-0.0815272104394807, -0.832715873190541, 0.04095960035920143], [-0.0815272104394807, -0.832715873190541, 3.0555601119995117], [-0.216780360466777, -0.832326947329312, 3.0555601119995117], [-0.216780360466777, -0.832326947329312, 0.04095960035920143]], [[-0.0231737561759769, -3.11232957296025, 3.05641508102417], [-0.0815272104394807, -0.832715873190541, 3.0555601119995117], [-0.0815272104394807, -0.832715873190541, 0.04095960035920143], [-0.0231737561759769, -3.11232957296025, 0.03781680017709732], [-0.0231737561759769, -3.11232957296025, 3.05641508102417]], [[4.69874137451093, -3.11099425179489, 3.05641508102417], [-0.0231737561759769, -3.11232957296025, 3.05641508102417], [-0.0231737561759769, -3.11232957296025, 0.03781680017709732], [4.69874137451093, -3.11099425179489, 0.03781680017709732], [4.69874137451093, -3.11099425179489, 3.05641508102417]], [[4.69874137451093, -3.11099425179489, 0.03781680017709732], [4.85290913535968, -3.11095065434166, 0.03781680017709732], [4.85290913535968, -3.11095065434166, 3.05641508102417], [4.69874137451093, -3.11099425179489, 3.05641508102417], [4.69874137451093, -3.11099425179489, 0.03781680017709732]]]
    a = rs.Point_sort()

    original_check = [False for i in range(len(list_original))]
    new_check = [False for i in range(len(new_points))]

    print original_check
    print new_check
    result_IOU_list = []
    print len(new_points)
    original_data = []
    e = 0.005
    for original_i in range(len(list_original)):


        plydata = PlyData.read(test_path+list_original[original_i]+".ply")
        test_data = []
        test_data.append(plydata['vertex']['x'])
        test_data.append(plydata['vertex']['y'])
        test_data.append(plydata['vertex']['z'])
        test_data = np.asarray(test_data).T.tolist()
        test_data_bbox = get_range(test_data, e)
        original_data.append(test_data)
        temp_list1 = []
        temp_list2 = []
        opoints = []
        for each_data in test_data:
            temp_list1.append([each_data[0], each_data[2]])
            temp_list2.append([each_data[1], each_data[2]])

        if check_closed(temp_list1) == -1:
            temp_list1 = rs2.CalculateCentroid2D(temp_list1)
        if check_closed(temp_list2) == -1:
            temp_list2 = rs2.CalculateCentroid2D(temp_list2)
        if get_PolygonArea(temp_list1) > get_PolygonArea(temp_list2):
            checkValue = True
            opoints.extend(temp_list1)
        else:
            checkValue = False
            opoints.extend(temp_list2)
        for new_j in range(len(new_points)):
            if new_j == 8 or new_j == 14:
                continue
            if new_check[new_j] == False:
                new_data_bbox = get_range(new_points[new_j], e)

                if check_bbox(test_data_bbox, new_data_bbox):

                    temp_list3 = []

                    for new_each_data in new_points[new_j]:
                        if checkValue:
                            temp_list3.append([new_each_data[0], new_each_data[2]])
                        else:
                            temp_list3.append([new_each_data[1], new_each_data[2]])

                    if check_closed(temp_list3) == -1:
                        temp_list3 = rs2.CalculateCentroid2D(temp_list3)

                    result_IOU = get_IOU(opoints, temp_list3)
                    print original_i,new_j,result_IOU
                    if result_IOU <= 1.0 and result_IOU >= 0.5:
                        result_temp = []
                        result_temp.append(original_i)
                        result_temp.append(new_j)
                        result_temp.append(result_IOU)
                        result_IOU_list.append(result_temp)
                        graph_data = []
                        graph_data.append(opoints)
                        graph_data.append(temp_list3)

                        a.visual_graph3(graph_data)
                    # original_check[i] = True
                    # new_check[j] = True

            else:
                continue

    print
    print "count of result : ", len(result_IOU_list)
    count = 0

    # for new_j in range(len(new_points)):
    #
    #     temp_list3 = []
    #     temp_list4 = []
    #     result_i = 0.0
    #     for new_each_data in new_points[new_j]:
    #
    #         temp_list3.append([new_each_data[0], new_each_data[2]])
    #         temp_list4.append([new_each_data[1], new_each_data[2]])
    #     if check_closed(temp_list3) == -1:
    #         temp_list1 = rs2.CalculateCentroid2D(temp_list3)
    #     if check_closed(temp_list4) == -1:
    #         temp_list2 = rs2.CalculateCentroid2D(temp_list4)
    #     if get_PolygonArea(temp_list3) > get_PolygonArea(temp_list4):
    #         checkValue = True
    #         result_i = get_PolygonArea(temp_list3)
    #     else:
    #         checkValue = False
    #         result_i = get_PolygonArea(temp_list4)
    #     if result_i < 1.0:
    #         print new_j, result_i
    for i in result_IOU_list:

        main = pcl.PointCloud()
        sub = pcl.PointCloud()
        main.from_list(original_data[i[0]])
        sub.from_list(new_points[i[1]])
        pcl.save(main, test_path+"main_"+str(count)+"_"+str(i[0])+".pcd")
        pcl.save(sub, test_path+"sub_"+str(count)+"_"+str(i[1])+".pcd")
        count = count + 1
        print i

    # opoints = []
    # npoints = []
    # for i in range(len(original_points)):
    #     temp_list1 = []
    #     temp_list2 = []
    #     temp_list3 = []
    #     checkValue = True
    #     for j in original_points[i]:
    #         temp_list1.append([j[0], j[2]])
    #         temp_list2.append([j[1], j[2]])
    #
        # if check_closed(temp_list1) == -1:
        #     temp_list1 = rs2.CalculateCentroid2D(temp_list1)
        # if check_closed(temp_list2) == -1:
        #     temp_list2 = rs2.CalculateCentroid2D(temp_list2)
    #
        # if get_PolygonArea(temp_list1) > get_PolygonArea(temp_list2):
        #     checkValue = True
        #     opoints.append(temp_list1)
        # else:
        #     checkValue = False
        #     opoints.append(temp_list2)
    #
        # for j in new_points[i]:
        #     if checkValue:
        #         temp_list3.append([j[0], j[2]])
        #     else:
        #         temp_list3.append([j[1], j[2]])
        #
        # if check_closed(temp_list3) == -1:
        #     temp_list3 = rs2.CalculateCentroid2D(temp_list3)
        #
        # npoints.append(temp_list3)


    # for i in new_points:
    #     temp_list = []
    #     for j in i:
    #         temp_list.append([j[0], j[2]])
    #     if check_closed(temp_list) == -1:
    #         temp_list = rs2.CalculateCentroid2D(temp_list)
    #     npoints.append(temp_list)
    #
    # for i in range(len(opoints)):
    #     print i
    #
    #     # a = get_PolygonArea(opoints[i])
    #     # b = get_PolygonArea(npoints[i])
    #     # # print i
    #     IOU = get_IOU(opoints[i], npoints[i])
    #     # print "Area_a : ", a
    #     # print "Area_b : ",b
    #     print "IOU : ", IOU


    # new_points = [[[1.17154207089495, -0.658473605762648, 3.047840118408203], [2.85423922802842, -0.658409047928616, 3.0363399982452393], [2.85423922802842, -0.658409047928616, 0.03525486961007118], [1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17154207089495, -0.658473605762648, 3.047840118408203]], [[1.17232344394907, 3.10833391622898, 3.0554399490356445], [1.17154207089495, -0.658473605762648, 3.047840118408203], [1.17154207089495, -0.658473605762648, 0.03899608179926872], [1.17232344394907, 3.10833391622898, 0.01646059937775135], [1.17232344394907, 3.10833391622898, 3.0554399490356445]], [[1.17232344394907, 3.10833391622898, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85592963672819, 3.10498754810542, 3.0554399490356445], [1.17232344394907, 3.10833391622898, 3.0554399490356445], [1.17232344394907, 3.10833391622898, 0.01646059937775135]], [[2.85592963672819, 3.10498754810542, 0.01646059937775135], [2.85423922802842, -0.658409047928616, 0.03525486961007118], [2.85423922802842, -0.658409047928616, 3.0363399982452393], [2.85592963672819, 3.10498754810542, 3.0554399490356445], [2.85592963672819, 3.10498754810542, 0.01646059937775135]]]
    # new_points = [[[-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186], [-0.0736667534394775, -0.792109886945326, 0.02443454973399639]], [[-0.0243317252695328, -3.11223151357283, 3.0621800422668457], [-0.0703359259474093, -0.948751631879418, 3.0423800945281982], [-0.0703359259474093, -0.948751631879418, 0.02952166646718979], [-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457]], [[4.69940499681119, -3.11213494616376, 3.0621800422668457], [-0.0243317252695328, -3.11223151357283, 3.0621800422668457], [-0.0243317252695328, -3.11223151357283, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69940499681119, -3.11213494616376, 3.0621800422668457]], [[4.69711197662828, 3.10132797859425, 3.0554399490356445], [4.69940499681119, -3.11213494616376, 3.0621800422668457], [4.69940499681119, -3.11213494616376, 0.03738019987940788], [4.69711197662828, 3.10132797859425, 0.01646059937775135], [4.69711197662828, 3.10132797859425, 3.0554399490356445]], [[2.99053100234752, 3.10472001183641, 3.0554399490356445], [4.69711197662828, 3.10132797859425, 3.0554399490356445], [4.69711197662828, 3.10132797859425, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.99053100234752, 3.10472001183641, 3.0554399490356445]], [[2.99053100234752, 3.10472001183641, 0.01646059937775135], [2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.98952686704443, -0.790252950611448, 3.038594961166382], [2.99053100234752, 3.10472001183641, 3.0554399490356445], [2.99053100234752, 3.10472001183641, 0.01646059937775135]], [[2.85417986314639, -0.790574479280031, 3.0363399982452393], [2.98952686704443, -0.790252950611448, 3.038594961166382], [2.98952686704443, -0.790252950611448, 0.024747176095843315], [2.85417986314639, -0.790574479280031, 0.03525486961007118], [2.85417986314639, -0.790574479280031, 3.0363399982452393]], [[1.03899695814266, -0.794886605340243, 3.0378000736236572], [2.85417986314639, -0.790574479280031, 3.0363399982452393], [2.85417986314639, -0.790574479280031, 0.03525486961007118], [1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03899695814266, -0.794886605340243, 3.0378000736236572]], [[1.03671818059138, 3.10860344786449, 3.0554399490356445], [1.03899695814266, -0.794886605340243, 3.0378000736236572], [1.03899695814266, -0.794886605340243, 0.033170800656080246], [1.03671818059138, 3.10860344786449, 0.01646059937775135], [1.03671818059138, 3.10860344786449, 3.0554399490356445]], [[-4.69822532075788, 3.12000233212, 3.0554399490356445], [1.03671818059138, 3.10860344786449, 3.0554399490356445], [1.03671818059138, 3.10860344786449, 0.01646059937775135], [-4.69822532075788, 3.12000233212, 0.01646059937775135], [-4.69822532075788, 3.12000233212, 3.0554399490356445]], [[-4.69822532075788, 3.12000233212, 0.01646059937775135], [-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-4.69805035458894, -0.792217156192569, 3.0480549335479736], [-4.69822532075788, 3.12000233212, 3.0554399490356445], [-4.69822532075788, 3.12000233212, 0.01646059937775135]], [[-0.0736667534394775, -0.792109886945326, 3.0496299266815186], [-4.69805035458894, -0.792217156192569, 3.0480549335479736], [-4.69805035458894, -0.792217156192569, 0.036078501492738724], [-0.0736667534394775, -0.792109886945326, 0.02443454973399639], [-0.0736667534394775, -0.792109886945326, 3.0496299266815186]]]

    # for i in new_points:
        # new_points_list = []
        # for j in i:
        #     new_points.append([j[0], j[2]])

    # test_points = [[3.0, 2.0], [3.8, 2.4], [3.2857142857142856, 3.4285714285714284], [2.0, 3.0], [2.0, 3.0],[3.0, 2.0]]
    # if check_closed(original_points) == -1:
    #     original_points = rs2.CalculateCentroid2D(original_points)
    #
    # if check_closed(new_points) == -1:
    #     new_points = rs2.CalculateCentroid2D(new_points)
    # a = get_PolygonArea(original_points)
    # b = get_PolygonArea(new_points)
    # # t = get_PolygonArea(test_points)
    # IOU = get_IOU(original_points, new_points)
    # print "Area_a : ", a
    # print "Area_b : ",b
    # # print "Area_t : ",t
    # print "IOU : ",IOU
    #

