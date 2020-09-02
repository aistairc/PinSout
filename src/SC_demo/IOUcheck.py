import Point_sort2 as ps2

def check_closed(points):

    points_length = len(points) - 1
    if (points[0][0] != points[points_length][0] or points[0][1] != points[points_length][1]):
        print "Last vertex and first vertex are not connected."
        return False
    else:
        return True

def ccw(p1,  p2,  p3):

    cross_product = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p3[0] - p1[0])*(p2[1] - p1[1])

    if (cross_product > 0):
        return 1

    elif (cross_product < 0):
        return -1
    else:
        return 0



def check_LineIntersection(l1, l2):


    l1_l2 = ccw(l1[0], l1[1], l2[0]) * ccw(l1[0], l1[1], l2[1])

    l2_l1 = ccw(l2[0], l2[1], l1[0]) * ccw(l2[0], l2[1], l1[1])

    rect = (l1_l2 < 0) and (l2_l1 < 0)

    return rect


def Min(p1, p2):

    return p1 if p1 < p2 else p2

def Max(p1, p2):
    return p1 if p1 > p2 else p2

def check_PolygonInOut(point, points):


    rect = 0

    for p_i in range(len(points) - 1):
        if (ccw( points[p_i], points[p_i+1], point) == 0 ):

            min_x = Min(points[p_i][0], points[p_i + 1][0])
            max_x = Max(points[p_i][0], points[p_i + 1][0])
            min_y = Min(points[p_i][1], points[p_i + 1][1])
            max_y = Max(points[p_i][1], points[p_i + 1][1])


            if(min_x <= point[0] and point[0] <= max_x) and (min_y <= point[1] and point[1] <= max_y):
                return 1


    outside_point = [1, 1234567]
    outside_line = []
    outside_line.append(outside_point)
    outside_line.append(point)

    for i in range(len(points) - 1):

        temp_line = []
        temp_line.append(points[i])
        temp_line.append(points[i + 1])
        rect = rect + check_LineIntersection(outside_line, temp_line)

    rect = rect % 2

    return rect


def get_IntersectionPoint(p1,p2,p3,p4):

    x = ((p1[0]*p2[1] - p1[1]*p2[0])*(p3[0] - p4[0]) - (p1[0] - p2[0])*(p3[0]*p4[1] - p3[1]*p4[0]))/( (p1[0] - p2[0])*(p3[1] - p4[1]) - (p1[1] - p2[1])*(p3[0] - p4[0]) )
    y = ((p1[0]*p2[1] - p1[1]*p2[0])*(p3[1] - p4[1]) - (p1[1] - p2[1])*(p3[0]*p4[1] - p3[1]*p4[0])) / ( (p1[0] - p2[0])*(p3[1] - p4[1]) - (p1[1] - p2[1])*(p3[0] - p4[0]) )
    rect = []
    rect.append(x)
    rect.append(y)

    return rect


def get_PolygonArea(points):

    rect = 0.0

    for i in range(len(points) - 1):
        rect = rect + points[i][0] * points[i+1][1] - points[i][1] * points[i+1][0]

    rect = rect if rect > 0 else -rect
    rect = rect / 2.0

    return rect


def get_Intersection(original_points, new_points):

    intersection_points = []

    for original_i in range(len(original_points) - 1):

        l1 = []
        l1.append(original_points[original_i])
        l1.append(original_points[original_i + 1])

        for new_i in range(len(new_points) - 1):
            l2 = []
            l2.append(new_points[new_i])
            l2.append(new_points[new_i + 1])

            if check_LineIntersection(l1, l2):

                intersection_points.append(get_IntersectionPoint(l1[0], l1[1], l2[0], l2[1]))

    for i in range(len(original_points) - 1):
        if check_PolygonInOut(original_points[i], new_points):
            intersection_points.append(original_points[i])

    for i in range(len(new_points) - 1):
        if check_PolygonInOut(new_points[i], orignal_points):
            intersection_points.append(new_points[i])

    if check_closed(intersection_points) == False:
        intersection_points = ps2.CalculateCentroid2D(intersection_points)

    print intersection_points
    rect = get_PolygonArea(intersection_points)

    return rect



def GetIoU(original_points, new_points):



    intersection_area = get_Intersection(original_points, new_points)
    A = get_PolygonArea(original_points)
    B = get_PolygonArea(new_points)
    print intersection_area
    union_area = A + B - intersection_area

    rect = intersection_area / union_area

    return rect


if __name__ == '__main__':


    orignal_points = []
    orignal_points.append([1.0, 2.0])
    orignal_points.append([3.0, 1.0])
    orignal_points.append([4.0, 2.0])
    orignal_points.append([3.0, 4.0])
    orignal_points.append([1.0, 3.0])

    new_points = []
    new_points.append([2.0, 3.0])
    new_points.append([3.0, 2.0])
    new_points.append([5.0, 3.0])
    new_points.append([5.0, 4.0])

    if check_closed(orignal_points) == False:
        orignal_points = ps2.CalculateCentroid2D(orignal_points)
    if check_closed(new_points) == False:
        new_points = ps2.CalculateCentroid2D(new_points)

    orignal_area = get_PolygonArea(orignal_points)
    new_area = get_PolygonArea(new_points)

    print orignal_area
    print new_area
    print "IoU : ", GetIoU(orignal_points,new_points)