import math
a=[ [3.9444040363752, 5.70785470504333], [6.98157308188523, 5.65679927725593],[1.21091214887766, 5.75380525819936]]


def sorted2Dpoints(point_list):
    max = -1.0
    index = []
    sorted_list = []
    for i in range(len(point_list) -1):
        for j in range(1, len(point_list)):
            distance = math.sqrt(((point_list[i][0] - point_list[j][0]) ** 2) + ((point_list[i][1] - point_list[j][1]) ** 2))
            if distance > max:
                max = distance
                index = []
                index.append(i)
                index.append(j)
    value0 = point_list[index[0]]
    value1 = point_list[index[1]]
    if index[0] < index[1]:
        point_list.pop(index[0])
        point_list.pop(index[1]-1)
    else:
        point_list.pop(index[1])
        point_list.pop(index[0])

    sorted_list.append(value0)
    sorted_list.append(value1)
    while True:
        if len(point_list) == 0:
            break
        min = float("inf")
        index = -1
        for i in range(len(point_list)):
            distance = math.sqrt(
                ((sorted_list[0][0] - point_list[i][0]) ** 2) + ((sorted_list[0][1] - point_list[i][1]) ** 2))
            if distance < min:
                min = distance
                index = i
            print min, index
        sorted_list.insert(len(sorted_list)-1, point_list[index])
        point_list.pop(index)

    return sorted_list
print sorted2Dpoints(a)