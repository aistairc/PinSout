from functools import reduce
import matplotlib.pyplot as plt
import numpy as np
import math
# center_point = []
# normal_vector = []
class Point_sort:
    def __init__(self):
        self.center_point = []
        self.normal_vector = []
    def SortPointsClockwise(self, point_list, direction):

        sign = 1.0 if direction else -1.0

        self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point), np.array(point_list[1]) - np.array(self.center_point)) * float(sign)
        # print self.center_point
        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))

        new_point_list.append(new_point_list[0])
        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def SortPointsClockwise2(self, point_list, direction):

        sign = 1.0 if direction else -1.0
        # center_x = 0
        # center_y = 0
        # for i in point_list:
        #     center_x = center_x + i[0]
        #     center_y = center_y + i[1]
        # self.center_point = np.asarray([center_x, center_y])
        self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point), np.array(point_list[1]) - np.array(self.center_point)) * float(sign)

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def SortPointsClockwise3(self, point_list, direction):

        sign = 1.0 if direction else -1.0
        # center_x = 0
        # center_y = 0
        # for i in point_list:
        #     center_x = center_x + i[0]
        #     center_y = center_y + i[1]
        # self.center_point = np.asarray([center_x, center_y])
        # print centre_point
        # self.center_point = np.asarray(centre_point)
        a = np.asarray(point_list)
        x = np.mean(a.T[0])
        y = np.mean(a.T[1])
        z = np.mean(a.T[2])
        self.center_point = [x, y, z]
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point),
                                      np.array(point_list[1]) - np.array(self.center_point)) * float(sign)

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
        new_point_list.append(new_point_list[0])
        self.center_point = []
        self.normal_vector = []
        return new_point_list


    def SortPointsClockwise2D(self, point_list, direction):

        sign = 1.0 if direction else -1.0
        # center_x = 0
        # center_y = 0
        # for i in point_list:
        #     center_x = center_x + i[0]
        #     center_y = center_y + i[1]
        # self.center_point = np.asarray([center_x, center_y])
        a = np.asarray(point_list)
        x = np.mean(a.T[0])
        y = np.mean(a.T[1])
        # z = np.mean(a.T[2])
        self.center_point = [x, y]
        # self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point),
                                      np.array(point_list[1]) - np.array(self.center_point)) * float(sign)

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))

        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def SortPointsClockwise2DClose(self, point_list, direction):

        sign = 1.0 if direction else -1.0
        # center_x = 0
        # center_y = 0
        # for i in point_list:
        #     center_x = center_x + i[0]
        #     center_y = center_y + i[1]
        # self.center_point = np.asarray([center_x, center_y])
        a = np.asarray(point_list)
        x = np.mean(a.T[0])
        y = np.mean(a.T[1])
        # z = np.mean(a.T[2])
        self.center_point = [x, y]
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point),
                                      np.array(point_list[1]) - np.array(self.center_point)) * float(sign)

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
        new_point_list.append(new_point_list[0])
        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def GetIsLess(self, x, y):

        result = np.dot(self.normal_vector, np.cross(np.array(y) - np.array(self.center_point), np.array(x) - np.array(self.center_point)))
        #result = np.dot(self.normal_vector, np.cross(np.array(x) - np.array( self.center_point), np.array(y) - np.array(self.center_point)))


        if result > 0:
            return -1
        elif result < 0:
            return 1
        else:
            result2 = math.copysign(1, result)
            if result2 < 0:
                return 1
            else:
                return -1

        # else:
        #     return 0

    def cmp_to_key(self, mycmp):
        'Convert a cmp= function into a key= function'

        class K:
            def __init__(self, obj, *args):
                self.obj = obj

            def __lt__(self, other):
                return mycmp(self.obj, other.obj) < 0

            def __gt__(self, other):
                return mycmp(self.obj, other.obj) > 0

            def __eq__(self, other):
                return mycmp(self.obj, other.obj) == 0

            # def __le__(self, other):
            #     return mycmp(self.obj, other.obj) <= 0
            #
            # def __ge__(self, other):
            #     return mycmp(self.obj, other.obj) >= 0
            #
            # def __ne__(self, other):
            #     return mycmp(self.obj, other.obj) != 0

        return K




    def visual_graph(self, point_list):
        x = []
        y = []
        print point_list
        for index in point_list:
            x.append(index[0])
            y.append(index[1])
        plt.scatter(x, y, label="stars", color="green",
                    marker="*", s=50)
        plt.plot(x, y)
        plt.legend()
        plt.show()

    def visual_graph2(self, point_list):
        x = []
        y = []
        print point_list
        for index in point_list:
            x.append(index[0])
            y.append(index[2])
            plt.annotate(str(point_list.index(index)), (index[0], index[1]))
        plt.scatter(x, y, label="star", color="green",
                    marker="*", s=50)

        plt.plot(x, y)
        plt.legend()
        plt.show()

    def visual_graph3(self, point_list):

        print point_list
        for index in point_list:
            x = []
            y = []
            for each_v in index:
                x.append(each_v[0])
                y.append(each_v[1])

            plt.plot(x, y)
        plt.legend()
        plt.show()

# from functools import reduce
# import matplotlib.pyplot as plt
# import numpy as np
# import math
# # center_point = []
# # normal_vector = []
# class Point_sort:
#     def __init__(self):
#         self.center_point = []
#         self.normal_vector = []
#     def SortPointsClockwise(self, point_list, direction):
#
#         sign = 1.0 if direction else -1.0
#
#         self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
#         # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]
#
#         self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point), np.array(point_list[1]) - np.array(self.center_point)) * float(sign)
#
#         new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
#
#         new_point_list.append(new_point_list[0])
#         self.center_point = []
#         self.normal_vector = []
#         return new_point_list
#
#     def SortPointsClockwise2(self, point_list, direction):
#
#         sign = 1.0 if direction else -1.0
#
#         self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
#         # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]
#
#         self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point), np.array(point_list[1]) - np.array(self.center_point)) * float(sign)
#
#         new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
#         # new_point_list = sorted(point_list, key=self.GetIsLess)
#         # new_point_list = sorted(point_list, key=self.GetIsLess(x, y))
#         self.center_point = []
#         self.normal_vector = []
#         return new_point_list
#
#     def GetIsLess(self, x, y):
#
#         result = np.dot(self.normal_vector, np.cross(np.array(x) - np.array(self.center_point), np.array(y) - np.array(self.center_point)))
#         result_1 = np.dot(np.array(x) - np.array(self.center_point), np.array(y) - np.array(self.center_point))
#         result_2 = np.linalg.norm(np.array(x) - np.array(self.center_point))
#
#         result_3 = np.linalg.norm(np.array(y) - np.array(self.center_point))
#         result_4 = result_1 / (result_3 * result_2)
#         result_6 = math.acos(result_4)
#
#         result_5 = result * result_6
#         a = np.array(x) - np.array(self.center_point)
#         c = math.pow(a[0], 2) + math.pow(a[1], 2)
#         print result_2, math.sqrt(c)
#
#         if result > 0:
#             return 1
#         else:
#             return -1
#         # print result, result_4, result_
#         # result = np.dot(self.normal_vector, np.cross(np.array(y) - np.array(self.center_point), np.array(x) - np.array(self.center_point)))
#         # if result > 0:
#         #     return -1
#         # else:
#         #     return 1
#
#
#     def cmp_to_key(self, mycmp):
#         'Convert a cmp= function into a key= function'
#
#         class K:
#             def __init__(self, obj, *args):
#                 self.obj = obj
#
#             def __lt__(self, other):
#                 print "__lt__", self.obj, other.obj, mycmp(self.obj, other.obj)
#                 return mycmp(self.obj, other.obj) < 0
#
#             def __gt__(self, other):
#                 print "__gt__", self.obj, other.obj, mycmp(self.obj, other.obj)
#                 return mycmp(self.obj, other.obj) > 0
#
#             def __eq__(self, other):
#                 print "__eq__", self.obj, other.obj, mycmp(self.obj, other.obj)
#                 return mycmp(self.obj, other.obj) == 0
#             # def __lt__(self, other):
#             #     return mycmp(self.obj, other.obj) < 0
#             #
#             # def __gt__(self, other):
#             #     return mycmp(self.obj, other.obj) > 0
#             #
#             # def __eq__(self, other):
#             #     return mycmp(self.obj, other.obj) == 0
#
#
#             # def __le__(self, other):
#             #     return mycmp(self.obj, other.obj) <= 0
#             #
#             # def __ge__(self, other):
#             #     return mycmp(self.obj, other.obj) >= 0
#             #
#             # def __ne__(self, other):
#             #     return mycmp(self.obj, other.obj) != 0
#
#         return K
#
#
#     def visual_graph(self, point_list):
#         fig = plt.figure()
#         ax = fig.add_subplot(111)
#         x = []
#         y = []
#         for index in point_list:
#             x.append(index[1])
#             y.append(index[2])
#
#         plt.plot(x, y)
#         zip_point = zip(x, y)
#         for xy in range(len(zip_point)):
#             ax.annotate('(%s)' % xy, xy=zip_point[xy], textcoords='data')
#
#         plt.legend()
#         plt.show()
#         # x = []
#         # y = []
#         # for index in point_list:
#         #     x.append(index[1])
#         #     y.append(index[2])
#         # plt.scatter(x, y, label="stars", color="green",
#         #             marker="*", s=50)
#         # plt.plot(x, y)
#         # plt.legend()
#         # plt.show()
#
#
# #
# # A = [-0.75, -0.25, 0, 0.25, 0.5, 0.75, 1.0]
# # B = [0.73, 0.97, 1.0, 0.97, 0.88, 0.73, 0.54]
# #
# # plt.plot(A,B)
# # a = zip(A, B)
# # for xy in range(len(a)):
# #     print xy
# #     ax.annotate('(%s)' % xy, xy=a[xy], textcoords='data')
# #
# # plt.grid()
# # plt.show()
#
#
# # https://stackoverflow.com/questions/14370636/sorting-a-list-of-3d-coplanar-points-to-be-clockwise-or-counterclockwise
