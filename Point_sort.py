from functools import reduce
import matplotlib.pyplot as plt
import numpy as np

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

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))

        new_point_list.append(new_point_list[0])
        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def SortPointsClockwise2(self, point_list, direction):

        sign = 1.0 if direction else -1.0

        self.center_point = (reduce(lambda x, y: np.array(x) + np.array(y), point_list)) / float(len(point_list))
        # center_point = [center_point_0[0] / float(len(point_list)), center_point_0[1] / float(len(point_list)), center_point_0[2] / float(len(point_list))]

        self.normal_vector = np.cross(np.array(point_list[0]) - np.array(self.center_point), np.array(point_list[1]) - np.array(self.center_point)) * float(sign)

        new_point_list = sorted(point_list, key=self.cmp_to_key(self.GetIsLess))
        self.center_point = []
        self.normal_vector = []
        return new_point_list

    def GetIsLess(self, x, y):

        result = np.dot(self.normal_vector, np.cross(np.array(x) - np.array( self.center_point), np.array(y) - np.array(self.center_point)))
        if result > 0:
            return -1
        else:
            return 1

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
        for index in point_list:
            x.append(index[1])
            y.append(index[2])
        plt.scatter(x, y, label="stars", color="green",
                    marker="*", s=50)
        plt.plot(x, y)
        plt.legend()
        plt.show()



